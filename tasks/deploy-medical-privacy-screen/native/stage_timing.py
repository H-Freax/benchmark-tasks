"""Read-only wall timing; pure stdlib, lazy existing-instance installation."""
from functools import wraps
from pathlib import Path
import json,threading,time,sys
_LOCAL=threading.local();_LOCK=threading.RLock();_DATA={};_TRACE=None;_STATE=lambda:{};_LAST_FLUSH_NS=0;_LOG_ERRORS=0;_AVAILABILITY={}

def configure(trace,state):
 global _TRACE,_STATE
 _TRACE=Path(trace);_STATE=state

class span:
 def __init__(self,label):self.label=label
 def __enter__(self):
  self.start=time.perf_counter_ns();self.children=0
  stack=getattr(_LOCAL,'stack',None)
  if stack is None:stack=[];_LOCAL.stack=stack
  stack.append(self);return self
 def __exit__(self,typ,exc,tb):
  end=time.perf_counter_ns();stack=_LOCAL.stack;assert stack[-1]is self;stack.pop();duration=end-self.start
  if stack:stack[-1].children+=duration
  with _LOCK:
   row=_DATA.setdefault(self.label,dict(calls=0,inclusive_ns=0,exclusive_ns=0,max_ns=0,exceptions=0))
   row['calls']+=1;row['inclusive_ns']+=duration;row['exclusive_ns']+=max(0,duration-self.children);row['max_ns']=max(row['max_ns'],duration);row['exceptions']+=int(typ is not None)
  return False

def timed(label,flush_after=False,flush_error=False):
 def decorate(fn):
  @wraps(fn)
  def call(*args,**kwargs):
   try:
    with span(label):return fn(*args,**kwargs)
   finally:
    if flush_after or flush_error and sys.exc_info()[0]is not None:flush()
  return call
 return decorate

def flush():
 global _LAST_FLUSH_NS,_LOG_ERRORS
 if _TRACE is None:return
 start=time.perf_counter_ns()
 try:
  with _LOCK:
   data={k:dict(v)for k,v in _DATA.items()};_DATA.clear()
  row=dict(perf_counter_ns=start,stages=data,previous_timing_flush_ns=_LAST_FLUSH_NS,timing_log_errors=_LOG_ERRORS,timing_targets={k:dict(v)for k,v in _AVAILABILITY.items()},**_STATE())
  _TRACE.mkdir(parents=True,exist_ok=True)
  with(_TRACE/'stage-timing.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 except Exception:_LOG_ERRORS+=1
 finally:_LAST_FLUSH_NS=time.perf_counter_ns()-start

def _target_status(label,obj,status,error=None):
 with _LOCK:
  previous=_AVAILABILITY.get(label,{})
  _AVAILABILITY[label]=dict(status=status,actual_type=type(obj).__module__+'.'+type(obj).__qualname__,attempts=previous.get('attempts',0)+1,error_type=type(error).__name__ if error is not None else None)
  current=_AVAILABILITY[label];current['unavailable_seen']=previous.get('unavailable_seen',False)or status!='available'
  current['first_unavailable_type']=previous.get('first_unavailable_type')or(current['actual_type']if status!='available'else None)
  current['first_unavailable_status']=previous.get('first_unavailable_status')or(status if status!='available'else None)

def _is_placeholder(obj):
 # Builtin type/MRO check: never accesses a placeholder instance attribute.
 return obj is None or issubclass(type(obj),(str,bytes))

def _wrap_instance(obj,name,label):
 # Probe/setup failures are optional diagnostics. The wrapped call is OUTSIDE
 # these try blocks, so actual physics/policy exceptions are never swallowed.
 if _is_placeholder(obj):_target_status(label,obj,'unavailable_placeholder');return False
 try:old=getattr(obj,name)
 except Exception as error:_target_status(label,obj,'unavailable_lookup',error);return False
 if _is_placeholder(old):_target_status(label,old,'unavailable_placeholder');return False
 if not callable(old):_target_status(label,obj,'unavailable_noncallable');return False
 if getattr(old,'_p6_timing_label',None)==label:_target_status(label,obj,'available');return True
 wrapped=timed(label)(old);wrapped._p6_timing_label=label
 try:setattr(obj,name,wrapped)
 except Exception as error:_target_status(label,obj,'unavailable_readonly',error);return False
 _target_status(label,obj,'available');return True

def _install_path(root,path,label):
 obj=root
 for name in path[:-1]:
  if _is_placeholder(obj):_target_status(label,obj,'unavailable_placeholder');return False
  try:obj=getattr(obj,name)
  except Exception as error:_target_status(label,obj,'unavailable_container',error);return False
 return _wrap_instance(obj,path[-1],label)

def install_environment(env):
 # Only existing targets are probed. A configuration placeholder may not yet
 # resolve to the actual backend; missing spans are honest missing data.
 for path,label in [
  (('unwrapped','sim','step'),'physics.sim_step_total'),
  (('unwrapped','sim','render'),'render.total'),
  (('unwrapped','sim','physics_manager','step'),'physics.backend_simulate_fetch_total'),
  (('step',),'environment.control_step_total'),
  (('unwrapped','scene','write_data_to_sim'),'scene.write_data_total'),
  (('unwrapped','scene','update'),'scene.update_total')]:
  _install_path(env,path,label)

def install_embodiment(cls):
 if _is_placeholder(cls):_target_status('embodiment.class',cls,'unavailable_placeholder');return
 if getattr(cls,'_p6_stage_timing',False):return
 old=cls._ensure_sim
 @wraps(old)
 def ensure(self,*args,**kwargs):
  with span('embodiment.ensure_sim'):
   out=old(self,*args,**kwargs)
   if self._env is not None:install_environment(self._env)
   return out
 cls._ensure_sim=ensure
 cls._task_step=timed('embodiment.task_step_total')(cls._task_step)
 cls._observation=timed('embodiment.observation_camera_total')(cls._observation)
 old_step=cls.step
 @wraps(old_step)
 def step(self,*args,**kwargs):
  if self._env is not None:install_environment(self._env)
  return old_step(self,*args,**kwargs)
 cls.step=timed('embodiment.step_total',flush_after=True)(step)
 cls._p6_stage_timing=True

def install_policy(mod):
 if _is_placeholder(mod):_target_status('policy.module',mod,'unavailable_placeholder');return
 cls=mod.YamScriptedPolicy
 if _is_placeholder(cls):_target_status('policy.class',cls,'unavailable_placeholder');return
 if not getattr(cls,'_p6_stage_timing',False):
  cls.act=timed('policy.act_total',flush_error=True)(cls.act);cls._p6_stage_timing=True
