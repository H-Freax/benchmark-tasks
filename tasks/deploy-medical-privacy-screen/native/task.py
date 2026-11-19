"""Full-size three-panel medical privacy screen prefix, read-only native state audit; no body writers."""
from pathlib import Path
import stage_timing as st
import os,sys,json,hashlib,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import native_geometry as ng
from observation_marks import with_observation_marks
GROOT=Path(os.environ.get('ROBOCURVE_GEOMETRY_ROOT',str(HERE.parent/'geometry')));G=with_observation_marks(json.loads((GROOT/'geometry.json').read_text()));GSHA=hashlib.sha256(json.dumps(G,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
from yam_arena import station
station.BASES={s:tuple(v)for s,v in G['robot_bases_m'].items()}
OBJECTS=ng.objects(G);SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=.05;CONTROL_HZ=20.
TRACE=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(HERE/'trace')));CONTROL=0;LAST_ACTUAL=None;_COMMAND=None;RESET_LAST=None;PHASE_PROVIDER=lambda:'model'
from primary_score import PrimaryScore
PRIMARY=PrimaryScore()
from contact_readback import ContactReadback
CONTACTS=ContactReadback(sys.modules[__name__])
def arr(x):
 x=getattr(x,'torch',x)
 if hasattr(x,'detach'):return x.detach().cpu().numpy().copy()
 if hasattr(x,'numpy'):return x.numpy().copy()
 return np.array(x)
def save(name,j):TRACE.mkdir(parents=True,exist_ok=True);(TRACE/name).write_text(json.dumps(j,indent=2,allow_nan=False)+'\n')
@ng.pk.on_reset
def reset(env):
 global CONTROL,LAST_ACTUAL,_COMMAND,RESET_LAST
 CONTROL=0;LAST_ACTUAL=None;_COMMAND=None;RESET_LAST=None;PRIMARY.reset();CONTACTS.reset()
 record_reset_actual(env.unwrapped,'on_reset')
def record_reset_actual(base,event):
 global RESET_LAST
 step=int(getattr(base,'_sim_step_counter',-1))
 if RESET_LAST==step:return
 RESET_LAST=step;row=dict(control=0,physics_step=step,event=event,phase='reset_settle',command14=None,geometry_sha256=GSHA,object_pose_writes_after_reset=0,score_evaluated=False)
 try:
  row['rigid_bodies']={p['name']:dict(pose_xyzw=arr(base.scene[p['name']].root_view.get_transforms())[0].tolist(),native_velocity=arr(base.scene[p['name']].root_view.get_velocities())[0].tolist())for p in G['parts']}
  json.dumps(row,allow_nan=False)
 except Exception as e:
  row.pop('rigid_bodies',None);row['unknown']=type(e).__name__+':'+str(e)
 TRACE.mkdir(parents=True,exist_ok=True)
 with(TRACE/'reset-actual.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
@ng.pk.on_substep
def reset_substep(env):
 if _COMMAND is None:record_reset_actual(env.unwrapped,'reset_physics_substep')
@ng.pk.on_step
@st.timed("observe.total")
def observe(env):
 global CONTROL,LAST_ACTUAL
 if _COMMAND is None:record_reset_actual(env.unwrapped,'command_none_control');return
 base=env.unwrapped;CONTROL+=1;bodies={};robots={}
 with st.span('observe.body_readback'):
  for p in G['parts']:
   v=base.scene[p['name']].root_view;bodies[p['name']]=dict(pose_xyzw=arr(v.get_transforms())[0].tolist(),native_velocity=arr(v.get_velocities())[0].tolist())
 with st.span('observe.robot_readback'):
  for s in ['left','right']:
   a=base.scene[s+'_robot'];v=a.root_view;robots[s]=dict(joint_names=list(a.backend_joint_names),joint_positions=arr(v.get_dof_positions())[0].tolist(),body_names=list(a.backend_body_names),body_link_pose_w=arr(v.get_link_transforms())[0].tolist())
 row=dict(control=CONTROL,time_s=CONTROL/20,physics_step=int(base._sim_step_counter),geometry_sha256=GSHA,phase=PHASE_PROVIDER(),command14=_COMMAND,rigid_bodies=bodies,robots=robots,object_pose_writes_after_reset=0,external_task_wrenches=0)
 with st.span('observe.contact_readback'):
  row['contacts']=CONTACTS.observe(base)
 with st.span('observe.scoring'):
  row['primary']=PRIMARY.observe(row,G)
 row['score']=PRIMARY.value;LAST_ACTUAL=row;TRACE.mkdir(parents=True,exist_ok=True)
 with st.span('observe.trace_write'):
  with(TRACE/'observations.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 if CONTROL==1:
  from pxr import Usd,UsdPhysics
  root=base.scene.env_prim_paths[0];inventory=[];shapes=[];robotshapes=[];bodies_actual={};problems=[]
  for prim in base.scene.stage.Traverse(Usd.TraverseInstanceProxies()):
   path=str(prim.GetPath())
   if prim.IsA(UsdPhysics.Joint):inventory.append(path)
   if prim.HasAPI(UsdPhysics.CollisionAPI):
    if any(path.startswith(root+'/'+p['name']+'/')for p in G['parts']):shapes.append(dict(path=path,source=prim.GetCustomDataByKey('source_geometry_name')))
    elif '/LeftArm/'in path or '/RightArm/'in path:robotshapes.append(path)
  for p in G['parts']:
   prim=base.scene.stage.GetPrimAtPath(root+'/'+p['name']+'/body');v=base.scene[p['name']].root_view
   rigid=bool(prim.IsValid()and prim.HasAPI(UsdPhysics.RigidBodyAPI));actual_mass=None;count=None
   try:actual_mass=float(np.asarray(arr(v.get_masses())).reshape(-1)[0])
   except Exception as e:problems.append(p['name']+':mass:'+str(e))
   try:count=int(v.max_shapes)
   except Exception as e:problems.append(p['name']+':shapes:'+str(e))
   body_shapes=[x for x in shapes if x['path'].startswith(root+'/'+p['name']+'/')]
   if not rigid:problems.append(p['name']+':missingRigidBodyAPI')
   if actual_mass is None or not np.isclose(actual_mass,p['mass_kg'],rtol=1e-5):problems.append(p['name']+':massmismatch')
   if len(body_shapes)!=sum(sh['collision_enabled']for sh in p['geoms']):problems.append(p['name']+':collisioncountmismatch')
   bodies_actual[p['name']]=dict(rigid_body_API=rigid,mass_kg=actual_mass,shape_count=count,fixed=p['fixed'],actual_collision_shapes=len(body_shapes))
  passed=bool(not problems and len(bodies_actual)==len(G['parts'])and len(shapes)==sum(sh['collision_enabled']for p in G['parts']for sh in p['geoms']))
  save('NATIVE-INVENTORY.json',dict(passed=passed,rigid_only=all(v['rigid_body_API']for v in bodies_actual.values()),geometry_sha256=GSHA,bodies=bodies_actual,shapes=shapes,joints=inventory,robot_colliders=robotshapes,problems=problems,physics_dt=float(base.physics_dt),control_dt=float(base.step_dt),object_pose_writes_after_reset=0,external_task_wrenches=0));base.scene.stage.Export(str(TRACE/'authored.usda'))
 if CONTROL%20==0:print('TERMINAL_ACTUAL '+json.dumps(dict(control=CONTROL,phase=row['phase'],primary=row['primary'])),flush=True)
prev=ng.pk._Finder.targets['yam_arena.scene']
def patch_scene(mod):
 prev(mod)
 if not getattr(mod,'_screen_no_table',False):
  oldroom=mod.task_room
  def task_room(scene,top_cam):return[a for a in oldroom(scene,top_cam)if a.name!='table']
  mod.task_room=task_room;mod._screen_no_table=True
 if getattr(mod,'_terminal_timing',False):return
 old=mod.env_cfg_callback
 def cb(control_hz=None):
  fn=old(control_hz)
  def config(cfg):cfg=fn(cfg);cfg.sim.dt=1/960;cfg.decimation=48;cfg.sim.render_interval=48;return cfg
  return config
 mod.env_cfg_callback=cb;mod._terminal_timing=True
ng.pk._Finder.targets['yam_arena.scene']=patch_scene
oldie=ng.pk._Finder.targets['yam_arena.inspect_embodiment']
def patch_ie(mod):
 oldie(mod);cls=mod.YamArenaEmbodiment
 if getattr(cls,'_terminal_command',False):return
 old=cls.step
 def step(self,action):
  global _COMMAND
  _COMMAND=np.asarray(action.data,float).reshape(-1).tolist();return old(self,action)
 cls.step=step;cls._terminal_command=True
 st.install_embodiment(cls)
ng.pk._Finder.targets['yam_arena.inspect_embodiment']=patch_ie
for name,fun in [('yam_arena.scene',patch_scene),('yam_arena.inspect_embodiment',patch_ie)]:
 if name in sys.modules:fun(sys.modules[name])
def score(states):return PRIMARY.value
def success(states):return PRIMARY.value>=1.

st.configure(TRACE,lambda:dict(control=CONTROL,physics_step=(LAST_ACTUAL or{}).get("physics_step"),phase=(LAST_ACTUAL or{}).get("phase")))
_previous_scripted_patch=ng.pk._Finder.targets.get("yam_arena.scripted")
def _install_stage_policy(mod):
 if _previous_scripted_patch is not None:_previous_scripted_patch(mod)
 st.install_policy(mod)
ng.pk._Finder.targets["yam_arena.scripted"]=_install_stage_policy
if "yam_arena.scripted"in sys.modules:_install_stage_policy(sys.modules["yam_arena.scripted"])
