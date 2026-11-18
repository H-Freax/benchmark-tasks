"""Privileged ordinary robot reference. No object or joint state setters."""
from pathlib import Path
import sys,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from task import OBJECTS,SCENE,SCORE_OVER,CONTROL_HZ,SUCCESS_HOLD_S,score,success
import native as rt
from reference import Controller
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import FINGER_OPEN
CTRL=Controller(rt)
def demo():return {s:[Waypoint(240.,(.2,0,.9),FINGER_OPEN,'phase_drain_'+s)]for s in ['left','right']}
old=ArmPlan.action7
def action(plan,t):
 label=plan.waypoints[0].label if plan.waypoints else''
 if label.startswith('phase_drain_'):
  before=CTRL.index;CTRL.action(t)
  if CTRL.index!=before:
   rt.TRACE.mkdir(parents=True,exist_ok=True)
   record=dict(control_index=CTRL.index,phase=CTRL.phase,stage=CTRL.stage,phase_start_control=CTRL.start,done=CTRL.done,failure=CTRL.failure,q={s:q.tolist()for s,q in CTRL.q.items()},grip=CTRL.grip,target_valve_angle_rad=CTRL.target_angle,last_actual_control=rt.LAST_ROW.get('control')if rt.LAST_ROW else None,valve_settle_diagnostic=CTRL.valve_settle_diagnostic)
   with(rt.TRACE/'controller.jsonl').open('a')as f:f.write(json.dumps(record,allow_nan=False)+'\n')
   if CTRL.done:
    terminal=rt.TRACE/'REFERENCE-TERMINAL.json'
    if not terminal.exists():terminal.write_text(json.dumps(record,indent=2)+'\n')
  s=label.rsplit('_',1)[1];return np.r_[CTRL.q[s],CTRL.grip[s]],CTRL.phase
 return old(plan,t)
ArmPlan.action7=action

# End the ordinary reference when its complete controller sequence ends.
# This is a transport stop, never a task-success or score override.
def patch_policy_stop(mod):
 cls=mod.YamScriptedPolicy
 if getattr(cls,'_phase_drain_stop',False):return
 old_act=cls.act
 def act(self,observation):
  chunk=old_act(self,observation)
  if CTRL.done and any(p.waypoints and p.waypoints[0].label.startswith('phase_drain_')for p in self._plans.values()):
   for action in chunk.actions:action.meta.update(request_stop=True,stop_reason='reference_sequence_ended',stop_detail=CTRL.failure or 'all_reference_stages_attempted')
  return chunk
 cls.act=act;cls._phase_drain_stop=True
rt.pk._Finder.targets['yam_arena.scripted']=patch_policy_stop
if 'yam_arena.scripted'in sys.modules:patch_policy_stop(sys.modules['yam_arena.scripted'])
