"""Complete ordinary reference with original-handle grasp and material feedback."""
from pathlib import Path
import json,sys
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from task import *
import task as rt
from frame_controller import Controller
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import FINGER_OPEN
PLAN=json.loads((rt.GROOT/'reference_targets.json').read_text());CONTROL=Controller(rt,PLAN)
def demo():return {a:[Waypoint(210.,(.2,0,.8),FINGER_OPEN,'bottle_reference_'+a)]for a in ['left','right']}
_original=ArmPlan.action7
def dispatch(plan,t):
 if plan.waypoints and plan.waypoints[0].label.startswith('bottle_reference_'):
  side=plan.waypoints[0].label.rsplit('_',1)[1]
  return CONTROL.action(side,t)
 return _original(plan,t)
ArmPlan.action7=dispatch
# A failed required grasp requests a normal bounded policy stop after10HOLDs.
from yam_arena.scripted import YamScriptedPolicy
_old_act=YamScriptedPolicy.act
def policy_act(self,observation):
 result=_old_act(self,observation)
 if CONTROL.stop_requested:
  for action in result.actions:action.meta['request_stop']=True
 return result
YamScriptedPolicy.act=policy_act
