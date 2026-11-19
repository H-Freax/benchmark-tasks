"""Shortread-onlypassivereset: armsremainREADY, no referenceworkclaimed."""
from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import READY,FINGER_OPEN
import task as rt
rt.PHASE_PROVIDER=lambda:'passive_reset'
def demo():return {s:[Waypoint(30.,(.3,0,.9),FINGER_OPEN,'valve_passive_'+s)]for s in['left','right']}
old=ArmPlan.action7
def action(plan,t):
 label=plan.waypoints[0].label if plan.waypoints else''
 if label.startswith('valve_passive_'):return np.r_[READY,1.],'passive_reset'
 return old(plan,t)
ArmPlan.action7=action
