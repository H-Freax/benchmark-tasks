"""Ordinary bounded fullsettingattempt; baselinepartialscore notqualifiedtask."""
from pathlib import Path
import sys,json,math,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import task_runtime as rt
from task_runtime import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import READY,FINGER_OPEN
from material_grasp import MaterialGraspGuard,GraspFailure
from punch_alignment import PunchAlignment,collision_guard
GRASP_GUARD=MaterialGraspGuard()
ALIGNER=PunchAlignment(rt.G);LEFT_DELTA=np.zeros(6)
PLAN=json.load(open(rt.GROOT/'full_plan.json'))['rows'];LAST=-1;FAILED=None;HOLDCOUNT=0;STOP=False;LASTCMD={s:np.r_[READY,1.]for s in ['left','right']};PHASE='initial_settle'
rt.PHASE_PROVIDER=lambda:PHASE

def demo():return{s:[Waypoint(100.,(.3,0,.9),FINGER_OPEN,'diabfull_'+s)]for s in ['left','right']}
def contacts(side,row):
 keys=[side+'_tip_left',side+'_tip_right'];pairs=row['contacts']['pairs']
 return all(k in pairs and'counts'in pairs[k]and int(np.array(pairs[k]['counts']).sum())>0 and sum(abs(x['normal_force_N'])for x in pairs[k].get('entries',[]))>.05 for k in keys)
def decide(t):
 global LAST,FAILED,HOLDCOUNT,STOP,PHASE,LEFT_DELTA
 index=max(0,int(math.floor((t+1e-8)*20)))
 if index==LAST:return
 LAST=index;i=0 if index<20 else min(index-20,len(PLAN)-1);row=PLAN[i];PHASE='initial_settle'if index<20 else row['phase'];actual=rt.LAST_ACTUAL
 if actual is not None:
  if not all(np.isfinite(v['pose_xyzw']).all()for v in actual['rigid_bodies'].values()):FAILED='Nonfiniteactualbody'
  if any(not np.isfinite(v['joint_positions']).all()for v in actual['robots'].values()):FAILED='Nonfiniteactualrobot'
  # Contactexistence is not forcecalibration; unknown/failed views failclosed on loaded actions.
  loaded_left=PHASE not in ['initial_settle','punch_hover','punch_approach','punch_close','punch_release','punch_retreat','released_terminal_hold']
  loaded_right=PHASE in ['hammer_lift','hammer_transit','hammer_return_high','hammer_return']or PHASE.startswith(('strike_','strike_settle_','relift_','punch_align_'))
  if loaded_left and not contacts('left',actual):FAILED='Leftactualpunchgraspabsent'
  if loaded_right and not contacts('right',actual):FAILED='Rightactualhammergraspabsent'
  if not FAILED:
   try:
    measurements=[]
    if loaded_left:measurements.append(GRASP_GUARD.observe(actual,'left','punch'))
    if loaded_right:measurements.append(GRASP_GUARD.observe(actual,'right','hammer'))
    if measurements:
     rt.TRACE.mkdir(parents=True,exist_ok=True)
     with(rt.TRACE/'material-grasp.jsonl').open('a')as f:f.write(json.dumps(dict(control=index,measurements=measurements))+'\n')
   except (ValueError,GraspFailure)as e:
    FAILED=type(e).__name__+':'+str(e)
    if hasattr(e,'measurement'):rt.save('GRASP-GUARD-FAILURE.json',e.measurement)
 if FAILED:
  PHASE='guard_hold';HOLDCOUNT+=1;STOP=HOLDCOUNT>=20
 else:
  priorcmd={s:v.copy()for s,v in LASTCMD.items()}
  for s in LASTCMD:LASTCMD[s]=np.r_[row[s+'_q'],row[s+'_grip']]
  try:
   if PHASE.startswith('punch_align_'):
    if actual is None:raise ValueError('Missingactualalignmentreadback')
    corrected,measurement=ALIGNER.update(actual,row['left_q'],PHASE)
    LEFT_DELTA=corrected-np.array(row['left_q'])
    with(rt.TRACE/'punch-alignment.jsonl').open('a')as f:f.write(json.dumps(measurement)+'\n')
    if i+1<len(PLAN)and PLAN[i+1]['phase']!=PHASE and ALIGNER.streak<5:raise ValueError('Actualpunchalignmentdidnotsettle')
   weight=1.
   if PHASE=='punch_withdraw':
    first=next(k for k,v in enumerate(PLAN)if v['phase']=='punch_withdraw')
    u=min(1.,(i-first+1)/70.);weight=1.-u*u*(3.-2.*u)
   elif PHASE in ['punch_stow_high','punch_lower_to_vrest','punch_release','punch_retreat','released_terminal_hold']:weight=0.
   proposed=np.array(row['left_q'])+weight*LEFT_DELTA
   if np.linalg.norm(LEFT_DELTA)>1e-10:
    collision_guard(actual,proposed,rt.G)
   LASTCMD['left']=np.r_[proposed,row['left_grip']]
  except ValueError as e:
   for s in LASTCMD:LASTCMD[s]=priorcmd[s]
   FAILED=type(e).__name__+':'+str(e);PHASE='guard_hold';HOLDCOUNT=1
  STOP=index>=20+len(PLAN)
 trace=dict(control=index,phase=PHASE,failed=FAILED,stop_requested=STOP,plan_index=i)
 rt.TRACE.mkdir(parents=True,exist_ok=True)
 with(rt.TRACE/'controller.jsonl').open('a')as f:f.write(json.dumps(trace)+'\n')
old=ArmPlan.action7

def action(plan,t):
 label=plan.waypoints[0].label if plan.waypoints else''
 if label.startswith('diabfull_'):
  decide(t);side=label.rsplit('_',1)[1];return LASTCMD[side],PHASE
 return old(plan,t)
ArmPlan.action7=action

def policy_stop(mod):
 cls=mod.YamScriptedPolicy
 if getattr(cls,'_diabfull_stop',False):return
 old=cls.act
 def act(self,observation):
  out=old(self,observation)
  if STOP:
   for a in out.actions:a.meta.update(request_stop=True,stop_reason='reference_hold'if FAILED else'reference_attempt_budget_complete',stop_detail=FAILED or'Boundedcomplete settingattempt finished. Scoreisrecomputedfromactualtrace; successmetadataisnotproof.')
  return out
 cls.act=act;cls._diabfull_stop=True
rt.ng.pk._Finder.targets['yam_arena.scripted']=policy_stop
if'yam_arena.scripted'in sys.modules:policy_stop(sys.modules['yam_arena.scripted'])
