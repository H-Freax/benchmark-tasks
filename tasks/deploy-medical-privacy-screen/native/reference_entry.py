"""Ordinary public robot commands with measured screen-frame grasp planning.

Uses MIT donor2036 Arm implementation; no task-body pose writes. Reference
may read actual screen pose; this is separate from the model entry and score.
"""
from pathlib import Path
import json,sys,numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
import task as rt
from checked_arm import Arm
from yam_arena import ik
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.constants import TCP_OFFSET
from yam_arena.constants import ARM_JOINTS,SHOULDER,WRIST,GRIPPER
ARMS={s:Arm(s,rt.G['robot_bases_m'][s])for s in['left','right']};ITER={};LABEL={};LAST={};HELD={}
def demo():return{s:[Waypoint(60.,(.3,0,.9),FINGER_OPEN,'screen_reference_'+s)]for s in['left','right']}
def actual(name):
 if rt.LAST_ACTUAL is None:raise RuntimeError('No fresh measured screen state')
 a=np.asarray(rt.LAST_ACTUAL['rigid_bodies'][name]['pose_xyzw'],float)
 if a.shape!=(7,)or not np.isfinite(a).all():raise RuntimeError('Invalid actual screen pose')
 return a[:3],Rotation.from_quat(a[3:]).as_matrix()
def actual_hand(side):
 a=rt.LAST_ACTUAL['robots'][side];i=a['body_names'].index('gripper');p=np.asarray(a['body_link_pose_w'][i],float)
 if p.shape!=(7,)or not np.isfinite(p).all():raise RuntimeError('Missing actual native gripper frame')
 R=Rotation.from_quat(p[3:]).as_matrix();return p[:3]+R@np.asarray(TCP_OFFSET),R
def sync_measured_arm(side):
 obs=rt.LAST_ACTUAL['robots'][side];values=dict(zip(obs['joint_names'],obs['joint_positions'],strict=True));q=np.array([values[n]for n in ARM_JOINTS],float)
 if q.shape!=(6,)or not np.isfinite(q).all():raise RuntimeError('Missing finite actual robot joints')
 arm=ARMS[side];arm.q=q;arm.pos,arm.rot=actual_hand(side);arm.cmd=np.r_[q,arm.grip]
def double_pad(name,side):
 data=rt.LAST_ACTUAL['contacts']['pairs'];pads=set()
 for key,obs in data.items():
  if not key.startswith(name+'__'+side+'__'):continue
  for pad in['tip_left','tip_right']:
   if pad in key and any(np.isfinite(float(x['normal_force_N']))and abs(float(x['normal_force_N']))>.01 for x in obs.get('entries',[])):pads.add(pad)
 return pads=={'tip_left','tip_right'}
def pregrasp_jaw_gap(name,side,griplocal):
 p,R=actual(name);hp,hR=actual_hand(side);obs=rt.LAST_ACTUAL['robots'][side];q=np.asarray(obs['joint_positions'],float)
 if q.shape!=(8,)or not np.isfinite(q).all():raise RuntimeError('Missing actual jaw joints')
 section=np.array([[x,y,griplocal[2]]for x in[.53,.56]for y in[-.01,.01]])@R.T+p;local=(section-hp)@hR;lo=float(q[6]);hi=float(-q[7]);inside=bool(local[:,0].min()>=lo-.0005 and local[:,0].max()<=hi+.0005)
 row=dict(control=rt.LAST_ACTUAL['control'],physics_step=rt.LAST_ACTUAL['physics_step'],side=side,target=name,actual_jaw_interval_m=[lo,hi],actual_frame_closing_interval_m=[float(local[:,0].min()),float(local[:,0].max())],actual_frame_depth_interval_m=[float(local[:,2].min()),float(local[:,2].max())],inside_open_jaws=inside)
 rt.TRACE.mkdir(parents=True,exist_ok=True)
 with(rt.TRACE/'pregrasp-actual-jaw-gap.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 if not inside:raise RuntimeError(('actual frame is not inside actual open jaws',row))
def guarded_move(side,label,p,R,seconds,grip):
 arm=ARMS[side]
 motion=arm.joint_approach(p,R,seconds,grip)if label=='outside_screen_end'else arm.return_ready(seconds)if label=='hands_clear'else arm.move(p,R,seconds,grip)
 for _ in motion:
  ap,aR=ik.tcp_pose(arm.q,arm.base);pe=np.linalg.norm(ap-arm.pos);re=np.linalg.norm(Rotation.from_matrix(aR@arm.rot.T).as_rotvec())
  if pe>(.005 if label=='approach_frame'else .002) or re>np.radians(1.):raise RuntimeError(('reference unreachable',side,label,pe,re))
  LABEL[side]=label;yield
def path(side):
 # Wait for the actual on-step observer, then compute ordinary-frame targets.
 for _ in range(10):yield
 name='right_panel'if side=='left'else'left_panel';p,R=actual(name)
 a=np.radians(-30.);gr=np.array([[0.,np.sin(a),np.cos(a)],[-1.,0.,0.],[0.,-np.cos(a),np.sin(a)]])
 griplocal=np.array([.545,0,.9-rt.G['floor_surface_z_m']]);tcp=p+R@griplocal-.012*gr[:,2]
 base=np.asarray(rt.G['robot_bases_m'][side]);sg=1 if side=='left'else-1
 pre=tcp-.03*gr[:,2]
 outside=np.array([pre[0]+.045,base[1],.95])
 yield from guarded_move(side,'outside_screen_end',outside,gr,3.,1.)
 stage=np.array([pre[0]+.045,pre[1]+sg*.10,pre[2]+.025])
 yield from guarded_move(side,'stage_outside_frame',stage,gr,3.,1.)
 corner=np.array([pre[0],pre[1]+sg*.055,pre[2]+.015])
 yield from guarded_move(side,'round_frame_corner',corner,gr,3.,1.)
 p,R=actual(name);gr=R@Rotation.from_euler('z',180,degrees=True).as_matrix().T@gr;tcp=p+R@griplocal-.012*gr[:,2]
 yield from guarded_move(side,'approach_frame',tcp-.03*gr[:,2],gr,3.,1.)
 p,R=actual(name);tcp=p+R@griplocal-.012*gr[:,2]
 yield from guarded_move(side,'insert_frame',tcp,gr,2.,1.)
 p,R=actual(name);tcp=p+R@griplocal-.012*gr[:,2]
 hp,hR=actual_hand(side);local=R.T@(hp-p)
 if abs(local[1])>.006 or abs(local[0]-.556)>.010:raise RuntimeError(('pregraspactualframeoutofjawgap',side,local.tolist()))
 pregrasp_jaw_gap(name,side,griplocal)
 yield from guarded_move(side,'grasp_frame',tcp,gr,1.5,.18)
 good=0;last=-1;relations=[]
 for _ in range(40):
  step=rt.LAST_ACTUAL['physics_step']
  if step>last:
   last=step;good=good+1 if double_pad(name,side)else 0
   pp,rr=actual(name);hp,hR=actual_hand(side);relations.append((hR.T@(pp-hp),hR.T@rr))
  yield
 if good<3:raise RuntimeError(('native double-pad contact unconfirmed',side,good))
 if len(relations)<20:raise RuntimeError('Missing fresh actual grasp observations')
 a,b=relations[-20],relations[-1];dp=np.linalg.norm(a[0]-b[0]);dr=np.linalg.norm(Rotation.from_matrix(b[1]@a[1].T).as_rotvec())
 if dp>.005 or dr>np.radians(3):raise RuntimeError(('native actual grasp drifting',side,dp,dr))
 sync_measured_arm(side);p0,R0=actual(name);hp,hR=actual_hand(side);lp=hR.T@(p0-hp);lR=hR.T@R0;HELD[side]=(lp,lR)
 # Desired unfolded panel follows its actual native hinge pivot. Centre is
 # measured after grasp, so pre-grasp global drift cannot be silently ignored.
 cp,cR=actual('centre_panel');j=next(j for j in rt.G['joints']if j['child']==name);pivot=cp+cR@j['parent_local_pos']
 desired=np.radians(rt.G['target_wing_angles_world_deg'][name]);goalR=Rotation.from_euler('z',desired).as_matrix();duration=9.;n=round(duration*20)
 for k in range(1,n+1):
  Rnext=ik.slerp_matrix(R0,goalR,ik.smoothstep(k/n));pnext=pivot-Rnext@j['child_local_pos'];hr=Rnext@lR.T;hp=pnext-hr@lp
  yield from guarded_move(side,'deploy_screen',hp,hr,.05,.18)
 for _ in range(20):yield
 ap,aR=actual(name);expected=pivot-goalR@j['child_local_pos']
 if np.linalg.norm(ap-expected)>.025:raise RuntimeError(('screen lost from actual grasp',side,np.linalg.norm(ap-expected)))
 # A small whole-screen placement correction is an ordinary physical slide.
 # Both hands remain on their actual frame grips; all panels must follow by
 # native hinges/floor contact. It is not a pose adjustment or score command.
 sync_measured_arm(side);hp,hR=actual_hand(side)
 yield from guarded_move(side,'place_shielding_line',hp+[.05,0,0],hR,3.,.18)
 for _ in range(20):yield
 sync_measured_arm(side);hp,hR=actual_hand(side)
 yield from guarded_move(side,'release_screen',hp,hR,1.,1.)
 sync_measured_arm(side);hp,hR=actual_hand(side);sg=1 if side=='left'else-1
 yield from guarded_move(side,'retreat_normal',hp+[-.09,-.09*sg,0],hR,2.,1.)
 sync_measured_arm(side)
 yield from guarded_move(side,'hands_clear',hp,hR,3.,1.)
 for _ in range(50):yield
 while True:LABEL[side]='done';yield
old=ArmPlan.action7
def action(plan,t):
 label=plan.waypoints[0].label if plan.waypoints else''
 if not label.startswith('screen_reference_'):return old(plan,t)
 side=label.rsplit('_',1)[-1];step=round(t*20)
 if LAST.get(side)!=step:
  if side not in ITER:ITER[side]=path(side)
  next(ITER[side]);LAST[side]=step
 return ARMS[side].cmd.copy(),LABEL.get(side,'wait_actual')
ArmPlan.action7=action
rt.PHASE_PROVIDER=lambda:' / '.join(s+':'+LABEL.get(s,'wait_actual')for s in['left','right'])
