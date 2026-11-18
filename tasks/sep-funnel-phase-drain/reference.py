"""Ordinary 14D stock YAM joint reference; actual feedback only, no pose writers."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik,station
from yam_arena.constants import READY,SHOULDER,WRIST,ARM_JOINTS
from scene_spec import STOPPER,CUP_START,CUP_TARGET,VALVE,STOPPER_TRAY,TABLE
from grasp_geometry import stopper_vertices,target_for_material_bbox
from valve_settle import PoseSettle

DOWN=np.diag([-1.,1.,-1.])
CUP_R=Rotation.from_euler('y',90,degrees=True).as_matrix()
SIDE=Rotation.from_euler('x',90,degrees=True).as_matrix()@Rotation.from_euler('z',180,degrees=True).as_matrix()
# Finger closing X->worldZ; approach Z->world+Y, palm stays in front of valve.
KEY_R=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])


class Controller:
 def __init__(self,rt):
  self.rt=rt;self.q={s:np.array(READY,float)for s in ['left','right']};self.grip={s:1. for s in self.q};self.stage=0;self.start=None;self.index=None;self.failure=None;self.done=False;self.anchor={};self.phase='settle';self.target_angle=0.;self.qpath=[];self.stopper_material_in_hand=None;self.valve_settle=PoseSettle();self.valve_settle_diagnostic={}
  self.sequence=[('settle',25),('stopper_hover',45),('stopper_down',35),('stopper_close',30),('stopper_lift',40),('stopper_clear',40),('stopper_tray_hover',45),('stopper_tray_down',35),('stopper_open',25),('left_retreat',35),('key_hover',45),('key_approach',35),('key_close',30),('key_open',50),('drain',2500),('key_close_valve',45),('key_closed_settle',60),('key_release',100),('key_withdraw',40),('right_retreat',40),('final_settle',40)]
 def solve(self,side,p,R):
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q[side],base_pos=station.BASES[side],iters=250,tol=1e-6,damping=.025)
  if pe>.003 or re>.03:
   best=(q,pe,re)
   seeds=[np.array([2.8,.05,1.6,1.5,-1.2,0.])]+[np.random.default_rng(k).uniform(ik.LOWER,ik.UPPER)for k in range(12)]
   for seed in seeds:
    qq,ee,rr=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,base_pos=station.BASES[side],iters=300,tol=1e-6,damping=.02)
    if ee+rr*.1<best[1]+best[2]*.1:best=(qq,ee,rr)
    if ee<.0001 and rr<.001:break
   q,pe,re=best
  if pe>.003 or re>.03:raise ValueError('IK:'+repr((self.phase,side,pe,re)))
  self.q[side]=q
 def actual(self,side):
  row=self.rt.LAST_ROW
  if row and row.get('known'):
   return np.array(row['robot_joint_positions'][side][:6])
  return self.q[side].copy()
 def pose(self,name):
  if self.rt.LAST is not None:
   p=self.rt.LAST[name][0];return p[:3].copy(),Rotation.from_quat(p[3:]).as_matrix()
  if name=='stopper':return STOPPER.copy(),np.eye(3)
  if name=='receiver':return CUP_START.copy(),np.eye(3)
  return VALVE.copy(),np.eye(3)
 def enter(self,index):
  self.phase,n=self.sequence[self.stage];self.start=index
  side='right'if self.phase.startswith(('key','right'))or self.phase=='drain'else'left'
  p,R=ik.tcp_pose(self.q[side],station.BASES[side]);self.anchor=dict(p=p,R=R,side=side,q=self.q[side].copy())
  if self.phase=='stopper_tray_hover':
   actual_tcp,actual_R=ik.tcp_pose(self.actual('left'),station.BASES['left']);sp,Rs=self.pose('stopper')
   self.stopper_material_in_hand=(stopper_vertices()@Rs.T+sp-actual_tcp)@actual_R
  if self.phase in ['stopper_hover','stopper_tray_hover','cup_hover','key_hover','left_retreat','cup_retreat','right_retreat']:
   target=self.target(self.phase);self.solve(side,*target);self.anchor['q_end']=self.q[side].copy();self.q[side]=self.anchor['q'].copy()
  if self.phase=='stopper_close':self.grip['left']=1.
  if self.phase=='key_open':
   self.anchor['center']=VALVE.copy();self.anchor['R']=KEY_R.copy();self.anchor['theta']=self.rt.LAST_ROW.get('valve_angle_rad',0.)if self.rt.LAST_ROW else 0.
  if self.phase=='key_close_valve':self.anchor['theta']=self.target_angle
 def target(self,phase):
  sp,Rs=self.pose('stopper');cp,Rc=self.pose('receiver')
  if phase in ['stopper_tray_hover','stopper_tray_down']and self.stopper_material_in_hand is not None:
   z=STOPPER_TRAY[2]+(.145 if phase=='stopper_tray_hover'else .078)
   return target_for_material_bbox(local_in_hand=self.stopper_material_in_hand,target_R=SIDE,target_xy=STOPPER_TRAY[:2],tcp_z=z),SIDE
  return {
   'stopper_hover':(STOPPER,SIDE),'stopper_down':(sp,SIDE),'stopper_lift':(STOPPER+[0,0,.045],SIDE),'stopper_clear':(STOPPER+[-.08,.09,.045],SIDE),
   'stopper_tray_hover':(STOPPER_TRAY+[0,0,.145],SIDE),'stopper_tray_down':(STOPPER_TRAY+[0,0,.068],SIDE),'left_retreat':ik.tcp_pose(np.array(READY,float),station.BASES['left']),
   'cup_hover':(CUP_START+[-.060,.055,.060],CUP_R),'cup_down':(cp+Rc@[0,.055,.060],CUP_R),'cup_lift':(CUP_START+[0,.055,.060],CUP_R),
   'cup_move':(CUP_TARGET+[0,.055,.060],CUP_R),'cup_lower':(CUP_TARGET+[0,.055,.060],CUP_R),'cup_retreat':(CUP_TARGET+[-.100,.055,.100],CUP_R),
   'key_withdraw':(VALVE+[0,-.060,0],KEY_R),'key_hover':(VALVE+[0,-.060,0],KEY_R),'key_approach':(VALVE+[0,-.014,0],KEY_R),'right_retreat':ik.tcp_pose(np.array(READY,float),station.BASES['right'])
  }.get(phase)
 def key(self,angle):
  # Contact-driven native revolute key, not a joint-angle/pose setter.
  R=Rotation.from_euler('y',angle).as_matrix();self.solve('right',VALVE+R@np.array([0,-.014,0]),R@KEY_R);self.target_angle=angle
 def action(self,t):
  index=int(math.floor((t+1e-8)*20))
  if index==self.index:return
  self.index=index
  if self.done:return
  if self.start is None:self.enter(index)
  phase,n=self.sequence[self.stage];elapsed=index-self.start
  if phase=='key_closed_settle':
   self.valve_settle_diagnostic=self.valve_settle.observe(self.rt.LAST_ROW)
   if elapsed>=n and not self.valve_settle_diagnostic.get('pose_settled'):
    if elapsed>=120:self.failure='ActualClosedPoseNotSettledWithinBoundedHold';self.done=True;return
    n=121
  if phase=='drain':
   # Read actual lower-phase residual; slowing near interface is normal visual/
   # privileged scripted reference feedback, not a simulated outcome latch.
   row=self.rt.LAST_ROW
   if row and row.get('known')and row['fluids']['water_ml']<=2.2:
    self.stage+=1;self.enter(index);phase,n=self.sequence[self.stage];elapsed=0
  if elapsed>=n:
   if phase=='drain':self.failure='DrainBudgetNoLowerPhaseRecovery';self.done=True;return
   self.stage+=1
   if self.stage>=len(self.sequence):self.done=True;return
   self.enter(index);phase,n=self.sequence[self.stage];elapsed=0
  self.phase=phase;u=min(1.,(elapsed+1)/n);u=ik.smoothstep(u)
  previous={s:q.copy()for s,q in self.q.items()}
  try:
   if phase in ['key_withdraw','right_retreat','final_settle']:
    row=self.rt.LAST_ROW
    if not row or not row.get('known'):raise ValueError('WithdrawalContactObservationUnknown')
    if row and row.get('known'):
     contact=row['release_contact_observation']
     if not contact.get('known')or np.asarray(contact['counts']).shape!=(3,18):raise ValueError('WithdrawalContactObservationUnknown')
     if np.asarray(contact['counts'])[1].sum()>0:raise ValueError('UnexpectedActualKeyRecontactDuringWithdrawal')
   target=self.target(phase)
   if target:
    side=self.anchor['side'];p,R=target
    if 'q_end'in self.anchor:self.q[side]=self.anchor['q']*(1-u)+self.anchor['q_end']*u
    else:self.solve(side,self.anchor['p']*(1-u)+p*u,ik.slerp_matrix(self.anchor['R'],R,u))
   if phase in ['stopper_close','cup_close']:self.grip['left']=1-u
   elif phase in ['stopper_open','cup_open']:self.grip['left']=u
   elif phase=='key_close':self.grip['right']=1-u
   elif phase=='key_open':self.key(math.radians(50)*u)
   elif phase=='drain':
    row=self.rt.LAST_ROW;remaining=row['fluids']['water_ml']if row and row.get('known')else 100.
    desired=math.radians(50 if remaining>12 else 18 if remaining>5 else 8)
    self.key(self.target_angle+np.clip(desired-self.target_angle,-.02,.02))
   elif phase=='key_close_valve':self.key(self.anchor['theta']*(1-u))
   elif phase=='key_closed_settle':self.key(0.)
   elif phase=='key_release':self.grip['right']=u
   for side in self.q:
    delta=np.array([SHOULDER['velocity']]*3+[WRIST['velocity']]*3)/20
    if np.any(abs(self.q[side]-previous[side])>delta+1e-7):raise ValueError('StockVelocityLimit:'+side)
  except Exception as exc:
   self.failure=type(exc).__name__+':'+str(exc);self.done=True;self.q={s:self.actual(s)for s in self.q}
  self.qpath.append(dict(index=index,phase=phase,q_left=self.q['left'].tolist(),q_right=self.q['right'].tolist(),grip=dict(self.grip),target_angle=self.target_angle,failure=self.failure))
