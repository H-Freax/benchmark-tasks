"""Ordinary joint-target servo; actual arrival gates precede neck insertion."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik,station
from yam_arena.constants import READY,ARM_JOINTS
from yam_arena.demo import grip_rotation
from cleaning import transform
from pad_contacts import two_pad

MAX_STEP_M=.0008
MAX_STEP_RAD=math.radians(.5)

def material_target(desired_brush,current_brush,current_hand):
 return desired_brush @ np.linalg.inv(current_brush) @ current_hand

def bounded_material_step(current,desired):
 out=current.copy();d=desired[:3,3]-current[:3,3];n=np.linalg.norm(d)
 out[:3,3]+=d*min(1.,MAX_STEP_M/max(n,1e-12))
 w=Rotation.from_matrix(desired[:3,:3]@current[:3,:3].T).as_rotvec();a=np.linalg.norm(w)
 out[:3,:3]=Rotation.from_rotvec(w*min(1.,MAX_STEP_RAD/max(a,1e-12))).as_matrix()@current[:3,:3]
 return out

def cylinder_vertices(radius,z0,z1):
 a=np.arange(32)*2*np.pi/32
 return np.array([[radius*np.cos(t),radius*np.sin(t),z]for z in [z0,z1]for t in a])

def neck_projection_clear(brush,bottle):
 relative=np.linalg.inv(bottle)@brush;v=cylinder_vertices(.02,0.,.04)
 p=v@relative[:3,:3].T+relative[:3,3]
 # The true32sector inner aperture has a smaller inradius than22mm.
 return bool(np.linalg.norm(p[:,:2],axis=1).max()<=.022*math.cos(math.pi/32)-.0002)

def neck_shape_clear(brush,bottle):
 """Clip every original brush cylinder edge to the actual25mm neck slab."""
 rel=np.linalg.inv(bottle)@brush;report={}
 edges=[(i,i+32)for i in range(32)]+[(layer*32+i,layer*32+(i+1)%32)for layer in [0,1]for i in range(32)]
 for name,r,z0,z1 in [('head',.02,0.,.04),('stem',.0025,.04,.20),('handle',.01,.20,.24)]:
  v=cylinder_vertices(r,z0,z1)@rel[:3,:3].T+rel[:3,3];points=[p for p in v if .114<=p[2]<=.139]
  for i,j in edges:
   a,b=v[i],v[j]
   for z in [.114,.139]:
    if(a[2]-z)*(b[2]-z)<0:points.append(a+(b-a)*((z-a[2])/(b[2]-a[2])))
  maximum=max((float(np.linalg.norm(p[:2]))for p in points),default=0.)
  report[name]=dict(vertices_in_neck=len(points),max_radius_m=maximum,clear=maximum<=.022*math.cos(math.pi/32)-.0002)
 return all(x['clear']for x in report.values()),report

class Controller:
 def __init__(self,rt,plan):
  self.rt=rt;self.rows=plan['rows'];self.last_index=-1;self.last_tick=None;self.virtual=0;self.wait=0;self.streak=0
  self.q={a:np.array(READY,float)for a in ['left','right']};self.sent={a:q.copy()for a,q in self.q.items()};self.grip={'left':1.,'right':1.}
  self.failure=None;self.failed_ticks=0;self.stop_requested=False;self.grasp=None;self.details={};self.arrival_wait=0
  self.R0=grip_rotation(0);self.B0=Rotation.from_euler('y',90,degrees=True).as_matrix()
  self.nominal_bottle=transform(next(p for p in rt.G['parts']if p['name']=='feeding_bottle')['pos']+[0,0,0,1]);self.nominal_grasp=np.array([0.,0.,.22])
 def hand(self,row,side):
  d=row['robots'][side];g=np.asarray(d['links'][d['body_names'].index('gripper')],float);T=transform(g);T[:3,3]+=T[:3,:3]@np.array([0,0,.134]);return T
 def actual_q(self,row,side):
  d=row['robots'][side];return np.array([d['q'][d['joint_names'].index(n)]for n in ARM_JOINTS],float)
 def advance(self,t):
  index=max(0,int(math.floor((t+1e-8)*20))-1)
  if index==self.last_index:return
  self.last_index=index;row=self.rt.LATEST
  if row is None:
   if index>0:self.failure='MissingActualObservation'
   return
  actual={a:self.actual_q(row,a)for a in self.q};self.details={}
  try:
   tick=row['actual_physics_tick']
   if self.last_tick is not None and tick<=self.last_tick:raise RuntimeError('DuplicateOrReversedActualPhysicsTick')
   self.last_tick=tick
   if self.failure:raise RuntimeError(self.failure)
   pads=row['pad_contacts'];holding=two_pad(pads,'bottle_brush','right')and two_pad(pads,'feeding_bottle','left');self.streak=self.streak+1 if holding else 0
   if self.virtual==79 and self.streak<5:
    self.wait+=1
    if self.wait>40:raise RuntimeError('ActualTwoPadBottleOrHandleGraspAbsent')
    for a in self.grip:self.grip[a]=0.
    self.log(row,index,'grasp_verify_wait');return
   target=self.rows[min(self.virtual,len(self.rows)-1)];phase=target['phase'];arrived=True
   for side in ['left','right']:
    desired_p=np.array(target['target'][side]['p']);desired_R=np.array(target['target'][side]['R']);self.grip[side]=target['finger'][side]
    servo=side=='right'and self.virtual>=80 and phase not in ['release_brush','both_hands_clear','released_stability']
    if servo:
     if not two_pad(pads,'bottle_brush','right'):raise RuntimeError('ActualHandlePadContactLost')
     current=transform(row['brush_pose']);hand=self.hand(row,'right');grasp=np.linalg.inv(hand)@current
     if self.grasp is None:self.grasp=grasp.copy()
     slide=float(np.linalg.norm(grasp[:3,3]-self.grasp[:3,3]));twist=float(Rotation.from_matrix(grasp[:3,:3]@self.grasp[:3,:3].T).magnitude())
     self.details['grasp_drift']=dict(m=slide,rad=twist)
     if slide>.004 or twist>math.radians(2):raise RuntimeError('ActualMaterialGraspDriftRequiresRegrasp')
     nominal=np.eye(4);nominal[:3,:3]=desired_R@self.R0.T@self.B0;nominal[:3,3]=desired_p-nominal[:3,:3]@self.nominal_grasp
     bottle=transform(row['bottle_pose'])
     if any(k in phase for k in ['neck','insert','bottom','shoulder','withdraw_brush']):nominal=bottle@np.linalg.inv(self.nominal_bottle)@nominal
     pe=float(np.linalg.norm(nominal[:3,3]-current[:3,3]));re=float(Rotation.from_matrix(nominal[:3,:3]@current[:3,:3].T).magnitude())
     arrived=pe<=.0015 and re<=math.radians(.5)
     clear,shape=neck_shape_clear(current,bottle);self.details.update(actual_brush_pose_error_m=pe,actual_brush_pose_error_rad=re,neck_shape=shape,desired_brush=nominal.tolist())
     if any(k in phase for k in ['insert','bottom','shoulder','withdraw_brush'])and not clear:raise RuntimeError('ActualBrushCylinderOutsideNeckCorridor')
     # Last above-neck pose cannot advance to insertion until the whole head
     # projects inside the actual aperture, with actual pose already reached.
     if phase=='above_neck'and self.virtual==199:
      projection=neck_projection_clear(current,bottle);self.details['whole_head_aperture_clear']=projection;arrived=arrived and projection
     next_material=bounded_material_step(current,nominal);wanted=material_target(next_material,current,hand);desired_p=wanted[:3,3];desired_R=wanted[:3,:3]
    q,pe_ik,re_ik=ik.solve(desired_p,ik.matrix_to_quat_xyzw(desired_R),actual[side],base_pos=station.BASES[side],iters=140,damping=.03,tol=.0002)
    if pe_ik>.003 or re_ik>.025:raise RuntimeError('ActualFrameIKOutsideOrdinaryTolerance:'+side+':'+repr((pe_ik,re_ik)))
    if servo:
     # Increment ordinary position targets from observed Cartesian error.
     # This closes static loaded tracking error without changing any drive.
     delta=np.clip(.45*(q-actual[side]),-.03,.03)
     command=self.sent[side]+delta
     if(command<ik.LOWER).any()or(command>ik.UPPER).any():raise RuntimeError('OrdinaryJointTargetAtStockLimit')
     self.q[side]=command
    else:self.q[side]=q
   if arrived:self.virtual+=1;self.arrival_wait=0
   else:
    self.arrival_wait+=1
    if self.arrival_wait>300:raise RuntimeError('ActualMaterialTargetArrivalTimeout')
   self.sent={a:q.copy()for a,q in self.q.items()};self.details['actual_arrived']=arrived;self.log(row,index,phase)
  except Exception as exc:
   if self.failure is None:self.failure=str(exc)
   self.failed_ticks+=1
   # The failed step holds the previous ordinary target, not a new pose.
   self.q={a:q.copy()for a,q in self.sent.items()}
   if self.failed_ticks>=10:self.stop_requested=True
   self.log(row,index,'failed_hold')
 def log(self,row,index,phase):
  import json
  self.rt.TRACE.mkdir(parents=True,exist_ok=True)
  with(self.rt.TRACE/'controller.jsonl').open('a')as f:f.write(json.dumps(dict(action=index,virtual=self.virtual,actual_physics_tick=row['actual_physics_tick'],phase=phase,failure=self.failure,q={a:q.tolist()for a,q in self.q.items()},grip=self.grip,grasp_streak=self.streak,grasp_wait=self.wait,arrival_wait=self.arrival_wait,details=self.details,score_unchanged=True),allow_nan=False)+'\n')
 def action(self,side,t):
  self.advance(t);return np.r_[self.q[side],self.grip[side]],'actual_material_arrival_gated_reference'
