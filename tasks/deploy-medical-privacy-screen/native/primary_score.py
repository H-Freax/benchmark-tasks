"""Observable coverage/support/release, fresh native stability; no phase use."""
from collections import deque
import math,numpy as np
from scipy.spatial.transform import Rotation
from coverage_geometry import coverage
from convex_distance import distance_bounds
NAMES=['centre_panel','left_panel','right_panel']
class PrimaryScore:
 def __init__(self):self.reset()
 def reset(self):self.value=0.;self.history=deque();self.last_step=None;self.last_time=None;self.clock_offset=None
 def invalidate(self,reason):
  self.reset();return dict(valid=False,unknown=reason,score=0.)
 def observe(self,row,G):
  try:return self._observe(row,G)
  except Exception as e:return self.invalidate(type(e).__name__+':'+str(e))
 def _observe(self,row,G):
  self.value=0.;rawstep=float(row['physics_step']);t=float(row['time_s']);hz=float(G['physics_hz']);control_hz=float(G['control_hz'])
  if not all(math.isfinite(v)for v in[rawstep,t,hz,control_hz])or rawstep<0 or rawstep!=int(rawstep)or min(hz,control_hz)<=0:return self.invalidate('invalid actual physics clock')
  step=int(rawstep);offset=t-step/hz
  if self.last_step is None:self.clock_offset=offset
  else:
   dt=t-self.last_time;dst=step-self.last_step
   if dst<=0 or dt<=0:return self.invalidate('duplicate/nonincreasing actual observation')
   if abs(dt-dst/hz)>1e-7 or abs(offset-self.clock_offset)>1e-7:return self.invalidate('actual time/physics tick interval mismatch')
   if abs(dt-1/control_hz)>1e-7:return self.invalidate('missing fresh control observations inside hold')
  self.last_step=step;self.last_time=t;poses=[];world=[];feet=[];upright=True;fast=False
  for n in NAMES:
   b=row['rigid_bodies'][n];pose=np.asarray(b['pose_xyzw'],float);velocity=np.asarray(b['native_velocity'],float)
   if pose.shape!=(7,)or velocity.shape!=(6,)or not np.isfinite(np.r_[pose,velocity]).all():raise ValueError('Missing/nonfinite actual body data')
   poses.append(pose.tolist());R=Rotation.from_quat(pose[3:]).as_matrix();p=pose[:3]
   upright&=R[2,2]>=math.cos(math.radians(5));fast|=np.linalg.norm(velocity[:3])>.02 or np.linalg.norm(velocity[3:])>.03
   part=next(x for x in G['parts']if x['name']==n)
   for sh in part['geoms']:
    if not sh['collision_enabled']:continue
    v=np.asarray(sh['vertices'])@R.T+p;world.append(v)
    if sh['name']=='normal_floor_protector':feet.append(float(v[:,2].min()))
  floor=float(G['floor_surface_z_m']);gaps=[z-floor for z in feet];supported=len(feet)==6 and min(gaps)>=-.005 and max(gaps)<=.005
  integrity=True
  for j in G['joints']:
   a=row['rigid_bodies'][j['parent']]['pose_xyzw'];b=row['rigid_bodies'][j['child']]['pose_xyzw'];ap=np.array(a[:3])+Rotation.from_quat(a[3:]).as_matrix()@j['parent_local_pos'];bp=np.array(b[:3])+Rotation.from_quat(b[3:]).as_matrix()@j['child_local_pos']
   integrity&=np.linalg.norm(ap-bp)<=.005
  c=coverage(poses,G['doorway_sight_points_m'],G['target_samples_m'],G)
  from yam_arena.analysis import world_hulls
  target_low=np.array([v.min(0)for v in world]);target_high=np.array([v.max(0)for v in world]);clear=True;count=0;min_gap=float('inf');min_upper=None;unknown_pairs=[];violations=[];distance_calls=0
  for side in['left','right']:
   obs=row['robots'][side];link={}
   for name,pose in zip(obs['body_names'],obs['body_link_pose_w'],strict=True):
    v=np.asarray(pose,float)
    if v.shape!=(7,)or not np.isfinite(v).all():raise ValueError('Nonfinite robot link pose')
    M=np.eye(4);M[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();M[:3,3]=v[:3];link[name]=M
   hs=world_hulls(link);count+=len(hs)
   for ln,h in hs:
    lo=h.vertices.min(0);hi=h.vertices.max(0);gaps=np.linalg.norm(np.maximum(np.maximum(target_low-hi,lo-target_high),0.),axis=1)
    # Broad-phase separation can prove PASS only. Insufficient lower bounds
    # are refined using finite-solid Euclidean distance, never called failures.
    guard=max(1e-9,128*np.finfo(float).eps*max(1.,float(np.abs(h.vertices).max()),float(np.abs(target_low).max()),float(np.abs(target_high).max())))
    min_gap=min(min_gap,max(0.,float(gaps.min())-guard))
    for wi in np.flatnonzero(gaps<.05+guard):
     # One certified violation proves no credit; still validate all robot data.
     if violations:break
     result=distance_bounds(h.vertices,world[wi],threshold=.05);distance_calls+=1
     item=dict(side=side,link=ln,screen_solid=int(wi),**result)
     if result['status']=='violation':violations.append(item)
     elif result['status']=='indeterminate':unknown_pairs.append(item)
     elif result['status']!='clear':raise ValueError('Unknown convex-distance decision')
     u=result.get('upper_m')
     if u is not None:min_upper=float(u)if min_upper is None else min(min_upper,float(u))
  clear=not violations and not unknown_pairs
  if count!=630:raise ValueError('Incomplete actual stock robot hull coverage: '+str(count))
  if unknown_pairs and not violations:
   out=self.invalidate('Finite convex clearance numerically indeterminate')
   out.update(score_status='indeterminate',physical_clearance_violation=False,clearance_bounds=unknown_pairs,robot_hulls=count)
   return out
  # P7 new shared rule: sampled first-pose envelope, not native velocity.
  eligible=bool(upright and supported and clear and integrity and c>=.70)
  if not eligible:self.history.clear()
  else:self.history.append((t,np.asarray(poses),float(c),bool(fast)))
  # Freshness above fixes the cadence; retain the canonical final 21 rows.
  # A floating timestamp cutoff can retain 22 rows and change the anchor pose.
  while len(self.history)>round(control_hz)+1:self.history.popleft()
  stable=False
  if self.history and self.history[0][0]<=t-1+1e-9 and len(self.history)>=round(control_hz)+1:
   first=self.history[0][1];dp=max(float(np.linalg.norm(x[:,:3]-first[:,:3],axis=1).max())for tt,x,cc,ff in self.history);dr=max(np.linalg.norm((Rotation.from_quat(b[3:])*Rotation.from_quat(a[3:]).inv()).as_rotvec())for tt,x,cc,ff in self.history for a,b in zip(first,x));stable=dp<=.005 and dr<=math.radians(1)
  window_coverage=min((cc for tt,x,cc,ff in self.history),default=0.)
  if eligible and stable:self.value=1. if window_coverage>=.99 else .5
  return dict(valid=True,coverage_fraction=float(c),minimum_window_coverage_fraction=window_coverage,upright=bool(upright),supported=bool(supported),hinge_integrity=bool(integrity),foot_min_z_m=feet,hands_clear=bool(clear),robot_hulls=count,minimum_clearance_lower_bound_m=min_gap,minimum_refined_clearance_upper_bound_m=min_upper,clearance_metric='finite-convex Euclidean distance; lower bounds prove pass only',clearance_refinement_calls=distance_calls,clearance_violations=violations,clearance_violations_exhaustive=not bool(violations),clearance_diagnostic_scope='first certified violation witness'if violations else'all necessary pair queries',clearance_status='violation'if violations else'clear',native_speed_exceeds_old_P6_threshold=bool(fast),native_speed_is_P7_score_gate=False,score_rule_version='P7-observable-pose-envelope',stable_1s=bool(stable),fresh_hold_samples=len(self.history),clock_offset_s=self.clock_offset,score=self.value,angle_rule='No135degreehardgate; alternative stable shielding configurations allowed')
