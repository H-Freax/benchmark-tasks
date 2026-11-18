"""Released-safe terminal, independent of reference phases and cleaning counters."""
import numpy as np
from scipy.spatial.transform import Rotation
from vertex_helper import vertices
def released(G,bodies,hand):
 points=np.asarray(hand,float)
 if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():return False
 parts={p['name']:p for p in G['parts']}
 for name in ['feeding_bottle','bottle_brush']:
  row=bodies.get(name,{})
  pose=np.asarray(row.get('pose'),float);vel=np.asarray(row.get('velocity'),float)
  if pose.shape!=(7,)or vel.shape!=(6,)or not np.isfinite(np.r_[pose,vel]).all():return False
  v=vertices(parts[name])@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
  if v[:,2].min()<G['table_top_m']-.002 or np.linalg.norm(vel[:3])>.02 or np.linalg.norm(vel[3:])>.1:return False
  d=np.linalg.norm(np.maximum(np.maximum(v.min(0)-points,points-v.max(0)),0),axis=1)
  if d.min()<.03:return False
 b=bodies['feeding_bottle']['pose'];h=bodies['bottle_brush']['pose']
 # The brush head must be out of the bottle. The brush is not required to use
 # a designated stand; its actual head must be outside the vessel envelope.
 R=Rotation.from_quat(b[3:]).as_matrix();rel=R.T@(np.array(h[:3])-b[:3])
 head=vertices(dict(geoms=[parts['bottle_brush']['geoms'][0]]))@Rotation.from_quat(h[3:]).as_matrix().T+h[:3]
 local=(head-np.array(b[:3]))@R
 return bool(local[:,2].min()>G['bottle']['top_m']+.005 or np.linalg.norm(rel[:2])>.070)
