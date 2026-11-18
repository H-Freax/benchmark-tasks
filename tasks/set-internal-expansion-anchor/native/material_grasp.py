"""Measured actual object-in-hand rigidity guard; never controls an objectpose."""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.constants import TCP_OFFSET

class MaterialGraspGuard:
 def __init__(self):self.anchors={};self.last={}
 def observe(self,row,side,name):
  r=row['robots'][side];bn=r['body_names'];poses=np.array(r['body_link_pose_w'],float)
  if len(bn)!=9 or poses.shape!=(9,7)or not np.isfinite(poses).all():raise ValueError('Invalidactualfullrobotpose')
  hand=poses[bn.index('gripper')];obj=np.array(row['rigid_bodies'][name]['pose_xyzw'],float)
  if obj.shape!=(7,)or not np.isfinite(obj).all():raise ValueError('Invalidactualheldbodypose')
  Rh=Rotation.from_quat(hand[3:]).as_matrix();Ro=Rotation.from_quat(obj[3:]).as_matrix();tcp=hand[:3]+Rh@np.array(TCP_OFFSET)
  # Material coordinates of the actualTCP and actualrelative orientation, nottargets.
  x=Ro.T@(tcp-obj[:3]);rel=Rh.T@Ro;key=(side,name);c=row['control']
  if key not in self.anchors:self.anchors[key]=(x.copy(),rel.copy(),c)
  x0,R0,c0=self.anchors[key]
  previous=self.last.get(key)
  if previous is not None and c<=previous:raise ValueError('Graspguardrequiresfreshactualrow')
  self.last[key]=c;shift=float(np.linalg.norm(x-x0));angle=float(Rotation.from_matrix(rel@R0.T).magnitude())
  out=dict(actual_control=c,side=side,object=name,anchor_actual_control=c0,TCP_in_object_m=x.tolist(),material_point_drift_m=shift,
           held_rotation_drift_rad=angle,limits_m_rad=[.003,float(np.deg2rad(5))],no_object_pose_or_force_write=True)
  if shift>.003:raise GraspFailure('Actualheldmaterialpointslipped',out)
  if angle>np.deg2rad(5):raise GraspFailure('Actualheldtoolrotatedinhand',out)
  return out
class GraspFailure(RuntimeError):
 def __init__(self,reason,measurement):self.measurement=measurement;super().__init__(reason)
