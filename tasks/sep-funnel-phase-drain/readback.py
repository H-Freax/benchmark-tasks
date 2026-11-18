"""Read actual backend tensors without cached ProxyArray slicing."""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.constants import ARM_JOINTS,TCP_OFFSET


def array(value):
 value=getattr(value,'torch',value)
 if hasattr(value,'detach'):return value.detach().cpu().numpy().copy()
 if hasattr(value,'numpy'):return value.numpy().copy()
 return np.asarray(value).copy()


def robots(base):
 """Return gripper feature points, backend-named joint q, and native links.

 No Isaac cached `.data` access; body/joint names match root_view ordering.
 A failure raises to the caller's fail-closed observation handler.
 """
 grips={};joints={};links={}
 for side in ['left','right']:
  robot=base.scene[side+'_robot'];view=robot.root_view
  q=array(view.get_dof_positions());p=array(view.get_link_transforms())
  joint_names=list(robot.backend_joint_names);body_names=list(robot.backend_body_names)
  if q.shape!=(1,8)or p.shape!=(1,9,7)or len(joint_names)!=8 or len(body_names)!=9:raise ValueError('ActualRobotTensorShape:'+side)
  if len(set(joint_names))!=8 or len(set(body_names))!=9:raise ValueError('ActualRobotNamesNotUnique:'+side)
  if any(n not in joint_names for n in (*ARM_JOINTS,'joint7','joint8'))or 'gripper'not in body_names:raise ValueError('ActualRobotNameMissing:'+side)
  if not np.isfinite(q).all()or not np.isfinite(p).all()or np.any(abs(np.linalg.norm(p[:,:,3:],axis=-1)-1.)>1e-3):raise ValueError('NonfiniteOrInvalidActualRobot:'+side)
  gp=p[0,body_names.index('gripper')];R=Rotation.from_quat(gp[3:]).as_matrix()
  origin=np.zeros(3)
  if hasattr(base.scene,'env_origins'):origin=array(base.scene.env_origins)[0]
  position=gp[:3]-origin
  q7=float(q[0,joint_names.index('joint7')]);q8=float(q[0,joint_names.index('joint8')])
  half=(abs(q7)+abs(q8))*.5+.008
  points=[position+R@np.array(v)for v in ((0,0,.06),TCP_OFFSET,(half,0,.147),(-half,0,.147),(half,0,.12),(-half,0,.12))]
  grips[side]=(points,abs(q7)+abs(q8))
  joints[side]=dict(joint_names=joint_names,joint_positions=q[0].tolist())
  links[side]=dict(body_names=body_names,body_link_pose_w=p[0].tolist(),pose_convention='XYZ+XYZW',source='actual root_view backend tensor getters')
 return grips,joints,links
