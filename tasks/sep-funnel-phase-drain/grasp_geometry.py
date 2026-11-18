"""CPU geometry for ordinary placement of an actually grasped stopper."""
import numpy as np


def stopper_vertices():
 return np.array([[x,y,z]for x in [-.0125,.0125]for y in [-.0125,.0125]for z in [-.008,.008]]+[[.01098*np.cos(a),.01098*np.sin(a),z]for a in np.arange(32)*np.pi/16 for z in [-.036,-.008]])


def target_for_material_bbox(*,local_in_hand,target_R,target_xy,tcp_z):
 v=np.asarray(local_in_hand,float);R=np.asarray(target_R,float);xy=np.asarray(target_xy,float)
 if v.ndim!=2 or v.shape[1]!=3 or R.shape!=(3,3)or xy.shape!=(2,)or not np.isfinite(v).all()or not np.isfinite(R).all()or not np.isfinite(xy).all()or not np.isfinite(tcp_z):raise ValueError('InvalidActualGraspGeometry')
 relative=v@R.T;center=(relative.min(0)+relative.max(0))*.5
 return np.array([*(xy-center[:2]),tcp_z])
