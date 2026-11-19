"""Pure observable sight-line geometry; no simulator or policy state."""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from functools import lru_cache
def panel_corners(pose,width=.56,z0=.080,z1=1.622):
 a=np.asarray(pose,float)
 if a.shape!=(7,)or not np.isfinite(a).all():raise ValueError('finite physical pose required')
 R=Rotation.from_quat(a[3:]).as_matrix();return a[:3],R,width,z0,z1
def blocked(eye,target,panels):
 eye=np.asarray(eye);v=np.asarray(target)-eye
 for p,R,w,z0,z1 in panels:
  n=R[:,1];den=np.dot(n,v)
  if abs(den)<1e-10:continue
  t=np.dot(n,p-eye)/den
  if 0<t<1:
   q=R.T@(eye+t*v-p)
   if 0<=q[0]<=w and z0<=q[2]<=z1:return True
 return False
@lru_cache(maxsize=128)
def planes(vertices):return ConvexHull(np.asarray(vertices)).equations
def solid_coverage(poses,eyes,targets,geometry):
 """Exact finite opaque convex solids. Air/hinge slots are never filled."""
 starts=np.array([e for e in eyes for t in targets],float);ends=np.array([t for e in eyes for t in targets],float);hit=np.zeros(len(starts),bool)
 if not len(hit):raise ValueError('no sight lines')
 for name,pose in zip(['centre_panel','left_panel','right_panel'],poses,strict=True):
  a=np.asarray(pose,float)
  if a.shape!=(7,)or not np.isfinite(a).all():raise ValueError('finite physical pose required')
  R=Rotation.from_quat(a[3:]).as_matrix();s=(starts-a[:3])@R;v=(ends-starts)@R
  part=next(p for p in geometry['parts']if p['name']==name)
  for shape in part['geoms']:
   if not shape['collision_enabled']:continue
   E=planes(tuple(tuple(float(x)for x in row)for row in shape['vertices']));A=E[:,:3];b=E[:,3];d=v@A.T;f=s@A.T+b
   parallel=np.abs(d)<1e-12;valid=~np.any(parallel&(f>0),axis=1)
   q=np.divide(-f,d,out=np.zeros_like(f),where=~parallel)
   low=np.maximum(0.,np.max(np.where(d< -1e-12,q,-np.inf),axis=1));high=np.minimum(1.,np.min(np.where(d>1e-12,q,np.inf),axis=1))
   hit|=valid&(low<=high)&(high>0)&(low<1)
 return float(hit.mean())
def coverage(poses,eyes,targets,geometry=None):
 if geometry is not None:return solid_coverage(poses,eyes,targets,geometry)
 panels=[panel_corners(x)for x in poses];n=len(eyes)*len(targets)
 if not n:raise ValueError('no sight lines')
 return sum(blocked(e,t,panels)for e in eyes for t in targets)/n
