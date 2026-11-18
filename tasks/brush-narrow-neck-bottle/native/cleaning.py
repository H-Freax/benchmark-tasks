"""Visual surrogate removal from actual head contact and measured relative slip."""
import numpy as np
from scipy.spatial.transform import Rotation
def transform(pose):
 a=np.asarray(pose,float)
 if a.shape!=(7,)or not np.isfinite(a).all()or abs(np.linalg.norm(a[3:])-1)>.01:raise ValueError('invalid actual pose')
 T=np.eye(4);T[:3,:3]=Rotation.from_quat(a[3:]).as_matrix();T[:3,3]=a[:3];return T
def point(T,p):return T[:3,:3]@np.asarray(p)+T[:3,3]
class Film:
 def __init__(self,G):
  self.G=G;self.dose=np.zeros(len(G['patches']));self.last_serial=-1;self.previous=None
 def update(self,row):
  serial=row.get('serial')
  if not isinstance(serial,int)or serial<=self.last_serial:return dict(advanced=False,removed=[],dose_m=self.dose.tolist())
  old=self.last_serial;self.last_serial=serial
  Tb=transform(row['bottle_pose']);Th=transform(row['brush_pose']);relative=np.linalg.inv(Tb)@Th
  # An actual contact must belong to the declared head envelope, not the shaft.
  good=[]
  for c in row.get('native_contacts',[]):
   force=float(c['normal_force_N']);p=np.asarray(c['world_point_m'],float)
   if not np.isfinite(force)or p.shape!=(3,)or not np.isfinite(p).all():raise ValueError('invalid contact row')
   local=point(np.linalg.inv(Th),p)
   if abs(force)>=self.G['cleaning']['contact_force_gate_N'] and -.0005<=local[2]<=.0405 and np.linalg.norm(local[:2])<=.023:good.append(local)
  previous=self.previous;self.previous=relative.copy()
  if previous is None or old+1!=serial or not good:
   return dict(advanced=True,removed=[],dose_m=self.dose.tolist(),head_contact_count=len(good))
  removed=[]
  for i,patch in enumerate(self.G['patches']):
   p=np.asarray(patch['center_m']);q=point(np.linalg.inv(relative),p)
   # The marked glass point must actually intersect the compliant head region.
   # A scalar pair contact does not clean distant points or a surrounding image.
   if np.linalg.norm(q[:2])>.0205 or not -.0005<=q[2]<=.0405:continue
   displacement=point(relative,q)-point(previous,q)
   normal=np.asarray(patch['normal']);tangent=displacement-normal*np.dot(displacement,normal)
   travel=float(np.linalg.norm(tangent))
   if not np.isfinite(travel):raise ValueError('nonfinite relative slip')
   if travel>.020:continue # Missing/high-speed interval is unknown, not clean.
   before=self.dose[i];self.dose[i]+=travel
   if before<patch['required_tangential_slip_m']<=self.dose[i]:removed.append(i)
  return dict(advanced=True,removed=removed,dose_m=self.dose.tolist(),head_contact_count=len(good))
 def coverage(self):
  area=np.array([p['area_m2']for p in self.G['patches']]);done=self.dose>=np.array([p['required_tangential_slip_m']for p in self.G['patches']])
  answer={'all':float(area@done/area.sum())}
  for kind in ['bottom','shoulder']:
   mask=np.array([p['kind']==kind for p in self.G['patches']]);answer[kind]=float(area[mask]@done[mask]/area[mask].sum())
  return answer
