"""Independent current-state criteria, with unique finite-observation stability."""
import numpy as np
from scipy.spatial.transform import Rotation
def vertices(part):
 out=[]
 for s in part['geoms']:
  if s['type']=='convex':v=np.array(s['vertices'])
  elif s['type']=='box':
   h=np.array(s['size']);v=np.array([[x,y,z]for x in[-h[0],h[0]]for y in[-h[1],h[1]]for z in[-h[2],h[2]]])
  elif s['type']=='cylinder':
   r,h=s['size'][:2];v=np.array([[r*np.cos(a),r*np.sin(a),z]for z in[-h,h]for a in np.arange(32)*np.pi/16])
  else:raise ValueError(s['type'])
  out.append(v@Rotation.from_quat(s['quat_xyzw']).as_matrix().T+s['pos'])
 return np.vstack(out)
def evaluate(G,row):
 names=('taper_shank_tool','steel_taper_drift')
 ps={p['name']:p for p in G['parts']};current={}
 if not isinstance(row.get('bodies'),dict)or any(n not in row['bodies']for n in names):return dict(score=0.,complete=False,valid=False)
 for n in names:
  a=row['bodies'][n];pose=np.asarray(a['pose'],float);vel=np.asarray(a['velocity'],float)
  if pose.shape!=(7,)or vel.shape!=(6,)or not np.isfinite(np.r_[pose,vel]).all():return dict(score=0.,complete=False,valid=False)
  current[n]=vertices(ps[n])@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
 tool=current[names[0]]
 # Mouth plane is the observed fixed sleeve's y=0 datum. Every point of the
 # taper/tang must clear that plane; after release a tool anywhere is partial.
 origin=np.array(G['spindle_origin_m'])
 extracted=bool(tool[:,1].min()>origin[1]+.003)
 # Final tray can have another orientation, so end-state extraction means no
 # original tool vertex overlaps the sleeve bore region.
 sleeve_clear=bool(np.any(tool.max(0)<origin-np.array([.026,.091,.026]))or np.any(tool.min(0)>origin+np.array([.026,.001,.026])))
 tr=G['tray'];center=np.array(tr['center_m']);half=np.array(tr['inner_halfsize_m'])
 placed=[]
 for n in names:
  v=current[n];a=row['bodies'][n]
  placed.append(bool(np.all(v[:,:2]>=center-half-.001)and np.all(v[:,:2]<=center+half+.001)
   and v[:,2].min()>=tr['floor_z_m']-.001 and v[:,2].max()<=tr['wall_top_m']+.003
   and np.linalg.norm(a['velocity'][:3])<.02 and np.linalg.norm(a['velocity'][3:])<.1))
 handpoints=np.asarray(row['hand_points'],float)
 if handpoints.ndim!=2 or handpoints.shape[1]!=3 or not np.isfinite(handpoints).all():return dict(score=0.,complete=False,valid=False)
 # Clearance from complete part AABBs is conservative for the ordinary hand
 # sample points. Independent CPU path uses all 630 collision hulls.
 cleared=all(np.linalg.norm(np.maximum(np.maximum(v.min(0)-handpoints,handpoints-v.max(0)),0),axis=1).min()>=.03 for v in current.values())
 intact=all(np.isfinite(v).all()and v[:,2].min()>=G['table_top_m']-.003 for v in current.values())
 complete=bool(intact and sleeve_clear and all(placed)and cleared)
 return dict(score=1. if complete else .25 if intact and (extracted or sleeve_clear)else 0.,complete=complete,valid=True,
  tool_extracted=extracted,sleeve_clear=sleeve_clear,placed=placed,hands_clear=cleared)
class Stable:
 def __init__(self):self.last=-1;self.last_time=None;self.start=None;self.answer={'score':0.,'complete':False,'valid':False}
 def update(self,G,row):
  serial=row.get('serial')
  if not isinstance(serial,int)or serial<self.last:return {'score':0.,'complete':False,'valid':False}
  if serial==self.last:return self.answer
  previous=self.last;self.last=serial;a=evaluate(G,row);t=row.get('time_s')
  if not isinstance(t,(int,float))or not np.isfinite(t):return {'score':0.,'complete':False,'valid':False}
  if self.last_time is not None and (serial!=previous+1 or abs(t-self.last_time-.05)>1e-6):self.start=None
  self.last_time=t
  if a['complete']:
   if self.start is None:self.start=t
   held=t-self.start
   if held<2.:a=dict(a,score=.25,complete=False,stable_s=held)
  else:self.start=None
  self.answer=a;return a
