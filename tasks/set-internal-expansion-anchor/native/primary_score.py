"""Measured outcome baseline, no referencephase/commands as score input."""
import numpy as np
from scipy.spatial.transform import Rotation

class PrimaryScore:
 def __init__(self):self.reset()
 def reset(self):self.last=-1;self.setting_witness=None;self.withdraw_streak=0;self.value=0.;self.latest={};self.release_streak=0;self.prev_pose=None
 def observe(self,row,G):
  control=row['control']
  if not isinstance(control,int)or control<=self.last:raise ValueError('Fresh unique actualscoretime required')
  if self.last>=0 and control!=self.last+1:raise ValueError('Actual scoregap')
  self.last=control;parts={p['name']:p for p in G['parts']};b=row['rigid_bodies'];pos={};R={}
  for n,d in b.items():
   a=np.array(d['pose_xyzw'],float)
   if a.shape!=(7,)or not np.isfinite(a).all()or np.linalg.norm(a[3:])<1e-8:raise ValueError('Invalidactualscorebody')
   pos[n]=a[:3];R[n]=Rotation.from_quat(a[3:]).as_matrix()
  origin=np.array(G['frame_origin_m']);axis=R['anchor_ring'][:,2];top=pos['anchor_ring']+axis*.0396875
  shoulder=pos['punch']+R['punch'][:,2]*.0414875
  gap=float(np.dot(shoulder-top,axis));lateral=float(np.linalg.norm((shoulder-top)-gap*axis))
  align=float(np.arccos(np.clip(np.dot(axis,R['punch'][:,2]),-1,1)))
  anchor_shift=np.linalg.norm(pos['anchor_ring']-origin);anchor_upright=float(np.arccos(np.clip(axis[2],-1,1)))
  wedge=(R['anchor_ring'].T@(pos['setting_wedge']-pos['anchor_ring']))[2]
  # Bodies use full common modelorigin; actualconegeometry startsz5mm.
  actual_down=-float(wedge)
  contacts=row.get('contacts',{});pairs=contacts.get('pairs',{});unavailable=contacts.get('unavailable_views',{})
  wall=[]
  for k in range(4):
   c=pairs.get(f'petal{k}_concrete')
   wall.append(None if c is None or'unknown'in c else int(np.array(c['counts']).sum())>0)
  current_anchor_seated=anchor_shift<.0008 and anchor_upright<np.deg2rad(2)
  current_material_good=all(np.isfinite(x.get('plastic_angle',np.nan))for x in row.get('material',{}).values())and len(row.get('material',{}))==4
  # Visible shoulderflush is the primary settingwitness; actual wedge/material/wall
  # readback prevent awarding a geometric proxy to a physically unset/broken mechanism.
  witness=(current_anchor_seated and-.0002<=gap<=.00015 and lateral<.0004 and align<np.deg2rad(2)
            and actual_down>=.0014 and all(x is True for x in wall)and current_material_good)
  if witness:self.setting_witness={'control':control,'gap_m':gap,'actual_wedge_down_m':actual_down}
  if self.setting_witness and not current_anchor_seated:self.setting_witness=None
  pverts=np.vstack([s['vertices']for s in parts['punch']['geoms']]);world=pverts@R['punch'].T+pos['punch'];zmin=float(((world-top)@axis).min())
  low=world.min(0);high=world.max(0);hc=np.array(G['frame_origin_m']);rad=.00635
  sideclear=float(max(low[0]-hc[0]-rad,hc[0]-rad-high[0],low[1]-hc[1]-rad,hc[1]-rad-high[1]))
  out=zmin>.01 or sideclear>.01
  retained=(self.setting_witness is not None and current_anchor_seated and all(x is True for x in wall))
  # Both poseincrement and actualnativevelocities must support a releasedstable claim.
  rv=np.array(b['anchor_ring']['native_velocity']);wv=np.array(b['setting_wedge']['native_velocity'])
  velocity_known_stable=np.isfinite(rv).all()and np.isfinite(wv).all()and np.linalg.norm(rv[:3])<.005 and np.linalg.norm(wv[:3])<.005 and np.linalg.norm(rv[3:])<.1 and np.linalg.norm(wv[3:])<.1
  self.withdraw_streak=self.withdraw_streak+1 if retained and out and velocity_known_stable else 0
  released,release_reason=self.released_tools_and_hands(row,G,parts,pos,R)
  tool_pose_now=np.array([pos['punch'],pos['hammer']])
  pose_quiet=(self.prev_pose is not None and float(np.max(np.linalg.norm(tool_pose_now-self.prev_pose,axis=1)))<.0005)
  self.prev_pose=tool_pose_now.copy()
  tool_velocity_quiet=all(np.linalg.norm(np.array(b[n]['native_velocity'])[:3])<.005 and np.linalg.norm(np.array(b[n]['native_velocity'])[3:])<.1 for n in ['punch','hammer'])
  self.release_streak=self.release_streak+1 if retained and out and released and velocity_known_stable and pose_quiet and tool_velocity_quiet else 0
  self.value=1. if self.release_streak>=21 else .75 if self.withdraw_streak>=11 else .5 if retained else 0.
  self.latest=dict(primary_score=self.value,setting_witness=self.setting_witness,shoulder_gap_m=gap,actual_wedge_down_m=actual_down,wall_contacts=wall,
    punch_min_axial_clearance_m=zmin,punch_wholebody_lateral_separation_m=sideclear,anchor_origin_shift_m=float(anchor_shift),stable_fresh_streak=self.withdraw_streak,
    native_velocity_stable=bool(velocity_known_stable),contacts_unknown=unavailable,fullscore_implemented=True,tool_release_clearance=released,tool_release_reason=release_reason,release_stable_fresh_streak=self.release_streak,
    limit='Actual0/.5/.75/1 withsamefrozeninputprofile. Fullscope/model/media/referenceevidence stillpending. Native angularvelocity/pose inconsistencies cannot be silently treatedstable.')
  return self.latest

 def released_tools_and_hands(self,row,G,parts,pos,R):
  # Ordinaryrectangularacceptancezones match the realtwoVblocks andhammercradle.
  # Stablegeometriczones do not use controllerphase or desiredpose.
  p=pos['punch'];h=pos['hammer'];ax=R['punch'][:,2]
  if not(.17<p[0]<.20 and .23<p[1]<.27 and .780<p[2]<.800 and abs(ax[0])>.995):return False,'PunchnotinactualhorizontalVrestzone'
  if np.linalg.norm(h-np.array(parts['hammer']['pos']))>.010 or np.linalg.norm(Rotation.from_matrix(R['hammer']).as_rotvec())>np.deg2rad(5):return False,'Hammernotinactualoriginalcradlezone'
  from yam_arena.analysis import link_hulls
  from scipy.spatial.transform import Rotation as Rot
  target=[]
  for name in ['punch','hammer','anchor_ring']:
   for sh in parts[name]['geoms']:
    v=np.array(sh['vertices'])@R[name].T+pos[name];target.append((v.min(0),v.max(0)))
  low=np.array([x[0]for x in target]);high=np.array([x[1]for x in target]);count=0
  for side in ['left','right']:
   rob=row.get('robots',{}).get(side,{})
   names=rob.get('body_names',[]);poses=rob.get('body_link_pose_w',[])
   if len(names)!=9 or set(names)!=set(link_hulls())or np.array(poses).shape!=(9,7)or not np.isfinite(poses).all():return False,'Missingactualcomplete9linkrobotposes'
   for name,pose in zip(names,poses):
    if name not in link_hulls():return False,'Unknownactualrobotbody'
    rr=Rot.from_quat(pose[3:]).as_matrix()
    for hull in link_hulls()[name]:
     w=hull.vertices@rr.T+pose[:3];count+=1
     # AABBpositiveaxisgap is a sufficient conservative fullgeometryseparation.
     if np.any(np.max(np.maximum(low-w.max(0),w.min(0)-high),axis=1)<.020):return False,'Completebodygeometrynot20mmclear'
  if count!=630:return False,'Incompleteactual630hullcoverage'
  return True,'Bothreleasedinordinarysupportzonesand630actualrobotpieces20mmclear'
