"""Native rigid stopcock hinge and read-only liquid/terminal observations."""
from pathlib import Path
import json,math,os,traceback
import numpy as np
from scipy.spatial.transform import Rotation
import partkit as pk
from scene_spec import specs,CENTER,VALVE,STOPPER,STOPPER_TRAY,CUP_TARGET,OUTLET,TABLE
from fluid_model import Fluids,height_ml,RADIUS
from grade import grade
from readback import robots as actual_robots, array as actual_array
from passive_retention import author_retention

HERE=Path(__file__).resolve().parent
TRACE=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(HERE/'trace')))
OBJECTS=specs(pk)
F=Fluids();LAST=None;LAST_ROW=None;COUNT=0;STABLE=0.;LAST_STEP=None;LAST_P=None;LAST_ANGLE=0.
RELEASE_VIEW=None


def arr(x):return actual_array(x)

def native_pose(base,name):
 a=base.scene[name];p=arr(a.root_view.get_transforms())[0];v=arr(a.root_view.get_velocities())[0]
 if p.shape!=(7,)or not np.isfinite(p).all()or not np.isfinite(v).all():raise ValueError('NonfiniteActual:'+name)
 return p,v


_ORIGINAL_SPAWN=pk._spawn_part
def spawn_observable(*a,**kw):
 prim=_ORIGINAL_SPAWN(*a,**kw)
 from pxr import PhysxSchema,UsdShade,Usd,Sdf
 from isaaclab.sim.utils.stage import get_current_stage
 stage=get_current_stage();path=str(prim.GetPath())
 # Only the glass material is translucent, stand/handles remain opaque.
 for mat in [stage.GetPrimAtPath(path+'/Looks/c0')]:
  if mat and path.endswith(('/funnel_stand','/receiver')):
   for child in Usd.PrimRange(mat):
    if child.IsA(UsdShade.Shader):UsdShade.Shader(child).CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.02)
 if any(path.endswith('/'+n)for n in ['stopper','stopcock','receiver']):
  PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path+'/body')).CreateThresholdAttr(0.)
 return prim
pk._spawn_part=spawn_observable


def release_contacts(base):
 global RELEASE_VIEW
 try:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics,Usd
  root=base.scene.env_prim_paths[0]
  sensors=[root+'/'+n+'/body'for n in ['stopper','stopcock','receiver']]
  if RELEASE_VIEW is None:
   paths=[str(p.GetPath())for p in base.scene.stage.Traverse(Usd.TraverseInstanceProxies())if p.HasAPI(UsdPhysics.RigidBodyAPI)]
   filters=[]
   names={'base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right'}
   for arm in ['LeftArm','RightArm']:
    ps=sorted(p for p in paths if p.startswith(root+'/'+arm+'/'))
    if len(ps)!=9 or {p.rsplit('/',1)[-1]for p in ps}!=names:raise ValueError('incomplete_actual_robot_inventory')
    filters.extend(ps)
   RELEASE_VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*3,max_contact_data_count=8192)
   if RELEASE_VIEW is None or getattr(RELEASE_VIEW,'_backend',None)is None:raise ValueError('missing_actual_release_contact_backend')
  v=RELEASE_VIEW
  if list(v.sensor_paths)!=sensors or v.sensor_count!=3 or v.filter_count!=18 or len(v.filter_paths)!=3:raise ValueError('release_view_identity_mismatch')
  actual_filters=[list(f)for f in v.filter_paths]
  if any(len(f)!=18 or len(set(f))!=18 for f in actual_filters)or actual_filters[1:]!=actual_filters[:1]*2:raise ValueError('release_view_partner_order_mismatch')
  for f in actual_filters:
   for arm in ['LeftArm','RightArm']:
    ps=[p for p in f if p.startswith(root+'/'+arm+'/')]
    if len(ps)!=9 or {p.rsplit('/',1)[-1]for p in ps}!={'base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right'}:raise ValueError('release_filter_fullbody_coverage_mismatch')
  raw=v.get_contact_data(dt=float(base.physics_dt));counts=arr(raw[4]);starts=arr(raw[5])
  if counts.shape!=(3,18)or not np.isfinite(counts).all()or np.any(counts<0)or np.any(counts!=np.floor(counts))or counts.sum()>=8192:raise ValueError('invalid_or_saturated_actual_counts')
  return dict(known=True,sensors=sensors,filters=actual_filters,counts=counts.tolist(),starts=starts.tolist(),all_robot_contacts_zero=bool(np.sum(counts)==0),physics_step=int(base._sim_step_counter))
 except Exception as exc:return dict(known=False,reason=type(exc).__name__+':'+str(exc),all_robot_contacts_zero=False)


def author_joint(stage,root):
 from pxr import UsdPhysics,Gf,Sdf
 path=root+'/phase_stopcock_hinge'
 if stage.GetPrimAtPath(path):return
 joint=UsdPhysics.RevoluteJoint.Define(stage,path)
 joint.CreateBody0Rel().SetTargets([Sdf.Path(root+'/funnel_stand/body')])
 joint.CreateBody1Rel().SetTargets([Sdf.Path(root+'/stopcock/body')])
 joint.CreateAxisAttr('Y');joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,VALVE-CENTER)));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
 joint.CreateLocalRot0Attr(Gf.Quatf(1,Gf.Vec3f(0,0,0)));joint.CreateLocalRot1Attr(Gf.Quatf(1,Gf.Vec3f(0,0,0)))
 joint.CreateLowerLimitAttr(-2.);joint.CreateUpperLimitAttr(92.);joint.CreateCollisionEnabledAttr(False)
 # Passive bounded velocity drag approximates retained PTFE plug resistance.
 # Zero stiffness and zero target velocity: no selected-angle attraction.
 author_retention(joint)


def patch_scene(module):
 old=module.InteractiveScene.__init__
 def init(self,*a,**kw):
  old(self,*a,**kw);author_joint(self.stage,self.env_prim_paths[0])
  from types import SimpleNamespace
  from phase_visuals import draw_phases
  from scene_spec import CUP_START
  initial={'receiver':(np.array([*CUP_START,0.,0.,0.,1.]),np.zeros(6))}
  draw_phases(SimpleNamespace(scene=self),initial,F)
 module.InteractiveScene.__init__=init
pk._Finder.targets['isaaclab.scene.interactive_scene']=patch_scene


@pk.on_reset
def reset(env):
 global F,LAST,LAST_ROW,COUNT,STABLE,LAST_STEP,LAST_P,LAST_ANGLE,RELEASE_VIEW
 RELEASE_VIEW=None
 F=Fluids();LAST=LAST_ROW=None;COUNT=0;STABLE=0.;LAST_STEP=None;LAST_P=None;LAST_ANGLE=0.


def seal_open(stopper):
 # Native rigid seat: aligned stem within9mm XY and below20mm pull remains
 # sealed. Lateral removal also admits air, rather than forcing one grasp path.
 return bool(np.linalg.norm(stopper[:2]-STOPPER[:2])>.014 or stopper[2]-STOPPER[2]>.024)


def draw(base,poses):
 from phase_visuals import draw_phases
 draw_phases(base,poses,F)


@pk.on_step
def observe(env):
 global LAST,LAST_ROW,COUNT,STABLE,LAST_STEP,LAST_P,LAST_ANGLE
 base=env.unwrapped;step=int(base._sim_step_counter)
 if LAST_STEP is not None and step<=LAST_STEP:return
 fresh_contiguous=LAST_STEP is None or step-LAST_STEP==int(base.cfg.decimation)
 if not fresh_contiguous:STABLE=0.
 dt=float(base.step_dt);LAST_STEP=step;COUNT+=1
 observer_stage='rigid_poses'
 try:
  poses={n:native_pose(base,n)for n in ['funnel_stand','stopper','stopcock','receiver','stopper_dish']}
  pp,vp=poses['stopcock'];R=Rotation.from_quat(pp[3:]).as_matrix();angle=math.atan2(-R[2,0],R[0,0]);cp,cv=poses['receiver'];Rc=Rotation.from_quat(cp[3:]).as_matrix();tilt=math.acos(np.clip(Rc[2,2],-1,1))
  # Downward stream intersection with actual receiver mouth, not start layout.
  lip=cp[:3]+Rc@np.array([0,0,.102]);caught=bool(np.linalg.norm(lip[:2]-OUTLET[:2])<=.027 and lip[2]<=OUTLET[2]+.005 and lip[2]>=OUTLET[2]-.030 and tilt<math.radians(20))
  F.step(dt,angle=angle,vented=seal_open(poses['stopper'][0]),caught=caught,cup_tilt=tilt)
  observer_stage='actual_robot_tensors'
  grips,robot_states,robot_links=actual_robots(base);positions={s:g[0][1]for s,g in grips.items()}
  observer_stage='release_contact_counts'
  release=release_contacts(base)
  actual_open=all(grips[s][1]>.070 for s in ['left','right'])
  stopperp=poses['stopper'][0][:3]
  stopper_R=Rotation.from_quat(poses['stopper'][0][3:]).as_matrix()
  stopper_local=np.array([[x,y,z]for x in [-.0125,.0125]for y in [-.0125,.0125]for z in [-.008,.008]]+[[.01098*np.cos(a),.01098*np.sin(a),z]for a in np.arange(32)*np.pi/16 for z in [-.036,-.008]])
  stopper_vertices=stopper_local@stopper_R.T+stopperp
  stopper_safe=bool(np.all(np.abs(stopper_vertices[:,:2]-STOPPER_TRAY[:2])<=[.041,.036])and stopper_vertices[:,2].min()>=TABLE+.004 and stopper_vertices[:,2].max()<=TABLE+.070)
  receiver_supported=abs(cp[2]-TABLE)<.006 and np.linalg.norm(cv[:3])<.01 and np.linalg.norm(cv[3:])<.1 and tilt<math.radians(5)
  clear=all(min(np.linalg.norm(np.asarray(p)-cp[:3]-Rc@np.array([0,.055,.060])),np.linalg.norm(np.asarray(p)-pp[:3]),np.linalg.norm(np.asarray(p)-stopperp))>.035 for p in positions.values())
  clear=bool(clear and release['known'] and release['all_robot_contacts_zero'] and actual_open)
  stable=receiver_supported and clear and abs(angle)<=math.radians(3)and abs(F.flow_ml_s)<.005 and np.linalg.norm(poses['stopper'][1][:3])<.01
  STABLE=STABLE+dt if stable else 0.
  row=dict(known=F.invalid_reason is None and release['known'],release_contact_observation=release,fresh_contiguous=fresh_contiguous,actual_grippers_open=actual_open,fluids=F.snapshot(),closed=bool(abs(angle)<=math.radians(3)),stopper_safe=stopper_safe,receiver_released=bool(receiver_supported and clear),robot_clear=clear,stable_s=STABLE,flow_ml_s=F.flow_ml_s,
           control=COUNT,physics_step=step,time_s=step*float(base.physics_dt),valve_angle_rad=angle,vented=seal_open(poses['stopper'][0]),caught=caught,receiver_tilt_rad=tilt,
           poses={n:p.tolist()for n,(p,v)in poses.items()},velocities={n:v.tolist()for n,(p,v)in poses.items()},tcp={s:np.asarray(p).tolist()for s,p in positions.items()},robot_joint_positions={s:[robot_states[s]['joint_positions'][robot_states[s]['joint_names'].index(n)]for n in ['joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8']]for s in ['left','right']},actual_robot_links=robot_links)
  row['score']=grade(row);LAST_ROW=row;LAST=poses;observer_stage='render_only_liquid_visual';draw(base,poses)
 except Exception as exc:
  TRACE.mkdir(parents=True,exist_ok=True)
  error_path=TRACE/'FIRST-OBSERVER-ERROR.txt'
  if not error_path.exists():error_path.write_text('observer_stage='+observer_stage+'\n'+traceback.format_exc())
  LAST_ROW=dict(known=False,reason=type(exc).__name__+':'+str(exc),observer_stage=observer_stage,traceback_file=str(error_path),control=COUNT,physics_step=step,score=0.)
 TRACE.mkdir(parents=True,exist_ok=True)
 with(TRACE/'observations.jsonl').open('a')as f:f.write(json.dumps(LAST_ROW,allow_nan=False)+'\n')
 (TRACE/'LATEST.json').write_text(json.dumps(LAST_ROW,indent=2)+'\n')
