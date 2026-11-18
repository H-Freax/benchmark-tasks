"""Brush the marked bottom and inner shoulder of the loose narrow-neck bottle.

Use the bottle brush, then take it out and leave both items safely released.
This is a visible-film test fixture, not a biological cleaning/sterilization test.
Any ordinary support, hand use or grasp is permitted.
"""
from pathlib import Path
import hashlib,json,os,sys
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import native_geometry as ng
from cleaning import Film,transform,point
from pad_contacts import PadObserver
from yam_arena import station
GROOT=Path(os.environ.get('ROBOCURVE_GEOMETRY_ROOT',str(HERE.parent/'geometry')))
G=json.loads((GROOT/'geometry.json').read_text());OBJECTS=ng.objects(G)
SCENE='station';CONTROL_HZ=20.;SUCCESS_HOLD_S=.1;SCORE_OVER='end'
TRACE=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(HERE/'trace')))
FILM=Film(G);SERIAL=0;VIEW=None;LATEST=None;STABLE_START=None;LAST_PHYSICS_TICK=None;PADS=PadObserver();ACTION_CONTROL=0
def arr(v):
 v=getattr(v,'torch',v)
 if hasattr(v,'detach'):return v.detach().cpu().numpy().copy()
 if hasattr(v,'numpy'):return v.numpy().copy()
 return np.asarray(v)
def write(name,value):
 TRACE.mkdir(parents=True,exist_ok=True)
 (TRACE/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def body(base,name):
 view=base.scene[name].root_view
 pose=arr(view.get_transforms()).reshape(-1,7)[0].tolist()
 velocity=arr(view.get_velocities()).reshape(-1,6)[0].tolist()
 transform(pose)
 if not np.isfinite(velocity).all():raise RuntimeError('Nonfinite actual body velocity')
 return dict(pose=pose,velocity=velocity)
def actual_robot(base,side):
 robot=base.scene[side+'_robot'];view=robot.root_view
 q=arr(view.get_dof_positions()).reshape(-1,8)[0]
 links=arr(view.get_link_transforms()).reshape(-1,9,7)[0]
 if not np.isfinite(np.r_[q,links.reshape(-1)]).all():raise RuntimeError('Nonfinite native robot readback')
 return dict(joint_names=list(robot.backend_joint_names),body_names=list(robot.backend_body_names),q=q,links=links)
def actual_gripper_points(env):
 base=env.unwrapped;out={}
 for side in ['left','right']:
  robot=actual_robot(base,side);g=robot['links'][robot['body_names'].index('gripper')]
  R=transform(g)[:3,:3];p=np.array(g[:3]);half=abs(float(robot['q'][robot['joint_names'].index('joint7')]))+.008
  points=[p+R@np.array(v,float)for v in [(0,0,.06),(0,0,.134),(half,0,.147),(-half,0,.147),(half,0,.12),(-half,0,.12)]]
  out[side]=(points,2*half-.016)
 return out
def actual_partkit_pose(env,name):
 base=env.unwrapped;v=base.scene[name].root_view
 pose=arr(v.get_transforms()).reshape(-1,7)[0]
 velocity=arr(v.get_velocities()).reshape(-1,6)[0]
 origin=arr(base.scene.env_origins).reshape(-1,3)[0]
 return pose[:3]-origin,pose[3:7],velocity[:3]
ng.pk.gripper_points=actual_gripper_points
ng.pk.pose=actual_partkit_pose

@ng.pk.on_reset
def reset(env):
 global FILM,SERIAL,VIEW,LATEST,STABLE_START,LAST_PHYSICS_TICK,PADS,ACTION_CONTROL
 FILM=Film(G);SERIAL=0;VIEW=None;LATEST=None;STABLE_START=None;LAST_PHYSICS_TICK=None;PADS=PadObserver();ACTION_CONTROL=0
 # Only visible material state resets; no body pose/velocity writes or forces.
 from pxr import UsdGeom
 base=env.unwrapped;root=base.scene.env_prim_paths[0]
 for patch in G['patches']:
  prim=base.scene.stage.GetPrimAtPath(root+f"/feeding_bottle/body/geom_{patch['geom_index']}")
  if prim:UsdGeom.Imageable(prim).MakeVisible()
def bind(base):
 global VIEW
 from pxr import Usd,UsdPhysics,PhysxSchema
 from isaaclab_physx.physics.physx_manager import PhysxManager
 import omni.physx
 root=base.scene.env_prim_paths[0];stage=base.scene.stage
 sensor=root+'/bottle_brush/body';target=root+'/feeding_bottle/body'
 view=PhysxManager.get_physics_sim_view().create_rigid_contact_view([sensor],filter_patterns=[[target]],max_contact_data_count=4096)
 if view is None:raise RuntimeError('Actual brush-bottle contact view unavailable; force is UNKNOWN')
 inv={};shapes=[]
 for p in G['parts']:
  v=base.scene[p['name']].root_view
  expected=sum(s['collision_enabled']for s in p['geoms'])
  actual=int(v.max_shapes)
  if int(v.count)!=1 or actual!=expected:raise RuntimeError(f"Nativebodyshape mismatch {p['name']}:{actual}/{expected}")
  inv[p['name']]=dict(body_count=int(v.count),shape_count=actual,mass_kg=arr(v.get_masses()).reshape(-1).tolist())
 for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
  path=str(prim.GetPath())
  if prim.HasAPI(UsdPhysics.CollisionAPI)and any(path.startswith(root+'/'+p['name']+'/')for p in G['parts']):shapes.append(dict(path=path))
 if len(shapes)!=102:raise RuntimeError('Native collider inventory differs')
 material=stage.GetPrimAtPath(root+'/bottle_brush/Looks/material_0')
 ma=PhysxSchema.PhysxMaterialAPI(material)
 compliance=dict(stiffness_N_m=ma.GetCompliantContactStiffnessAttr().Get(),damping_Ns_m=ma.GetCompliantContactDampingAttr().Get(),acceleration_spring=ma.GetCompliantContactAccelerationSpringAttr().Get())
 if compliance['stiffness_N_m']!=500. or compliance['acceleration_spring']:raise RuntimeError('Head compliance setting mismatch')
 write('NATIVE-INVENTORY.json',dict(passed=True,rigid_only=True,bodies=inv,shapes=shapes,
  actual_physics_stats=omni.physx.get_physxunittests_interface().get_physics_stats(),
  head_material_readback=compliance,task_joints=0,contacts=dict(sensor=sensor,target=target),
  stock_rig='yam_arena.station',body_pose_writes_after_reset=0,external_task_wrenches=0,formal_score=None))
 stage.Export(str(TRACE/'authored.usda'));VIEW=view
def observe(env):
 global SERIAL,LATEST,STABLE_START,LAST_PHYSICS_TICK
 base=env.unwrapped;tick=int(base._sim_step_counter)
 if LAST_PHYSICS_TICK==tick:return
 if LAST_PHYSICS_TICK is not None and tick<LAST_PHYSICS_TICK:raise RuntimeError("Actual physics clock reversed")
 LAST_PHYSICS_TICK=tick;SERIAL+=1
 if VIEW is None:bind(base)
 bodies={p['name']:body(base,p['name'])for p in G['parts']}
 raw=[arr(v)for v in VIEW.get_contact_data(dt=base.physics_dt)]
 force,position,normal,separation,counts,starts=raw
 if not np.isfinite(counts).all()or not np.isfinite(starts).all():raise RuntimeError('Invalid native contact extent')
 extent=int(np.max(counts+starts))if counts.size else 0
 if extent>len(force):raise RuntimeError('Truncated native contact buffer')
 native_contacts=[dict(normal_force_N=float(force.reshape(-1)[i]),world_point_m=position.reshape(-1,3)[i].tolist(),
  world_normal=normal.reshape(-1,3)[i].tolist(),separation_m=float(separation.reshape(-1)[i]))for i in range(extent)]
 row=dict(serial=SERIAL,control=ACTION_CONTROL,control_authority='actual embodiment.step call count excluding reset settling',time_s=tick*float(base.physics_dt),actual_physics_tick=tick,bottle_pose=bodies['feeding_bottle']['pose'],
  brush_pose=bodies['bottle_brush']['pose'],bodies=bodies,native_contacts=native_contacts,
  physics_dt_s=float(base.physics_dt),contact_counts=counts.tolist(),contact_starts=starts.tolist())
 result=FILM.update(row);coverage=FILM.coverage();row['material_update']=result;row['coverage']=coverage
 from pxr import UsdGeom
 root=base.scene.env_prim_paths[0]
 for i in result['removed']:
  patch=G['patches'][i];UsdGeom.Imageable(base.scene.stage.GetPrimAtPath(root+f"/feeding_bottle/body/geom_{patch['geom_index']}")).MakeInvisible()
 grippers=ng.pk.gripper_points(env)
 points=np.array([p for _,(ps,gap)in grippers.items()for p in ps])
 rb=bodies['feeding_bottle'];rh=bodies['bottle_brush']
 # Hands must clear each complete body AABB, using all source shape vertices.
 from grader import released
 pad_observation=PADS.observe(base)
 # Every actual robot rigid-body partner must be observed, and all pair counts
 # zero, before a clear TCP can count as released. Missing is never zero.
 all_robot_contacts_clear=all(v['count']==0 for pairs in pad_observation['pairs'].values()for v in pairs.values())and not np.asarray(pad_observation['tangent_counts']).any()
 terminal=released(G,bodies,points)and all_robot_contacts_clear
 row['all18_robot_partners_known_zero']=all_robot_contacts_clear
 if coverage['all']>=.90 and coverage['bottom']>=.80 and coverage['shoulder']>=.80 and terminal:
  if STABLE_START is None:STABLE_START=row['time_s']
 else:STABLE_START=None
 row['released']=terminal;row['stable_s']=0. if STABLE_START is None else row['time_s']-STABLE_START
 row['pad_contacts']=pad_observation
 if SERIAL==1:write('ACTUAL-18-PARTNER-BINDING.json',PADS.binding)
 row['hand_points']=points.tolist();row['robots']={side:{k:(v.tolist()if isinstance(v,np.ndarray)else v)for k,v in actual_robot(base,side).items()}for side in ['left','right']};LATEST=row
 with(TRACE/'observations.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 if SERIAL%20==0:print('BOTTLE_ACTUAL '+json.dumps(dict(serial=SERIAL,coverage=coverage,released=terminal)),flush=True)
def score(states):
 if LATEST is None:return 0.
 c=LATEST['coverage']
 if LATEST['released']and LATEST['stable_s']>=2. and c['all']>=.90 and min(c['bottom'],c['shoulder'])>=.80:return 1.
 return .50 if c['all']>=.60 else .25 if c['all']>=.25 else 0.
def success(states):return score(states)==1.

@ng.pk.on_step
def observe_fail_closed(env):
 try:
  observe(env)
 except Exception as error:
  import traceback
  TRACE.mkdir(parents=True,exist_ok=True)
  path=TRACE/'FIRST-OBSERVER-ERROR.json'
  if not path.exists():
   write(path.name,dict(status='READBACK_FAILURE_FAIL_CLOSED',exception_type=type(error).__name__,
    exception=str(error),traceback=traceback.format_exc(),formal_score=None))
  raise


# Preserve actual issued-action count separately from observation serial and
# physics tick. Initial reset settling rows have control0, never fake controls.
_previous_inspect=ng.pk._Finder.targets['yam_arena.inspect_embodiment']
def patch_action_clock(mod):
 _previous_inspect(mod);cls=mod.YamArenaEmbodiment
 if getattr(cls,'_bottle_action_clock',False):return
 old_step=cls.step
 def step(self,action):
  global ACTION_CONTROL
  ACTION_CONTROL+=1
  return old_step(self,action)
 cls.step=step;cls._bottle_action_clock=True
ng.pk._Finder.targets['yam_arena.inspect_embodiment']=patch_action_clock
if 'yam_arena.inspect_embodiment'in sys.modules:patch_action_clock(sys.modules['yam_arena.inspect_embodiment'])
