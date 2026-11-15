"""Full-size folding-rule task on the actual YAM station.

Open-loop reference targets come from the documented reset fixture, never from
simulator object poses. Native observations are used only by material forces,
contact/geometry scoring and diagnostics. No scored object pose writes.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).parent))
import mechkit as mk
import folding_rule as fr
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP

SCENE='station';TOP=TABLE_TOP[SCENE];DT=.05
PITCH=fr.PITCH;LAYER=fr.LAYER
BASE=np.array([.30,.10,TOP+.0015+.0003])
YAW=-math.pi/2;R0=fr.rz(YAW)
POCKET=np.array([.32,.18]);FLOOR=TOP+.004
BRANCH=os.environ.get('FOLD_BRANCH','action')
OPEN_JOINTS=int(os.environ.get('FOLD_OPEN_JOINTS','2'))
FRICTION=float(os.environ.get('FOLD_FRICTION_NM','.002'))
DETENT=float(os.environ.get('FOLD_DETENT_NM_RAD','.012'))
PRELOAD=math.radians(float(os.environ.get('FOLD_PRELOAD_DEG','20')))
MASS_SCALE=float(os.environ.get('FOLD_MASS_SCALE','1'))
WOOD_STATIC=float(os.environ.get('FOLD_WOOD_STATIC','.4'))
WOOD_DYNAMIC=float(os.environ.get('FOLD_WOOD_DYNAMIC','.3'))
assert BRANCH in ('action','stop') and OPEN_JOINTS in (1,2) and 0<=FRICTION<=.02 and 0<=DETENT<=.1 and .5<=MASS_SCALE<=3
assert 0<=PRELOAD<=math.radians(45)
assert 0<=WOOD_DYNAMIC<=WOOD_STATIC<=1.
angles=np.full(9,math.pi);angles[-1]=0.
if OPEN_JOINTS==2:
    angles[-2:]=0.;BASE[:2]=(.36,.205);POCKET=np.array([.32,-.18])
if BRANCH=='stop':
    angles[:]=math.pi;BASE=np.array([POCKET[0]-PITCH/2,POCKET[1],FLOOR+.0015+.0003]);YAW=0.;R0=fr.rz(YAW)

kit=mk.Kit()
for i in range(10):
    # Reference is fully extended, actual spawn uses native joint reset angles.
    origin=BASE+R0@np.array([i*PITCH,0,i*LAYER])
    kit.part(f'leaf_{i}',origin,[mk.box((fr.LEAF_LENGTH,fr.WIDTH,fr.THICKNESS),at=(PITCH/2,0,0))],
        mass=650*fr.LEAF_LENGTH*fr.WIDTH*fr.THICKNESS*MASS_SCALE,
        color=(.88,.79,.52) if i%2 else (.94,.91,.76),contact_offset=.0002,
        solver_position_iterations=64,yaw=YAW)
    if i:
        kit.hinge(f'hinge_{i}',f'leaf_{i-1}',f'leaf_{i}',anchor=origin,axis=(0,0,1),
                  lower_deg=0,upper_deg=180,start_deg=math.degrees(angles[i-1]),
                  stiffness=DETENT,rest_deg=-math.degrees(PRELOAD) if angles[i-1]<math.pi/2 else 180+math.degrees(PRELOAD),
                  damping=.0003,friction=FRICTION)
        kit.joints_[f'hinge_{i}'].angular_coordinate_offset=math.pi/2
sx,sy,sz=fr.POCKET_INNER
kit.part('pocket_floor',(*POCKET,TOP+.002),[mk.box((sx+.008,sy+.008,.004))],color=(.27,.31,.35))
for axis,sign in [(0,-1),(0,1),(1,-1),(1,1)]:
    center=np.array([*POCKET,FLOOR+sz/2]);center[axis]+=sign*(fr.POCKET_INNER[axis]/2+.002)
    size=np.array([sx+.008,sy+.008,sz]);size[axis]=.004
    kit.part(f'pocket_wall_{axis}_{"p" if sign>0 else "m"}',center,[mk.box(tuple(size))],color=(.27,.31,.35))
OBJECTS=kit.objects()

# This independent helper copy is shared own-task infrastructure. Retain actual
# connected-leaf contact; the generic helper's usual adjacent-pair disable would
# omit the very link interference that folding should resolve.
_author0=mk._author_joint
def _author_joint(stage,k,j,paths,env_T,suffix=''):
    path=_author0(stage,k,j,paths,env_T,suffix)
    if path and k is kit and j.kind=='hinge':
        from pxr import Gf,UsdPhysics
        joint=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(path))
        joint.GetCollisionEnabledAttr().Set(True)
        # PhysX D6 twist cannot represent an upper limit beyond180deg. Center
        # the native coordinate at physical90deg, without moving any body.
        q=joint.GetLocalRot1Attr().Get()
        offset=Gf.Quatf(math.sqrt(.5),Gf.Vec3f(-math.sqrt(.5),0,0))
        joint.GetLocalRot1Attr().Set(q*offset)
        joint.GetLowerLimitAttr().Set(math.degrees(j.lower-j.angular_coordinate_offset))
        joint.GetUpperLimitAttr().Set(math.degrees(j.upper-j.angular_coordinate_offset))
        from pxr import UsdGeom
        p0=stage.GetPrimAtPath(paths[j.parent]);p1=stage.GetPrimAtPath(paths[j.child])
        w0=UsdGeom.Xformable(p0).ComputeLocalToWorldTransform(0)
        w1=UsdGeom.Xformable(p1).ComputeLocalToWorldTransform(0)
        a0=np.array(w0.Transform(Gf.Vec3d(*joint.GetLocalPos0Attr().Get())))
        a1=np.array(w1.Transform(Gf.Vec3d(*joint.GetLocalPos1Attr().Get())))
        def qr(q):return mk.quat_to_mat((*q.GetImaginary(),q.GetReal()))
        r0=np.array(w0)[:3,:3].T@qr(joint.GetLocalRot0Attr().Get())
        r1=np.array(w1)[:3,:3].T@qr(joint.GetLocalRot1Attr().Get())
        rel=r0.T@r1;native=math.degrees(math.atan2(rel[2,1],rel[1,1]))
        print('[fold-native-author]',dict(joint=j.name,anchor_error_m=float(np.linalg.norm(a0-a1)),
              native_start_deg=native,logical_start_deg=math.degrees(j.start),
              limits_deg=[joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()],
              child_mass_kg=UsdPhysics.MassAPI(p1).GetMassAttr().Get(),
              drive_deg=UsdPhysics.DriveAPI(joint.GetPrim(),'angular').GetTargetPositionAttr().Get()),flush=True)
    return path
mk._author_joint=_author_joint

# Spawn-time material and PhysxContactReportAPI are authored inside this task's
# helper before scene patching; overriding a captured spawn factory here would
# be too late because mk import already installed it.

CONTACT={'view':None,'sim':None,'error':None}
def support_force():
    if CONTACT['error']:return 0.,False
    try:
        if 'setting_before_after' not in CONTACT:
            from isaaclab.sim import SimulationContext
            ctx=SimulationContext.instance();path='/physics/disableContactProcessing'
            before=ctx.get_setting(path);ctx.set_setting(path,False);after=ctx.get_setting(path)
            CONTACT['setting_before_after']={'path':path,'before':before,'after':after}
            print('[fold-contact-setting-runtime]',CONTACT['setting_before_after'],flush=True)
            if after is not False:raise RuntimeError('Runtime contact processing remains disabled')
        if CONTACT['view'] is None:
            from isaaclab_physx.physics import PhysxManager
            paths=mk._SIM['paths'];floor=paths.get('pocket_floor')
            leaves=[paths.get(f'leaf_{i}') for i in range(10)]
            if not floor or not all(leaves):return 0.,False
            # Use the installed runtime's actual stage-bound physics view.
            # Creating another default-stage Torch view can return empty actors
            # with no exception in this multi-stage SDK.
            sim=PhysxManager.get_physics_sim_view()
            if sim is None:return 0.,False
            body_view=sim.create_rigid_body_view(leaves)
            if body_view.count!=10:raise RuntimeError(f'Expected10 live leaf actors, got{body_view.count}')
            # Static floor has no RigidBodyAPI on its parent Xform. Match its
            # actual collision shape, as in the installed ContactSensor tests.
            floor_shape=floor+'/box0/geometry/mesh'
            view=sim.create_rigid_contact_view(leaves,filter_patterns=[[floor_shape] for _ in leaves],max_contact_data_count=256)
            CONTACT.update(view=view,sim=sim,bodies=body_view)
            print('[fold-contact-view]',dict(sensors=getattr(view,'sensor_names',leaves),
                  filters=getattr(view,'filter_names',[floor_shape])),flush=True)
        force=CONTACT['view'].get_contact_force_matrix(dt=1/240)
        raw=getattr(force,'torch',force)
        val=raw.detach().cpu().numpy() if hasattr(raw,'detach') else raw.numpy()
        net=CONTACT['view'].get_net_contact_forces(dt=1/240)
        raw_net=getattr(net,'torch',net);net_arr=raw_net.detach().cpu().numpy() if hasattr(raw_net,'detach') else raw_net.numpy()
        CONTACT['last_force_shapes']={'matrix':list(val.shape),'net':list(net_arr.shape),
                                      'matrix_abs_max':float(np.max(np.abs(val))) if val.size else 0.,
                                      'net_abs_max':float(np.max(np.abs(net_arr))) if net_arr.size else 0.}
        return float(np.sum(np.abs(val[...,2]))),True
    except Exception as e:
        CONTACT['error']=type(e).__name__+': '+str(e)
        print('[fold-contact-unavailable]',CONTACT['error'],flush=True)
        return 0.,False

def _detents(objects):
    """Passive over-center spring potential, no pose/joint enable changes.

    Each hinge has stable rest0 and180; branch follows actual angle through90.
    The drive continuously applies native bounded engineering spring torque.
    This is a disclosed material model, not reference-policy feedback.
    """
    from pxr import UsdPhysics
    for name,j in kit.joints_.items():
        q=kit.q(objects,name);rest=-PRELOAD if abs(q)<math.pi/2 else math.pi+PRELOAD
        j.rest=rest
        path=mk._SIM['joint_paths'].get((kit.id,name))
        if path:
            stick=mk._SIM['stick'].get((kit.id,name),j.start)
            drive=UsdPhysics.DriveAPI(mk._SIM['stage'].GetPrimAtPath(path),'angular')
            drive.GetTargetPositionAttr().Set(math.degrees(mk._drive_target(j,stick)))

STATE={'hold':0.,'fallen':False,'last':-1,'score':0.,'trace':[]}
def _update(objects):
    step=mk._EP['step']
    if step==STATE['last']:return
    if step<STATE['last']:
        STATE.update(hold=0.,fallen=False,trace=[])
        CONTACT.update(view=None,sim=None,error=None)
    STATE['last']=step
    if step==1:
        for i in range(10):
            rigid=mk._EP['env'].unwrapped.scene.rigid_objects[f'leaf_{i}']
            try:
                def array(v):
                    v=getattr(v,'torch',v)
                    if hasattr(v,'detach'):return v.detach().cpu().numpy().tolist()
                    if hasattr(v,'numpy'):return v.numpy().tolist()
                    return np.asarray(v).tolist()
                mass=array(rigid.root_physx_view.get_masses())
                inertia=array(rigid.root_physx_view.get_inertias())
                print('[fold-native-mass]',dict(leaf=i,mass=mass,inertia=inertia),flush=True)
            except Exception as e:
                print('[fold-native-mass-unavailable]',type(e).__name__,str(e),flush=True)
    centers=[];rotations=[];linear=[];angular=[]
    for i in range(10):
        st=objects[f'leaf_{i}'];p,q=kit.pose(objects,f'leaf_{i}');r=mk.quat_to_mat(q)
        centers.append(p+r@np.array([PITCH/2,0,0]));rotations.append(r)
        linear.append(st.lin_vel);angular.append(st.ang_vel)
    corners=fr.collision_corners(centers,rotations)
    lo=np.r_[POCKET-fr.POCKET_INNER[:2]/2,FLOOR-.0006]
    hi=np.r_[POCKET+fr.POCKET_INNER[:2]/2,FLOOR+fr.POCKET_INNER[2]]
    inside=bool(np.all(corners>=lo) and np.all(corners<=hi))
    STATE['fallen']|=bool(np.min(corners[:,:,2])<TOP-.03)
    boxes=mk.robot_collision_boxes();clearance=float('inf')
    for cs in corners:
        low,high=cs.min(0),cs.max(0)
        for blo,bhi in boxes:
            gap=np.maximum(np.maximum(low-bhi,blo-high),0.)
            clearance=min(clearance,float(np.linalg.norm(gap)))
    released=bool(boxes) and clearance>=.005
    slow=max(np.linalg.norm(linear,axis=1))<.015 and max(np.linalg.norm(angular,axis=1))<.1
    force,contact_complete=support_force();supported=contact_complete and force>.05
    stable=inside and released and slow and supported and not STATE['fallen']
    STATE['hold']=STATE['hold']+DT if stable else 0.
    value=(0. if STATE['fallen'] else 1. if STATE['hold']>=1. else
           .5 if inside and supported else .25 if fr.compact_progress(corners) else 0.)
    STATE['score']=value
    row=dict(step=step,score=value,inside=inside,force_N=force,contact_complete=contact_complete,
             released=released,clearance_m=clearance,slow=bool(slow),hold_s=STATE['hold'],
             q_deg=[round(kit.q_deg(objects,f'hinge_{i}'),3) for i in range(1,10)],
             spring_torque_Nm=[round(j.stiffness*(j.rest-kit.q(objects,name)),6) for name,j in kit.joints_.items()],
             envelope_low=corners.reshape(-1,3).min(0).tolist(),
             envelope_high=corners.reshape(-1,3).max(0).tolist(),
             root_pose={'origin':kit.pose(objects,'leaf_0')[0].tolist(),
                        'quat_xyzw':kit.pose(objects,'leaf_0')[1].tolist()},
             actual_tcps=[np.asarray(p).tolist() for p in mk.tcps()])
    STATE['trace'].append(row)
    if step%10==0:print('[fold-trace]',row,flush=True)

kit.on_step(_detents);kit.on_step(_update)
def score(objects):_update(objects);return STATE['score']
def success(objects):return score(objects)>=1.

TIP=PAD_TIP_PAST_TCP
def _reference():
    """Fixed Cartesian targets from visible documented reset fixture only."""
    if BRANCH=='stop':
        return {'left':[Waypoint(2.,(.18,.36,TOP+.20),FINGER_OPEN,'inspect completed placement')],
                'right':[Waypoint(2.,(.18,-.36,TOP+.20),FINGER_OPEN,'inspect completed placement')]}
    variant=os.environ.get('DEMO_VARIANT','')
    if variant=='always_stop':
        return {'left':[Waypoint(2.,(.18,.36,TOP+.16),FINGER_OPEN,'inspect and incorrectly stop')],
                'right':[Waypoint(2.,(.18,-.36,TOP+.16),FINGER_OPEN,'inspect and incorrectly stop')]}
    if OPEN_JOINTS==2:
        from two_leaf_reference import make_reference
        from types import SimpleNamespace
        # yam_arena.task_file.load executes a path module without registering
        # it in sys.modules; pass the immutable reset constants explicitly.
        plan=make_reference(SimpleNamespace(**globals()))
        if variant=='hold_above':
            waypoints=plan['right'];cut=next(i for i,w in enumerate(waypoints) if w.label=='release above pocket')
            hold=waypoints[cut-1]
            plan['right']=waypoints[:cut]+[Waypoint(6.,hold.pos,FINGER_CLOSED,'deliberately keep tool held above target',hold.yaw)]
            left=plan['left'];duration=sum(w.duration for w in plan['right'])
            elapsed=0.;trim=[]
            for w in left:
                d=min(w.duration,duration-elapsed)
                if d<=0:break
                trim.append(Waypoint(d,w.pos,w.finger,w.label,w.yaw));elapsed+=d
            plan['left']=trim
        elif variant:
            raise ValueError('Unknown DEMO_VARIANT:'+variant)
        return plan
    # Visible fixture rotated toward the two arm workspaces, preserving size.
    pivot=BASE+R0@np.array([PITCH,0,9*LAYER])
    hold=BASE+R0@np.array([.02,0,TIP+.001])
    left=[Waypoint(2.,tuple(hold+[0,0,.04]),FINGER_OPEN,'above full-size folded packet',YAW),
           Waypoint(2.,tuple(hold),FINGER_OPEN,'descend around packet',YAW),
           Waypoint(1.,tuple(hold),FINGER_CLOSED,'grasp actual packet sides',YAW)]
    theta=0.;grasp=pivot+R0@np.array([.10*math.cos(theta),.10*math.sin(theta),TIP-.012])
    right=[Waypoint(2.,tuple(grasp+[0,0,.06]),FINGER_OPEN,'above open terminal leaf',YAW+theta),
          Waypoint(2.,tuple(grasp),FINGER_OPEN,'descend around16mm leaf',YAW+theta),
          Waypoint(1.,tuple(grasp),FINGER_CLOSED,'grasp real leaf',YAW+theta)]
    for k in range(1,32):
        a=theta+(math.pi-theta)*k/31;p=pivot+R0@np.array([.10*math.cos(a),.10*math.sin(a),TIP-.012])
        # Finger pad axis follows the leaf; angle computed from fixed plan only.
        right.append(Waypoint(.25,tuple(p),FINGER_CLOSED,'follow passive hinge arc',YAW+a))
        left.append(Waypoint(.25,tuple(hold),FINGER_CLOSED,'support packet against real hinge torque',YAW))
    end=pivot+R0@np.array([-.10,0,TIP-.012])
    right.extend([Waypoint(1.,tuple(end),FINGER_OPEN,'release terminal leaf',YAW+math.pi),
                 Waypoint(2.,tuple(end+[0,-.10,.08]),FINGER_OPEN,'withdraw clear of folded packet')])
    left.extend([Waypoint(1.,tuple(hold),FINGER_CLOSED,'hold packet',YAW),
                  Waypoint(2.,tuple(hold),FINGER_CLOSED,'hold while other hand leaves',YAW)])
    target=np.array([POCKET[0]-.08,POCKET[1],hold[2]+FLOOR-TOP])
    # The real fingers cannot open between36mm high pocket walls. Place the
    # package above the50mm rim and release it for a short gravity-supported
    # descent; no object attachment/pose update is used.
    drop=target+[0,0,.055]
    left.extend([Waypoint(2.,tuple(hold+[0,0,.065]),FINGER_CLOSED,'lift folded whole tool',YAW),
                  Waypoint(3.,tuple(drop+[0,0,.010]),FINGER_CLOSED,'transfer above actual recess'),
                  Waypoint(2.,tuple(drop),FINGER_CLOSED,'lower package above pocket rim'),
                  Waypoint(1.,tuple(drop),FINGER_OPEN,'release all10leaves above rim'),
                  Waypoint(2.,tuple(drop+[0,.10,.020]),FINGER_OPEN,'clear all robot collision surfaces'),
                  Waypoint(3.,(.18,.36,TOP+.16),FINGER_OPEN,'observe released stability')])
    duration=sum(w.duration for w in left)-sum(w.duration for w in right)
    right.append(Waypoint(duration,(.18,-.36,TOP+.16),FINGER_OPEN,'observe released stability'))
    return {'left':left,'right':right}
def demo():return _reference()
