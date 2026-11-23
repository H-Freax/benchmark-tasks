"""Real yam_arena CPU checks; scorer inputs and gripper observations are synthetic.

TASK_MUTATION substitutes task-owned source only. No framework patch is used.
"""
from __future__ import annotations
import copy
from dataclasses import dataclass
import inspect
import json
import math
import os
from pathlib import Path
import types
import xml.etree.ElementTree as ET

import fcl
import numpy as np
import pytest
from scipy.spatial import ConvexHull
from yam_arena import ASSET_DIR, YAM_URDF, ik
from yam_arena.analysis import read_stl
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, FINGER_CLOSED, READY
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import Box, Cylinder, ObjectState, load
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix, rotation_angle_deg, rpy_matrix

ROOT=Path(__file__).resolve().parent
SOURCE=(ROOT/'task.py').read_text()
MUTATIONS={
 'sum_tiers':('return .75','return .25+.5+.75'),
 'yaw_is_tilt':('q[0]*q[0]+q[1]*q[1]','q[0]*q[0]+q[1]*q[1]+q[2]*q[2]'),
 'score_mutates':('def score(objects):\n    return float(_view(objects))','def score(objects):\n    _history.append(np.asarray(objects["soup_can"].pos).copy())\n    return float(_view(objects))'),
 'slow_reset':('reset = jumped and _nominal(objects)','reset = _nominal(objects)'),
 'forget_off_table':('if _off_table(s):\n            _off_table_latch = True','if False:\n            _off_table_latch = True'),
 'latch_tilt':('if _off_table(s):\n            _off_table_latch = True','if _off_table(s) or _tilt(s) >= MAX_TILT_DEG:\n            _off_table_latch = True'),
 'ignore_release':("if _hands is None or len(_hands) != 2:\n        return False","if _hands is None or len(_hands) != 2:\n        return True"),
 'ignore_hand_distance':('gap < OPEN_GAP or np.linalg.norm(np.maximum(separation,0.0)) < HAND_CLEARANCE','gap < OPEN_GAP'),
 'velocity_only':('return bool(np.max(np.linalg.norm(p[:,None,:]-p[None,:,:],axis=-1)) <= STILL_TOL)','return s.resting(max_speed=float(.01), max_spin=float(.2))'),
 'ignore_crossing':('or not _lifted or not _outside_drawer(s)','or not _outside_drawer(s)'),
 'no_open_release':("w(.8,(gx,gy,release),FINGER_OPEN,'release')","w(.8,(gx,gy,release),FINGER_CLOSED,'release')"),
 'high_release':('RELEASE_GAP = .0015','RELEASE_GAP = .004'),
 'low_approach':("w(1.5,(gx,gy,travel),FINGER_OPEN,'approach outside')","w(1.5,(gx,gy,float(TABLE_Z+.100)),FINGER_OPEN,'approach outside')"),
 'no_pause':("w(.4,(x,y,travel),LAYOUT['entry_finger_joint_m'],'align before descent')","w(.0,(x,y,travel),LAYOUT['entry_finger_joint_m'],'align before descent')"),
 'no_critical_debug':('_step==1 or _step%20==0 or critical','_step==1 or _step%20==0'),
 'numpy_numbers':('return tuple(float(v) for v in values)','return tuple(np.float64(v) for v in values)'),
}


def build():
    source=SOURCE
    mutation=os.environ.get('TASK_MUTATION')
    if mutation:
        old,new=MUTATIONS[mutation]
        assert old in source,mutation
        source=source.replace(old,new)
    module=types.ModuleType('drawer_task_test');module.__file__=str(ROOT/'task.py')
    exec(compile(source,str(ROOT/'task.py'),'exec'),module.__dict__)
    return module


def safe_hands():
    return [dict(low=np.array([.50,-.50,1.10]), high=np.array([.60,-.40,1.20]),gap=.0939),
            dict(low=np.array([.50,+.40,1.10]), high=np.array([.60,+.50,1.20]),gap=.0939)]


@pytest.fixture
def task():
    t=build()
    # Substitute only this task's observation adapter for deterministic CPU inputs.
    t.read_grippers=lambda:(safe_hands(),'synthetic CPU fixture')
    t.TASK_DEBUG=False
    return t


def state(pos,size,quat=(0,0,0,1),velocity=(0,0,0),spin=(0,0,0)):
    p=np.asarray(pos,dtype=float);q=np.asarray(quat,dtype=float)
    half=np.abs(quat_xyzw_to_matrix(q))@(np.asarray(size)/2)
    return ObjectState(p,q,np.asarray(velocity,dtype=float),np.asarray(spin,dtype=float),p-half,p+half)


def objects(t,can=None):
    out={b.name:state(b.pos,b.size,(0,0,math.sin(b.yaw/2),math.cos(b.yaw/2))) for b in t.FIXED.values()}
    out['soup_can']=can if can is not None else state(t.START_POS,t.CAN_SIZE)
    return out


def replace(t,o,pos,**kw):
    o['soup_can']=state(pos,t.CAN_SIZE,**kw)


def walk(t,o,pos,n=20):
    start=o['soup_can'].pos.copy()
    for u in np.linspace(0,1,n+1)[1:]:
        replace(t,o,start+(np.asarray(pos)-start)*u);t.success(o)
    return o


def lifted(t):
    o=objects(t);t.success(o)
    walk(t,o,(*t.START_XY,t.TABLE_Z+.160+t.CAN_HEIGHT/2))
    return o


def at_goal(t):
    o=lifted(t);walk(t,o,(*t.GOAL_XY,o['soup_can'].pos[2]));walk(t,o,t.GOAL_POS)
    for _ in range(10):t.success(o)
    return o


def snapshot(t):
    return (t._step,t._armed,t._off_table_latch,t._lifted,t._previous_bottom,
            tuple(tuple(p) for p in t._history),None if t._previous is None else tuple(t._previous),
            repr(t._hands),t._hand_reason)


def numeric_floats(value):
    if isinstance(value,(int,float,np.number)) and not isinstance(value,bool):assert type(value) is float,(value,type(value))
    elif isinstance(value,(list,tuple)):
        for v in value:numeric_floats(v)


def test_real_framework_contract_and_constructor_types(task):
    loaded=load(ROOT/'task.py')
    assert loaded.scene=='station' and loaded.free_objects==('soup_can',)
    assert loaded.score_over=='end' and loaded.success_hold_s==1.0
    for spec in task.OBJECTS:
        inspect.signature(type(spec)).bind(**vars(spec))
        for v in vars(spec).values():numeric_floats(v)
    for ws in task.demo().values():
        for w in ws:
            inspect.signature(Waypoint).bind(**vars(w))
            for v in vars(w).values():numeric_floats(v)


def test_tiers_and_external_success_hold(task):
    o=objects(task);task.success(o);assert task.score(o)==0
    o=lifted(task);assert task.score(o)==.25
    walk(task,o,(*task.GOAL_XY,o['soup_can'].pos[2]));walk(task,o,task.GOAL_POS)
    assert task.score(o)==.5
    for _ in range(10):task.success(o)
    assert task.score(o)==.75
    assert all(task.success(o) for _ in range(20))
    assert task.score(o)==.75


def test_terminal_spawn_and_direct_jump(task):
    o=objects(task,state(task.GOAL_POS,task.CAN_SIZE))
    for _ in range(25):assert not task.success(o)
    assert not task._armed and task.score(o)==0
    t=build();t.read_grippers=lambda:(safe_hands(),'synthetic')
    o=objects(t);t.success(o);replace(t,o,t.GOAL_POS)
    for _ in range(25):assert not t.success(o)
    assert not t._lifted and t.score(o)==0


def test_jump_to_lift_cannot_fake_upward_crossing(task):
    o=objects(task);task.success(o)
    replace(task,o,(*task.START_XY,task.TABLE_Z+.160+task.CAN_HEIGHT/2));task.success(o)
    for _ in range(10):task.success(o)
    assert not task._lifted and task.score(o)==0


def test_off_table_latch_survives_slow_restoration(task):
    o=at_goal(task)
    walk(task,o,(*task.GOAL_XY,task.GOAL_POS[2]-.020))
    assert task._off_table_latch and task.score(o)==0
    walk(task,o,task.GOAL_POS)
    for _ in range(20):task.success(o)
    assert task.score(o)==0
    walk(task,o,task.START_POS,n=60)
    assert task._off_table_latch


def test_can_launched_beyond_table_stays_failed(task):
    o=at_goal(task)
    p=task.GOAL_POS.copy()
    for x in np.arange(p[0],1.05,.04):
        p[0]=x;replace(task,o,p,velocity=(2,0,0));task.success(o)
    assert task._off_table_latch and task.score(o)==0
    walk(task,o,task.GOAL_POS,n=60)
    assert task.score(o)==0


def test_tilt_is_repairable_and_yaw_180_is_upright(task):
    o=at_goal(task)
    replace(task,o,task.GOAL_POS,quat=(math.sqrt(.5),0,0,math.sqrt(.5)));task.success(o)
    assert task.score(o)==0 and not task._off_table_latch
    replace(task,o,task.GOAL_POS,quat=(0,0,1,0))
    for _ in range(10):task.success(o)
    assert task.score(o)==.75


def test_topple_in_row_can_be_repaired(task):
    o=objects(task);task.success(o)
    replace(task,o,task.START_POS,quat=(math.sqrt(.5),0,0,math.sqrt(.5)));task.success(o)
    replace(task,o,task.START_POS);task.success(o)
    walk(task,o,(*task.START_XY,task.TABLE_Z+.160+task.CAN_HEIGHT/2))
    assert task._lifted and not task._off_table_latch and task.score(o)==.25


def test_real_reset_requires_jump_and_nominal_layout(task):
    o=at_goal(task)
    walk(task,o,(*task.GOAL_XY,task.GOAL_POS[2]-.020));assert task._off_table_latch
    replace(task,o,task.START_POS);task.success(o)
    assert not task._off_table_latch and task._armed and not task._lifted and task._step==1


def test_fixed_layout_and_invalid_readings_fail_closed_but_recover(task):
    o=at_goal(task);good=copy.deepcopy(o);s=o['divider_1']
    o['divider_1']=state(s.pos+[.002,0,0],task.FIXED['divider_1'].size,s.quat_xyzw);task.success(o)
    assert task.score(o)==0
    o=copy.deepcopy(good);task.success(o);assert task.score(o)==.75
    o['soup_can'].lin_vel[0]=np.nan;task.success(o);assert task.score(o)==0
    o=copy.deepcopy(good);task.success(o);assert task.score(o)==.75


@pytest.mark.parametrize('offset,armed',[(.004999,True),(.005001,False)])
def test_initial_position_boundary(task,offset,armed):
    p=task.START_POS.copy();p[0]+=offset;o=objects(task,state(p,task.CAN_SIZE));task.success(o)
    assert task._armed==armed


@pytest.mark.parametrize('angle,allowed',[(4.999,True),(5.001,False),(90,False),(180,False)])
def test_tilt_boundary(task,angle,allowed):
    o=at_goal(task);a=math.radians(angle)/2;replace(task,o,task.GOAL_POS,quat=(math.sin(a),0,0,math.cos(a)));task.success(o)
    assert (task.score(o)>.0)==allowed


@pytest.mark.parametrize('offset,full',[(.014999,True),(.015001,False),(-.014999,True),(-.015001,False)])
def test_target_boundary(task,offset,full):
    o=at_goal(task);p=task.GOAL_POS.copy();p[0]+=offset;walk(task,o,p)
    for _ in range(10):task.success(o)
    assert task.score(o)==(.75 if full else .5)


@pytest.mark.parametrize('height,supported',[(.001999,True),(.002001,False),(-.000999,True),(-.001001,False)])
def test_support_boundary(task,height,supported):
    o=at_goal(task);p=task.GOAL_POS.copy();p[2]+=height;walk(task,o,p)
    for _ in range(10):task.success(o)
    assert task.score(o)==(.75 if supported else .25)


@pytest.mark.parametrize('delta,still',[(.001999,True),(.002001,False)])
def test_still_window_boundary(task,delta,still):
    o=at_goal(task);p=task.GOAL_POS.copy();p[0]+=delta;replace(task,o,p);task.success(o)
    assert task.score(o)==(.75 if still else .5)


def test_frozen_velocity_and_noisy_velocity(task):
    o=at_goal(task);p=task.GOAL_POS.copy();p[0]+=.006;replace(task,o,p);task.success(o)
    assert task.score(o)==.5
    replace(task,o,task.GOAL_POS,velocity=(.03,0,.08),spin=(0,0,.8))
    for _ in range(10):task.success(o)
    assert task.score(o)==.75


def test_unfilled_window_speed_spin_fallback(task):
    o=objects(task);task.success(o)
    assert task._still(o['soup_can'])
    assert not task._still(state(task.START_POS,task.CAN_SIZE,velocity=(.02,0,0)))
    assert not task._still(state(task.START_POS,task.CAN_SIZE,spin=(0,0,.3)))


def test_holding_and_missing_hand_readings_do_not_get_release_credit(task):
    o=at_goal(task);assert task.score(o)==.75
    task.read_grippers=lambda:(None,'unavailable');task.success(o);assert task.score(o)==.25
    closed=safe_hands();closed[0]['gap']=.067
    task.read_grippers=lambda:(closed,'synthetic closed');task.success(o);assert task.score(o)==.25


@pytest.mark.parametrize('clearance,release',[(.029999,False),(.030001,True)])
def test_complete_hand_distance_boundary(task,clearance,release):
    o=at_goal(task);s=o['soup_can'];hands=safe_hands()
    hands[0]=dict(low=s.low+np.array([0,0,task.CAN_HEIGHT+clearance]),high=s.high+np.array([0,0,task.CAN_HEIGHT+clearance]),gap=.0939)
    task.read_grippers=lambda:(hands,'synthetic close hand');task.success(o)
    assert task.score(o)==(.75 if release else .25)


@pytest.mark.parametrize('gap,release',[(.084999,False),(.085001,True)])
def test_opening_boundary(task,gap,release):
    o=at_goal(task);hands=safe_hands();hands[0]['gap']=gap
    task.read_grippers=lambda:(hands,'synthetic opening');task.success(o)
    assert task.score(o)==(.75 if release else .25)


def test_public_reader_unavailable_is_closed():
    t=build();observations,reason=t.read_grippers()
    assert observations is None and reason.startswith('gripper unavailable:')


def test_score_distance_and_final_frame_are_read_only(task):
    o=at_goal(task)
    for dx in np.linspace(0,.001,5):
        p=task.GOAL_POS.copy();p[0]+=dx;replace(task,o,p);task.success(o)
    before=snapshot(task);inputs=copy.deepcopy(o)
    for _ in range(12):task.score(o);task.distance(o)
    assert snapshot(task)==before
    for name in o:
        for k in vars(o[name]):assert np.array_equal(getattr(o[name],k),getattr(inputs[name],k))
    replace(task,o,task.GOAL_POS,quat=(math.sqrt(.5),0,0,math.sqrt(.5)))
    assert task.score(o)==0 and snapshot(task)==before


def test_debug_cadence_and_critical_every_step(task,capsys):
    task.TASK_DEBUG=True;o=objects(task)
    for _ in range(41):task.success(o)
    lines=capsys.readouterr().out.splitlines()
    assert [json.loads(s.split(' ',1)[1])['step'] for s in lines]==[1,20,40]
    replace(task,o,task.START_POS+np.array([0,0,.010]))
    for _ in range(5):task.success(o)
    lines=capsys.readouterr().out.splitlines()
    assert [json.loads(s.split(' ',1)[1])['step'] for s in lines]==[42,43,44,45,46]
    task.TASK_DEBUG=False
    for _ in range(21):task.success(o)
    assert capsys.readouterr().out==''


def test_release_pause_run_limit_and_own_side(task):
    ws=task.demo()['left'];release=next(w for w in ws if w.label=='release');lower=next(w for w in ws if w.label=='lower')
    assert release.pos==lower.pos and release.finger==FINGER_OPEN
    assert release.pos[2]-task.TABLE_Z-task.PICK_HEIGHT<=.002
    assert release.pos[2]-task.TABLE_Z-task.PICK_HEIGHT>=0.0
    for i,w in enumerate(ws):
        if w.label in ('descend','lower'):
            assert ws[i-1].pos[:2]==w.pos[:2] and ws[i-1].duration>=.3 and ws[i-1].label.startswith('align')
        assert w.pos[1]>0 and w.yaw==task.YAW
    assert ws[-1].finger==FINGER_OPEN
    assert sum(w.duration for w in ws)*20+20<=task.RUN_LIMIT


LINK_NAMES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')


@dataclass
class Hull:
    link: str
    index: int
    vertices: np.ndarray
    obj: object
    source: str


def joint_map(q, finger):
    return dict(zip(ARM_JOINTS, q, strict=True)) | {'joint7': finger, 'joint8': finger}


def read_hulls():
    hulls = []
    for name in LINK_NAMES:
        link = ET.parse(YAM_URDF).getroot().find(f"link[@name='{name}']")
        for index, col in enumerate(link.findall('collision')):
            origin = col.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rotation = rpy_matrix(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
            mesh = col.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Unexpected non-mesh collision geometry')
            filename = mesh.get('filename')
            triangles = read_stl(ASSET_DIR / filename)
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            vertices = np.unique(triangles.reshape(-1, 3) * scale, axis=0) @ rotation.T + xyz
            ch = ConvexHull(vertices)
            faces = ch.simplices.copy()
            for i, face in enumerate(faces):
                tri = vertices[face]
                if np.dot(np.cross(tri[1]-tri[0], tri[2]-tri[0]), ch.equations[i, :3]) < 0:
                    faces[i] = face[::-1]
            packed = np.c_[np.full(len(faces), 3), faces].ravel().tolist()
            obj = fcl.CollisionObject(fcl.Convex(vertices, len(faces), packed))
            hulls.append(Hull(name, index, vertices, obj, filename))
    return hulls


@pytest.fixture(scope='module')
def geometry():
    return types.SimpleNamespace(LINK_NAMES=LINK_NAMES, joint_map=joint_map), read_hulls()


@pytest.fixture(scope='module')
def trace():
    t=build();ws=t.demo()['left'];plan=ArmPlan(ws,tuple(map(float,BASES['left'])));frames=[]
    for k in range(round(plan.duration*20)+1):
        tm=float(k/20);a,label=plan.action7(tm);target,rot,finger,_=plan.target(tm)
        p,r=ik.tcp_pose(a[:6],BASES['left'])
        frames.append((tm,label,a.copy(),float(finger),float(np.linalg.norm(p-target)),rotation_angle_deg(rot,r)))
    return t,plan,frames


def test_full_demo_20hz_ik_limits_and_plus10(trace):
    t,p,frames=trace
    assert len(frames)==289 and p.worst_ik_mm<.5 and p.worst_ik_deg<.1
    assert max(f[4] for f in frames)<.0005 and max(f[5] for f in frames)<.1
    actions=np.array([f[2][:6] for f in frames]);assert np.min(np.minimum(actions-ik.LOWER,ik.UPPER-actions))>math.radians(1)
    elapsed=0.0
    for w in p.waypoints:
        elapsed+=w.duration;seed=min(frames,key=lambda f:abs(f[0]-elapsed))[2][:6]
        q,pe,re=ik.solve(np.array(w.pos)+(0,0,.010),ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)),seed,base_pos=BASES['left'],iters=200)
        assert pe<.0005 and math.degrees(re)<.1
        assert np.min(np.minimum(q-ik.LOWER,ik.UPPER-q))>0


def test_real_collision_hulls_clear_all_fixed_structures(trace,geometry):
    t,plan,frames=trace;g,hulls=geometry
    requests=fcl.DistanceRequest(enable_nearest_points=True,gjk_solver_type=fcl.GJKSolverType.GST_LIBCCD)
    boxes=[]
    for b in t.FIXED.values():
        r=quat_xyzw_to_matrix(np.array([0,0,math.sin(b.yaw/2),math.cos(b.yaw/2)]))
        corners=np.array([[i,j,k] for i in (-1,1) for j in (-1,1) for k in (-1,1)])*(np.asarray(b.size)/2)
        vs=corners@r.T+np.asarray(b.pos)
        boxes.append((b.name,vs.min(0),vs.max(0),fcl.CollisionObject(fcl.Box(*b.size),fcl.Transform(r,np.array(b.pos)))))
    static=forward_kinematics(YAM_URDF,g.joint_map(READY,FINGER_OPEN))
    samples=[(0.0,'right',static)]
    for tm,label,a,finger,_,_ in frames:
        physical=min(finger,t.LAYOUT['contact_finger_joint_m']) if label in ('close','extract vertically','carry','align before placement','lower','release') else finger
        samples.append((tm,'left',forward_kinematics(YAM_URDF,g.joint_map(a[:6],physical))))
    minimum=.02;limiting=None
    bundles={}
    for name in g.LINK_NAMES:
        hs=[h for h in hulls if h.link==name];starts=np.r_[0,np.cumsum([len(h.vertices) for h in hs])[:-1]]
        bundles[name]=(np.concatenate([h.vertices for h in hs]),starts,hs)
    for tm,side,poses in samples:
        for name,(vertices,starts,hs) in bundles.items():
            mat=poses[name];r=mat[:3,:3];p=mat[:3,3]+BASES[side];vs=vertices@r.T+p
            lo=np.minimum.reduceat(vs,starts,axis=0);hi=np.maximum.reduceat(vs,starts,axis=0)
            for bn,blo,bhi,box in boxes:
                lower=np.linalg.norm(np.maximum(np.maximum(blo-hi,lo-bhi),0),axis=1)
                for i in np.flatnonzero(lower<=max(minimum,.003)):
                    hs[i].obj.setTransform(fcl.Transform(r,p));gap=max(0.0,float(fcl.distance(hs[i].obj,box,requests,fcl.DistanceResult())))
                    if gap<minimum:minimum=gap;limiting=(tm,side,name,bn)
    assert minimum>=.003,(minimum,limiting)


def test_carried_can_and_spawn_clearances(task):
    # Conservative OBB SAT axes certify nonoverlap; robot clearance is separately GJK checked.
    o=objects(task);specs=list(task.FIXED.values())
    can=o['soup_can'];assert can.low[2]-task.TABLE_Z==pytest.approx(.001)
    axes=[np.eye(3)[i] for i in range(3)]+[task.ROTATION[:,i] for i in range(3)]
    for i,a in enumerate(specs):
        for b in specs[i+1:]:
            ra=quat_xyzw_to_matrix(np.array([0,0,math.sin(a.yaw/2),math.cos(a.yaw/2)]));rb=quat_xyzw_to_matrix(np.array([0,0,math.sin(b.yaw/2),math.cos(b.yaw/2)]))
            assert any(abs(np.dot(np.asarray(a.pos)-b.pos,axis))-np.abs(axis@ra)@np.asarray(a.size)/2-np.abs(axis@rb)@np.asarray(b.size)/2>=-1e-10 for axis in axes),(a.name,b.name)
    for b in specs:
        rb=quat_xyzw_to_matrix(np.array([0,0,math.sin(b.yaw/2),math.cos(b.yaw/2)]))
        assert any(abs(np.dot(can.pos-b.pos,axis))-np.abs(axis)@task.CAN_SIZE/2-np.abs(axis@rb)@np.asarray(b.size)/2>=-1e-10 for axis in axes),b.name
    assert task.PICK_HEIGHT-float(.012842499535)>=.003
    # During transfer the whole carried can clears the taller outer frame.
    assert task.TRAVEL_HEIGHT-task.PICK_HEIGHT-task.LAYOUT['frame_height_m']>=.003
    release=next(w for w in task.demo()['left'] if w.label=='release')
    assert release.pos[2]-task.TABLE_Z-task.PICK_HEIGHT==pytest.approx(.0015)


def test_native_asset_metadata_and_text(task):
    assert max(task.CAN_SIZE[:2])<.085 and task.CAN['grasp_band_height_m']>=.015
    assert task.CAN['scale']==1.0 and task.OBJECTS[0].radius*2==max(task.CAN_SIZE[:2])
    assert task.CAN_HEIGHT==pytest.approx(.10186)
    assert task.LAYOUT['divider_height_m']==.127 and task.LAYOUT['divider_thickness_m']==.015875
    for path in (ROOT/'assets').rglob('*'):
        if path.is_file():
            assert path.suffix in ('.json', '.usda')
            for token in [bytes([47])+b'Users'+bytes([47]), bytes([47])+b'home'+bytes([47]),
                          bytes([67,58,92]), b'Exif', b'caBX', b'xmpmeta']:
                assert token not in path.read_bytes()
    meta=json.loads((ROOT/'task.json').read_text())
    assert set(['title','category','keywords','instruction','context','setup','scoring','run_limit','fidelity','scores','media','reset_notes'])<=set(meta)
    assert meta['scores']=='pending' and meta['media']=={}
    assert meta['title']=='Lift a can out of a drawer divider row'
    assert task.RUN_LIMIT==800 and meta['run_limit']=='800 control steps at 20 Hz (40 seconds)'
    for text in [meta['context'],(ROOT/'README.md').read_text()]:
        for token in ['#182','#1957','#2376','strong_reject','review rules']:assert token not in text
    assert meta['context'].startswith('Deep kitchen drawers often hold cans upright')


def test_debug_every_step_during_hand_descent(task,capsys):
    task.TASK_DEBUG=True;o=objects(task);s=o['soup_can'];hands=safe_hands()
    hands[0]=dict(low=s.low+np.array([0,0,.1]),high=s.high+np.array([0,0,.1]),gap=.0939)
    task.read_grippers=lambda:(hands,'synthetic descent')
    for _ in range(4):task.success(o)
    assert [json.loads(line.split(' ',1)[1])['step'] for line in capsys.readouterr().out.splitlines()]==[1,2,3,4]


@pytest.mark.parametrize('offset,resets',[(.099999,False),(.100001,True)])
def test_reset_jump_boundary(task,offset,resets):
    o=at_goal(task);walk(task,o,(*task.GOAL_XY,task.GOAL_POS[2]-.020))
    p=task.START_POS.copy();p[0]+=offset;walk(task,o,p,n=60)
    assert task._off_table_latch
    replace(task,o,task.START_POS);task.success(o)
    assert (not task._off_table_latch)==resets


@pytest.mark.parametrize('depth,latches',[(.009999,False),(.010001,True)])
def test_below_table_boundary(task,depth,latches):
    o=at_goal(task);p=task.GOAL_POS.copy();p[2]-=depth;replace(task,o,p);task.success(o)
    assert task._off_table_latch==latches


@pytest.mark.parametrize('clearance,extracted',[(.002999,False),(.003001,True)])
def test_lift_crossing_boundary(task,clearance,extracted):
    o=objects(task);task.success(o)
    walk(task,o,(*task.START_XY,task.TABLE_Z+task.LAYOUT['frame_height_m']+clearance+task.CAN_HEIGHT/2))
    assert task._lifted==extracted


@pytest.mark.parametrize('offset,allowed',[(.000499,True),(.000501,False)])
def test_fixed_board_translation_boundary(task,offset,allowed):
    o=at_goal(task);s=o['divider_1'];b=task.FIXED['divider_1']
    o['divider_1']=state(s.pos+[offset,0,0],b.size,s.quat_xyzw);task.success(o)
    assert (task.score(o)==.75)==allowed


def test_cylinder_fallback_dimensions_and_representation(task):
    can=task.OBJECTS[0]
    assert isinstance(can,Cylinder)
    assert can.radius==.03383 and can.height==.10186
    assert can.mass==.44999998807907104
    assert tuple(can.pos)==pytest.approx(tuple(task.START_POS))
    assert can.color[0]>can.color[1] and can.color[0]>can.color[2]
    assert task.CAN['max_horizontal_feret_m']==pytest.approx(.06766)
    assert task.LAYOUT['entry_finger_joint_m']==FINGER_OPEN
    assert not list((ROOT/'assets').glob('*.usda'))
    for text in [(ROOT/'README.md').read_text(),json.loads((ROOT/'task.json').read_text())['setup']]:
        assert 'standard soup can' in text and '67.7' in text and '101.9' in text
        assert 'horizontal' in text and 'yaw' in text
        assert 'YCB' not in text and 'CC BY 4.0' not in text
