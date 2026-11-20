"""Real yam_arena CPU contract, scorer and complete 20 Hz demo verification."""
from __future__ import annotations

import ast
import copy
import inspect
import json
import math
import os
from pathlib import Path
import types
from itertools import product

import numpy as np
import pytest
from yam_arena import YAM_URDF, ik
from yam_arena.analysis import link_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY, TCP_OFFSET
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import Asset, Box, ObjectState, load
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

ROOT = Path(__file__).resolve().parent
SOURCE = (ROOT/'task.py').read_text()
MUTATIONS = {
    'forget_tilt_latch':("if _tilt(s)>=MAX_TILT_DEG:\n            _violation=_violation or 'tipped'", "if False:\n            _violation=_violation or 'tipped'"),
    'sum_tiers':('return .75','return .25+.5+.75'),
    'yaw_is_tilt':('q[0]*q[0]+q[1]*q[1]','q[0]*q[0]+q[1]*q[1]+q[2]*q[2]'),
    'score_mutates':('def score(objects):\n    return _view(objects)','def score(objects):\n    _history.append(np.asarray(objects["soup_can"].pos).copy())\n    return _view(objects)'),
    'debug_every_step':('(_step==1 or _step%20==0)','True'),
    'slow_reset':('reset=jumped and _nominal(objects)','reset=_nominal(objects)'),
    'unreachable_hover':("travel=TABLE_Z+WALL_HEIGHT+P['tcp_pick_above_table_m']+P['release_gap_m']", "travel=TABLE_Z+WALL_HEIGHT+P['tcp_pick_above_table_m']+P['release_gap_m']+.4"),
    'panel_collision':("GAP_WIDTH = CAN['transverse_finger_width_m'] + P['gap_extra_m']", "GAP_WIDTH = .020"),
    'velocity_only_rest':("return bool(max(np.max(np.linalg.norm(p[:,None,:]-p[None,:,:],axis=-1)),\n                    np.max(np.linalg.norm(p-np.asarray(s.pos),axis=-1)))<=STILL_TOL)", "return s.resting(max_speed=.01,max_spin=.2)"),
    'no_open_release':("Waypoint(.8,(gx,gy,place),FINGER_OPEN,'release',yaw)","Waypoint(.8,(gx,gy,place),FINGER_CLOSED,'release',yaw)"),
}


def _build_task():
    source=SOURCE
    mutant=os.environ.get('TASK_MUTATION')
    if mutant:
        old,new=MUTATIONS[mutant]
        assert old in source
        source=source.replace(old,new)
    module=types.ModuleType('can_gap_test_module');module.__file__=str(ROOT/'task.py')
    exec(compile(source,str(ROOT/'task.py'),'exec'),module.__dict__)
    return module


@pytest.fixture
def task():
    return _build_task()


def state(pos,size,quat=(0,0,0,1),velocity=(0,0,0),spin=(0,0,0)):
    p=np.asarray(pos,dtype=float);q=np.asarray(quat,dtype=float)
    half=np.abs(quat_xyzw_to_matrix(q))@(np.asarray(size)/2)
    return ObjectState(p,q,np.asarray(velocity,dtype=float),np.asarray(spin,dtype=float),p-half,p+half)


def objects(task,can=None):
    out={s.name:state(s.pos,s.size) for s in task.OBJECTS if isinstance(s,Box)}
    out['soup_can']=can or state(task.START_POS,task.CAN_SIZE)
    return out


def replace_can(task,o,pos,**kw):
    o['soup_can']=state(pos,task.CAN_SIZE,**kw)
    return o


def walk(task,o,pos,steps=24):
    start=o['soup_can'].pos.copy()
    for fraction in np.linspace(0,1,steps+1)[1:]:
        replace_can(task,o,start+(np.asarray(pos)-start)*fraction)
        task.success(o)
    return o


def at_goal(task,settle=10):
    o=objects(task);task.success(o)
    high=task.TABLE_Z+task.WALL_HEIGHT+task.CAN_HEIGHT/2+.005
    walk(task,o,(*task.START_XY,high))
    walk(task,o,(*task.GOAL_XY,high))
    walk(task,o,task.GOAL_POS)
    for _ in range(settle):task.success(o)
    return o


def snapshot(task):
    return (task._step,task._armed,task._violation,
            tuple(tuple(p) for p in task._history),None if task._previous is None else tuple(task._previous))


def test_real_framework_contract_and_every_constructor_signature(task):
    loaded=load(ROOT/'task.py')
    assert loaded.scene=='station' and loaded.free_objects==('soup_can',)
    assert loaded.score_over=='end' and loaded.success_hold_s==1.
    for spec in task.OBJECTS:
        inspect.signature(type(spec)).bind(**vars(spec));spec._check()
    for ws in task.demo().values():
        for w in ws:inspect.signature(type(w)).bind(**vars(w))


def test_no_isaac_import_or_private_robot_state(task):
    tree=ast.parse(SOURCE)
    imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert not any(x.startswith(('isaac','omni')) for x in imports)
    assert 'monkeypatch' not in SOURCE and 'body_link_pose' not in SOURCE
    assert set(task.demo())=={'right'}


def test_each_tier_and_framework_hold(task):
    o=objects(task);assert not task.success(o) and task.score(o)==0.
    lift=task.TABLE_Z+task.WALL_HEIGHT+task.CAN_HEIGHT/2+.002
    walk(task,o,(*task.START_XY,lift));assert task.score(o)==.25
    off=task.GOAL_POS.copy();off[0]+=.03
    walk(task,o,(*off[:2],lift));walk(task,o,off)
    for _ in range(10):task.success(o)
    assert task.score(o)==.5
    walk(task,o,task.GOAL_POS)
    for _ in range(10):task.success(o)
    assert task.score(o)==.75
    held=0
    for _ in range(20):
        held=held+1 if task.success(o) else 0
        assert task.score(o)==.75
    assert held>=round(task.SUCCESS_HOLD_S*20)


def test_starting_at_final_state_fails_closed(task):
    o=objects(task,state(task.GOAL_POS,task.CAN_SIZE))
    for _ in range(30):assert not task.success(o)
    assert task.score(o)==0 and task._violation=='initial_layout'


def test_jump_straight_from_start_to_finish_is_not_progress(task):
    o=objects(task);task.success(o)
    replace_can(task,o,task.GOAL_POS);task.success(o)
    assert task.score(o)==0 and task._violation=='discontinuous_motion'


@pytest.mark.parametrize('violation',['tipped','off_table','fixed_layout','invalid_state','discontinuous_motion'])
def test_every_irreversible_violation_survives_later_restoration(task,violation):
    o=at_goal(task);good=copy.deepcopy(o)
    if violation=='tipped':
        o['soup_can']=state(o['soup_can'].pos,task.CAN_SIZE,quat=(math.sin(math.pi/4),0,0,math.cos(math.pi/4)))
    elif violation=='off_table':
        # Below-table violation is small enough to avoid resetting/jump classification.
        p=o['soup_can'].pos.copy();p[2]-=.02;replace_can(task,o,p)
    elif violation=='fixed_layout':
        s=o['cabinet_side'];o['cabinet_side']=state(s.pos+[.002,0,0],task.FIXED['cabinet_side'].size)
    elif violation=='invalid_state':
        o['soup_can'].lin_vel[0]=np.nan
    else:
        replace_can(task,o,o['soup_can'].pos+[.12,0,0])
    task.success(o);assert task.score(o)==0
    # Restore into the goal, never the nominal start, so no reset can be claimed.
    o=good
    for _ in range(30):task.success(o)
    assert task.score(o)==0 and task._violation is not None


def test_topple_inside_gap_and_slowly_restore_cannot_clear_latch(task):
    o=objects(task);task.success(o)
    s=o['soup_can'];o['soup_can']=state(s.pos,task.CAN_SIZE,quat=(.70710678,0,0,.70710678))
    task.success(o);replace_can(task,o,task.START_POS)
    for _ in range(30):task.success(o)
    assert task._violation=='tipped' and task.score(o)==0


def test_true_reset_requires_jump_and_nominal_layout(task):
    o=at_goal(task);replace_can(task,o,task.GOAL_POS,quat=(.70710678,0,0,.70710678));task.success(o)
    replace_can(task,o,task.START_POS);assert not task.success(o)
    assert task._violation is None and task._armed and task._step==1
    assert task.score(o)==0


def test_jump_to_near_start_with_wrong_fixed_layout_does_not_reset(task):
    o=at_goal(task);s=o['wall_panel'];o['wall_panel']=state(s.pos+[.003,0,0],task.FIXED['wall_panel'].size)
    replace_can(task,o,task.START_POS);task.success(o)
    assert task._violation is not None and task.score(o)==0


def test_final_frame_conditions_are_rechecked_without_advancing_history(task):
    o=at_goal(task);before=snapshot(task)
    replace_can(task,o,task.GOAL_POS,quat=(.70710678,0,0,.70710678))
    assert task.score(o)==0
    assert snapshot(task)==before


def test_yaw_180_is_not_tilt(task):
    o=at_goal(task);replace_can(task,o,task.GOAL_POS,quat=(0,0,1,0))
    task.success(o);assert task.score(o)==.75 and task._violation is None


@pytest.mark.parametrize('angle,allowed',[(4.999,True),(5.001,False),(90,False),(180,False)])
def test_tilt_threshold_both_sides(task,angle,allowed):
    o=at_goal(task);a=math.radians(angle)/2
    replace_can(task,o,task.GOAL_POS,quat=(math.sin(a),0,0,math.cos(a)))
    task.success(o);assert (task.score(o)>0)==allowed


@pytest.mark.parametrize('offset,full',[(.014999,True),(.015001,False),(-.014999,True),(-.015001,False)])
def test_target_threshold_both_sides(task,offset,full):
    o=at_goal(task);p=task.GOAL_POS.copy();p[0]+=offset;walk(task,o,p)
    for _ in range(10):task.success(o)
    assert task.score(o)==(.75 if full else .5)


@pytest.mark.parametrize('height,supported',[(.002999,True),(.003001,False)])
def test_support_threshold_both_sides(task,height,supported):
    o=at_goal(task);p=task.GOAL_POS.copy();p[2]+=height;walk(task,o,p)
    for _ in range(10):task.success(o)
    assert task.score(o)==(.75 if supported else .25)


@pytest.mark.parametrize('delta,still',[(.001999,True),(.002001,False)])
def test_position_window_threshold_both_sides(task,delta,still):
    o=at_goal(task)
    p=task.GOAL_POS.copy();p[0]+=delta;replace_can(task,o,p);task.success(o)
    assert task.score(o)==(.75 if still else .5)


def test_frozen_zero_velocity_cannot_hide_motion(task):
    o=at_goal(task)
    p=task.GOAL_POS.copy();p[0]+=.006;replace_can(task,o,p);task.success(o)
    assert task.score(o)==.5


def test_position_window_handles_frozen_noisy_velocity(task):
    o=at_goal(task)
    replace_can(task,o,task.GOAL_POS,velocity=(.02,0,.08),spin=(0,0,.8))
    for _ in range(10):task.success(o)
    assert task.score(o)==.75


def test_unfilled_window_uses_explicit_speed_and_spin(task):
    o=objects(task);task.success(o)
    assert len(task._history)<10
    s=state(task.START_POS,task.CAN_SIZE,velocity=(.02,0,0))
    assert not task._still(s)
    assert task._still(state(task.START_POS,task.CAN_SIZE))


def test_score_and_distance_are_read_only(task):
    o=at_goal(task)
    for dx in np.linspace(0,.001,5):
        p=task.GOAL_POS.copy();p[0]+=dx;replace_can(task,o,p);task.success(o)
    before=snapshot(task);inputs=copy.deepcopy(o)
    for _ in range(20):task.score(o);task.distance(o)
    assert snapshot(task)==before
    for name in o:
        for attr in vars(o[name]):assert np.array_equal(getattr(o[name],attr),getattr(inputs[name],attr))


def test_holding_without_release_is_documented_simulation_limit(task):
    o=at_goal(task);held=copy.deepcopy(o)
    assert task.score(o)==task.score(held)==.75
    meta=json.loads((ROOT/'task.json').read_text())
    assert 'the gripper is open and at least 30 mm from the can at the end, judged from the trial video' in meta['scoring']


def test_spawn_has_no_overlap(task):
    o=objects(task)
    assert o['soup_can'].low[2]-task.TABLE_Z==pytest.approx(.001)
    specs=list(task.OBJECTS)
    for i,a in enumerate(specs):
        for b in specs[i+1:]:
            overlap=np.minimum(o[a.name].high,o[b.name].high)-np.maximum(o[a.name].low,o[b.name].low)
            assert np.any(overlap<=1e-9),(a.name,b.name,overlap)


def test_debug_cadence_is_print_only(task,capsys):
    task.TASK_DEBUG=True;o=objects(task)
    for _ in range(41):task.success(o)
    lines=capsys.readouterr().out.splitlines()
    assert all(s.startswith('task_progress ') for s in lines)
    assert [json.loads(s.split(' ',1)[1])['step'] for s in lines]==[1,20,40]
    assert task.score(o)==0 and task._violation is None


def test_debug_disabled_does_not_print(task,capsys):
    task.TASK_DEBUG=False;o=objects(task)
    for _ in range(21):task.success(o)
    assert capsys.readouterr().out==''


def test_demo_opens_then_retreats_and_release_is_not_a_throw(task):
    ws=task.demo()['right'];release=next(w for w in ws if w.label=='release');lower=next(w for w in ws if w.label=='lower')
    assert release.pos==lower.pos and release.finger==FINGER_OPEN
    assert ws[-1].finger==FINGER_OPEN
    assert release.pos[2]-task.TABLE_Z-task.P['tcp_pick_above_table_m']==pytest.approx(.004)
    # Lowest fingertip, not only TCP, clears the standing can's highest point.
    assert ws[-1].pos[2]-.013-(task.TABLE_Z+task.CAN_HEIGHT)>=.030
    assert sum(w.duration for w in ws)+task.SUCCESS_HOLD_S+1<=task.RUN_LIMIT/20


@pytest.fixture(scope='module')
def real_demo_trace():
    task=_build_task();plan=ArmPlan(task.demo()['right'],BASES['right']);trace=[]
    for frame in range(round(plan.duration*20)+1):
        a,label=plan.action7(frame/20);_,_,finger,_=plan.target(frame/20)
        trace.append((frame/20,a.copy(),label,float(finger)))
    return task,plan,trace


def test_demo_calls_action7_every_20hz_frame_and_respects_limits(real_demo_trace):
    task,plan,trace=real_demo_trace
    assert len(trace)==207
    assert plan.worst_ik_mm<1 and plan.worst_ik_deg<1
    actions=np.array([a for _,a,_,_ in trace]);assert np.all(actions[:,:6]>=ik.LOWER) and np.all(actions[:,:6]<=ik.UPPER)
    assert np.min(np.minimum(actions[:,:6]-ik.LOWER,ik.UPPER-actions[:,:6]))>math.radians(1)
    # Independently recompute residuals so zero/unused ArmPlan counters cannot pass.
    for t,a,_,_ in trace:
        wanted,rot,_,_=plan.target(t);p,r=ik.tcp_pose(a[:6],BASES['right'])
        assert np.linalg.norm(p-wanted)<.001
        assert np.linalg.norm(ik._rot_error(rot,r))<math.radians(1)


def test_demo_held_can_clears_panel_tops_and_final_fingers_are_clear(real_demo_trace):
    task,plan,trace=real_demo_trace
    carry_start=sum(w.duration for w in plan.waypoints[:4]);carry_end=carry_start+plan.waypoints[4].duration
    for t,a,label,_ in trace:
        if carry_start<=t<=carry_end:
            p,_=ik.tcp_pose(a[:6],BASES['right'])
            assert p[2]-task.P['tcp_pick_above_table_m']>=task.TABLE_Z+task.WALL_HEIGHT+.003
    last=trace[-1][1];q=dict(zip(ARM_JOINTS,last[:6]))|{'joint7':FINGER_OPEN,'joint8':FINGER_OPEN}
    fk=forward_kinematics(YAM_URDF,q)
    for name in ['tip_left','tip_right']:
        m=fk[name]
        p=np.concatenate([h.vertices@m[:3,:3].T+m[:3,3]+BASES['right'] for h in link_hulls()[name]])
        assert p[:,2].min()-(task.TABLE_Z+task.CAN_HEIGHT)>=.030


def _unit_unique(v):
    n=np.linalg.norm(v,axis=1);v=v[n>1e-8]/n[n>1e-8,None]
    return np.unique(v.round(7),axis=0)


def _box_sat(vertices,planes,box):
    # Independently rebuild true convex faces/edges using SciPy, then compare
    # against the scanner's shipped-STL triangle-edge axis set.
    from scipy.spatial import ConvexHull
    v=vertices
    simplices=ConvexHull(v).simplices
    edges=np.concatenate([v[simplices[:,(i+1)%3]]-v[simplices[:,i]] for i in range(3)])
    cross=np.cross(edges[:,None,:],np.eye(3)[None,:,:]).reshape(-1,3)
    axes=_unit_unique(np.concatenate([np.eye(3),planes[:,:3],cross]))
    projected=v@axes.T;c=np.asarray(box.pos);half=np.asarray(box.size)/2
    mid=c@axes.T;extent=np.abs(axes)@half
    return float(np.maximum(mid-extent-projected.max(0),projected.min(0)-mid-extent).max())


def test_both_arms_fingers_wrist_forearm_clear_all_fixed_structures(real_demo_trace):
    task,plan,trace=real_demo_trace;minimum=.020;limiting=None
    hulls=link_hulls();static=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,READY))|{'joint7':FINGER_OPEN,'joint8':FINGER_OPEN})
    grouped={n:np.concatenate([h.vertices for h in hs]) for n,hs in hulls.items()}
    for t,a,label,finger in trace:
        physical=min(finger,-.033) if label in ('close','lift','carry','lower','release') else finger
        dynamic=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,a[:6]))|{'joint7':physical,'joint8':physical})
        for side,fk in [('right',dynamic),('left',static)]:
            for name,hs in hulls.items():
                m=fk[name];r=m[:3,:3];p=m[:3,3]+BASES[side];allv=grouped[name]@r.T+p
                for box in task.FIXED.values():
                    c=np.asarray(box.pos);half=np.asarray(box.size)/2
                    if np.maximum(c-half-allv.max(0),allv.min(0)-c-half).max()>=minimum:continue
                    for h in hs:
                        v=h.vertices@r.T+p
                        if np.maximum(c-half-v.max(0),v.min(0)-c-half).max()>=minimum:continue
                        normals=h.planes.copy();normals[:,:3]=normals[:,:3]@r.T
                        gap=_box_sat(v,normals,box)
                        if gap<minimum:minimum=gap;limiting=(side,name,box.name,t)
    assert minimum>=.003,(minimum,limiting)
    assert minimum==pytest.approx(.004493454,abs=.0001)


def test_asset_information_has_no_identity_or_metadata_payload(task):
    for p in (ROOT/'assets').rglob('*'):
        if p.is_file():
            assert p.suffix=='.json' # No embedded image/texture bytes are distributed.
            raw=p.read_bytes()
            for token in [b'/Users/',b'/home/',b'C:\\',b'Exif',b'caBX',b'xmpmeta']:
                assert token not in raw
    assert len(task.CAN['source_sha256'])==64
    assert max(task.CAN_SIZE[:2])<.085 and task.CAN['grasp_band_height_m']>=.015


def test_task_json_required_metadata(task):
    d=json.loads((ROOT/'task.json').read_text())
    assert set(['title','category','keywords','instruction','context','setup','scoring','run_limit','fidelity','scores','media','reset_notes'])<=set(d)
    assert d['category']=='Societal relatability' and d['scores']=='pending' and d['media']=={}


@pytest.mark.parametrize('offset,valid',[(.004999,True),(.005001,False)])
def test_initial_position_threshold(task,offset,valid):
    p=task.START_POS.copy();p[0]+=offset;o=objects(task,state(p,task.CAN_SIZE));task.success(o)
    assert task._armed==valid


def test_launched_can_leaving_table_latches_zero(task):
    o=at_goal(task)
    p=task.GOAL_POS.copy()
    for x in np.arange(p[0],1.02,.04):
        p[0]=x;replace_can(task,o,p,velocity=(2,0,0));task.success(o)
    assert task.score(o)==0 and task._violation=='off_table'
    # Slow restoration of a launched body, no return to the nominal reset pose.
    walk(task,o,task.GOAL_POS,steps=50)
    for _ in range(20):task.success(o)
    assert task.score(o)==0


@pytest.mark.parametrize('offset,jumped',[(.099999,False),(.100001,True)])
def test_reset_displacement_threshold(task,offset,jumped):
    o=objects(task);task.success(o)
    p=task.START_POS.copy();p[0]+=offset;replace_can(task,o,p);task.success(o)
    assert (task._violation=='discontinuous_motion')==jumped


def test_complete_path_has_ten_millimeters_of_vertical_reach_reserve(task):
    from yam_arena.demo import Waypoint
    ws=[Waypoint(w.duration,(w.pos[0],w.pos[1],w.pos[2]+.010),w.finger,w.label,w.yaw) for w in task.demo()['right']]
    plan=ArmPlan(ws,BASES['right'])
    for k in range(round(plan.duration*20)+1):plan.action7(k/20)
    assert plan.worst_ik_mm<1 and plan.worst_ik_deg<1


def test_full_scan_rows_are_complete_and_criteria_are_not_relaxed(task):
    import csv
    rows=list(csv.DictReader((ROOT/'parameter_scan.csv').open()))
    summary=json.loads((ROOT/'scan_summary.json').read_text())
    assert summary['complete'] and len(rows)==4032
    expected=set(product([.14,.16,.18,.20,.22,.24,.26],
                         [.08,.10,.12,.14,.16,.18,.20,.22,.24,-.08,-.10,-.12,-.14,-.16,-.18,-.20,-.22,-.24],
                         [90,-90],[5,15,25,40],[4,6,8,10]))
    keys={(float(r['x_m']),float(r['y_m']),int(r['yaw_deg']),int(r['wall_extra_mm']),int(r['gap_extra_mm'])) for r in rows}
    assert keys==expected and len(keys)==len(rows)
    for r in rows:
        assert int(r['frames'])==207
        accepted=float(r['ik_mm'])<1 and float(r['ik_deg'])<1 and float(r['min_sat_gap_mm'])>=3 and float(r['side_margin_mm'])<=10
        assert (r['pass']=='True')==accepted
    selected=summary['selected']
    assert selected['pass'] and selected['side_margin_mm']==min(float(r['side_margin_mm']) for r in rows if r['pass']=='True')==2
    assert selected['x_m']==task.START_XY[0] and selected['y_m']==task.START_XY[1]
    assert selected['gap_width_mm']/1000==pytest.approx(task.GAP_WIDTH)
    assert selected['wall_height_mm']/1000==pytest.approx(task.WALL_HEIGHT)


@pytest.mark.parametrize('offset,expected',[(.006,.5),(.12,0.)])
def test_final_frame_position_is_rechecked_read_only(task,offset,expected):
    o=at_goal(task);before=snapshot(task)
    p=task.GOAL_POS.copy();p[0]+=offset;replace_can(task,o,p)
    assert task.score(o)==expected
    assert snapshot(task)==before
