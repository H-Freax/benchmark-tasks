"""CPU tests; geometry and synthetic ObjectState traces are not native physics evidence."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def test_official_loader_in_fresh_process():
    assert (ROOT / "task.py").is_file(), "Task source must exist before official load"
    code = '''
import sys
from yam_arena.task_file import load
p = sys.argv[1]
a = load(p)
b = load(p)
assert a.scene == b.scene == "station"
assert a.names == b.names
assert a.free_objects == ("door",)
assert a.score and a.success and a.demo
assert not any(n in sys.modules for n in ("pxr", "isaaclab.sim", "yam_arena.scene"))
assert len([f for f in sys.meta_path if getattr(f, "lift_off_scene_hook", False)]) == 1
print("official loader twice; native imports absent; one scene hook")
'''
    result = subprocess.run([sys.executable, "-c", code, str(ROOT / "task.py")], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def geometry():
    import importlib.util
    p = ROOT / 'geometry.py'
    assert p.is_file(), 'Machinable pin-and-annulus geometry must exist'
    spec = importlib.util.spec_from_file_location('_lift_off_test_geometry', p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_real_scale_pin_shoulders_and_open_bores():
    import numpy as np
    g = geometry()
    assert np.allclose(g.PANEL_SIZE, (.018,.350,.400))
    assert 1.2 < g.DOOR_MASS < 1.4
    assert g.DOOR_MASS < 7
    assert np.isclose(g.HINGE_Z[1]-g.HINGE_Z[0], .26)
    assert 0.0002 <= g.BORE_RADIUS-g.PIN_RADIUS <= .0005
    assert g.PIN_LENGTH < g.SLEEVE_HEIGHT
    for ring in g.ring_segments():
        assert ring.shape == (8,3)
        # The convex wedge does not cross the circular pin bore.
        assert np.linalg.norm(ring[:,:2],axis=1).min() >= g.BORE_RADIUS-1e-12
    seated = g.hinge_clearances(g.START, np.eye(3))
    lifted = g.hinge_clearances(g.START + [0,0,.027], np.eye(3))
    pulled = g.hinge_clearances(g.START + [-.001,0,0], np.eye(3))
    assert not seated['detached'] and not pulled['detached']
    assert lifted['detached']
    assets = g.asset_texts()
    assert set(assets) == {'door.usda','cabinet.usda','landing_pad.usda'}
    assert 'def Cylinder "pin_0"' in assets['cabinet.usda']
    assert 'def Cylinder "shoulder_0"' in assets['cabinet.usda']
    assert 'ring_0_0' in assets['door.usda']
    assert all('PhysicsJoint' not in text for text in assets.values())
    assert all('prepend apiSchemas = []' not in text for text in assets.values())


def test_partial_credit_requires_actual_pin_detachment():
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    assert hasattr(m,'new_scorer'), 'Measured-pose scorer with numeric partial credit must exist'
    s=m.new_scorer()
    sample=dict(time=0.,origin=m.g.START.copy(),quat=[0.,0.,0.,1.],lin_vel=[0.,0.,0.],
                ang_vel=[0.,0.,0.],robot_clearance=0.,pad_up_force=0.,table_up_force=0.,measurement_valid=True)
    assert s.update(sample)['score']==0.
    sample['origin']=m.g.START+[0.,0.,.027]
    sample['time']=.05
    assert s.update(sample)['score']==.25
    sample['origin']=m.g.GOAL+[-.02,0.,0.]
    sample['time']=.10
    assert s.update(sample)['score']==.5


def test_full_score_requires_released_supported_still_door():
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    assert hasattr(m,'new_scorer'), 'Native measured support/release scorer must exist'
    s=m.new_scorer()
    sample=dict(origin=m.g.GOAL.copy(),quat=[0.,0.,0.,1.],lin_vel=[0.,0.,0.],ang_vel=[0.,0.,0.],
                robot_clearance=.04,pad_up_force=m.g.DOOR_MASS*9.81,table_up_force=0.,measurement_valid=True)
    for k in range(21):
        r=s.update(dict(sample,time=k/20))
    assert r['score']==1. and r['success']
    assert r['window_s']>=1.
    for name,value in [('robot_clearance',0.),('pad_up_force',0.),('lin_vel',[.02,0.,0.]),
                       ('measurement_valid',False)]:
        bad=dict(sample,time=1.05,**{name:value})
        assert not s.update(bad)['success']
    assert s.update({})['score']==0.
    s.reset()
    assert s.last['score']==0. and not s.history


def test_initial_leaf_plates_do_not_cut_through_fixed_shoulders():
    import numpy as np
    g=geometry()
    for name,size,center in g.DOOR_BOXES:
        lo,hi=np.asarray(center)-np.asarray(size)/2,np.asarray(center)+np.asarray(size)/2
        for z in g.HINGE_Z:
            if hi[2]<=z-g.SHOULDER_HEIGHT or lo[2]>=z:
                continue
            gap=np.maximum(np.maximum(lo[:2]-g.HINGE_XY,g.HINGE_XY-hi[:2]),0.)
            assert np.linalg.norm(gap)>=g.OUTER_RADIUS+2*g.CONTACT_OFFSET, name


def test_real_native_adapter_is_deferred_and_force_sign_is_preserved():
    from types import SimpleNamespace
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    assert callable(getattr(m.bridge,'patch_scene',None)), 'Native USDA spawn/sensor bridge must exist'
    b=m.bridge
    assert b.upward_support_force([[[0,0,4.]]])==4.
    assert b.upward_support_force([[[0,0,-4.]]])==0.
    calls=[]
    fake=SimpleNamespace(task_object=lambda x:x,build_task_environment=lambda *a,**k:None,
                         Object=lambda **kw:calls.append(kw) or kw,
                         ObjectType=SimpleNamespace(RIGID='rigid',BASE='base'),
                         Pose=lambda **kw:kw)
    b.patch_scene(fake)
    out=fake.task_object(m.OBJECTS[1])
    assert out['initial_pose']['position_xyz']==tuple(m.DOOR_RESET_ORIGIN)
    assert out['usd_path'].endswith('/door.usda') and out['object_type']=='rigid'
    first=fake.task_object
    b.patch_scene(fake)
    assert fake.task_object is first


def test_scripted_bimanual_grasp_uses_measured_fk_not_object_writes():
    import numpy as np
    from yam_arena.task_file import load
    from yam_arena import ik,station
    from yam_arena.demo import ArmPlan
    m=load(ROOT/'task.py').module
    routes=m.demo()
    assert set(routes)=={'left','right'}, 'Runnable bimanual door grasp and lift routes must exist'
    for side,waypoints in routes.items():
        labels={w.label for w in waypoints}
        assert {'grasp','lift_clear','withdraw','lower','release','retreat'}<=labels
        plan=ArmPlan(waypoints,station.BASES[side])
        previous=plan.q.copy()
        for k in range(round(plan.duration*20)+1):
            action,label=plan.action7(k/20)
            target,rotation,_,_=plan.target(k/20)
            actual,actual_rot=ik.tcp_pose(action[:6],station.BASES[side])
            assert np.linalg.norm(actual-target)<.002,(side,k,label)
            assert np.linalg.norm(actual_rot-rotation)<.02,(side,k,label)
            assert np.max(np.abs(action[:6]-previous))<.12
            assert np.all(action[:6]>=ik.LOWER) and np.all(action[:6]<=ik.UPPER)
            previous=action[:6]


def test_assets_parse_with_stock_openusd():
    import shutil
    uv=shutil.which('uv')
    assert uv
    script=ROOT/'check_assets.py'
    assert script.is_file(), 'Independent OpenUSD parser/extent/collider gate must exist'
    p=subprocess.run([uv,'run','--python','3.12','--with','usd-core==26.11','--no-project',
                      'python','-B',str(script)],capture_output=True,text=True,timeout=90)
    assert p.returncode==0,p.stdout+p.stderr
    assert p.stdout.count(': parsed')==3,p.stdout


def test_safe_off_pad_setdown_earns_meaningful_partial_credit():
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    s=m.new_scorer()
    sample=dict(origin=[.15,0.,m.g.TABLE_Z],quat=[0.,0.,0.,1.],lin_vel=[0.,0.,0.],ang_vel=[0.,0.,0.],
                robot_clearance=.04,pad_up_force=0.,table_up_force=0.,measurement_valid=True)
    for k in range(21):
        r=s.update(dict(sample,time=k/20))
    assert r['score']==.75 and not r['success']
    assert r['safe_support'] and r['released']


def test_moving_leaf_and_web_leave_the_real_pin_bore_open():
    import numpy as np
    g=geometry()
    for name,size,center in g.DOOR_BOXES:
        lo,hi=np.asarray(center)-np.asarray(size)/2,np.asarray(center)+np.asarray(size)/2
        for z in g.HINGE_Z:
            if hi[2]<=z or lo[2]>=z+g.PIN_LENGTH:
                continue
            gap=np.maximum(np.maximum(lo[:2]-g.HINGE_XY,g.HINGE_XY-hi[:2]),0.)
            assert np.linalg.norm(gap)>=g.PIN_RADIUS+2*g.CONTACT_OFFSET,name


def test_parent_runner_has_cpu_fk_collision_gate_and_native_command():
    script=ROOT/'run_native.py'
    assert script.is_file(), 'Parent runnable CPU and lock-owning native entrypoint must exist'
    p=subprocess.run([sys.executable,'-B',str(script),'--cpu-check'],capture_output=True,text=True,timeout=60)
    assert p.returncode==0,p.stdout+p.stderr
    import json
    r=json.loads(p.stdout)
    assert r['native_run'] is False and r['score'] is None
    assert set(r['ik'])=={'left','right'}
    assert r['collision']['fixed_penetration_m']<=.0005
    assert r['collision']['panel_penetration_m']<=.0005
    assert r['collision']['inter_arm_penetration_m']<=.0005
    p=subprocess.run([sys.executable,'-B',str(script),'--dry-run','--output','/unused/native-output'],
                     capture_output=True,text=True,timeout=30)
    assert p.returncode==0,p.stdout+p.stderr
    r=json.loads(p.stdout)
    assert r['gpu_lock'].endswith('.lock')
    assert r['command'][0]=='inspect-robots' and '--no-prompt' in r['command']


def test_cabinet_opening_matches_door_rotated_ninety_degrees():
    g=geometry()
    boxes={n:(s,c) for n,s,c in g.CABINET_BOXES}
    # A 350 mm door rotates into the x-z opening, so the full carcase
    # horizontal span must be along x; its y depth is the ordinary 240 mm.
    floor_size,_=boxes['floor']
    assert floor_size[0]>.32 and abs(floor_size[1]-.240)<1e-8


def test_inventory_binds_runtime_origins_and_asset_sizes():
    import json,numpy as np
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    assert (ROOT/'inventory.json').is_file(),'Complete actual-object inventory must exist'
    inv=json.loads((ROOT/'inventory.json').read_text())
    assert [r['id'] for r in inv['objects']]==[o.name for o in m.OBJECTS]
    for row,spec in zip(inv['objects'],m.OBJECTS,strict=True):
        assert np.allclose(row['asset_origin_world_m'],spec.origin)
        assert np.allclose(row['asset_envelope_m'],spec.size)
        assert row['movable'] is spec.free
        assert row['license_spdx']=='MIT' and row['asset_origin']=='self-authored procedural USDA'
        assert row['components'] and row['material']['intended'] and row['material']['simulated']
    metadata=json.loads((ROOT/'task.json').read_text())
    assert metadata['instruction']==m.INSTRUCTION
    assert isinstance(metadata['media'],str) and metadata['scores']


def test_initial_sleeves_start_above_shoulders_before_physical_settle():
    """Exact coplanar cylinder/sector contact explodes in the native narrow phase.

    A tiny positive initial seating clearance must settle under gravity; it is
    far below pin engagement and must not become a software attachment.
    """
    import numpy as np
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    specs={o.name:o for o in m.OBJECTS}
    for z in m.g.HINGE_Z:
        sleeve_bottom=float(specs['door'].origin[2]+z)
        shoulder_top=float(specs['cabinet'].origin[2]+z)
        gap=sleeve_bottom-shoulder_top
        assert .00015<=gap<=.0005, ('native initial seating gap',gap)
        assert gap+2*m.g.CONTACT_OFFSET<m.g.PIN_LENGTH/20
    assert np.allclose(np.asarray(specs['door'].origin)[:2],m.g.START[:2])


def test_lift_and_release_route_clears_loaded_hinges_before_side_retreat():
    """Retain extraction headroom and leave the pull axially before side travel."""
    import numpy as np
    from yam_arena.task_file import load
    m=load(ROOT/'task.py').module
    for side,route in m.demo().items():
        by_label={w.label:w for w in route}
        grasp,lift=by_label['grasp'],by_label['lift_clear']
        # Native loaded wrist lag exceeded 20 mm; 27 mm command did not clear
        # either 18 mm pin. Preserve the successful 67 mm lift command.
        assert lift.pos[2]-grasp.pos[2]>=.060
        release=by_label['release']
        pullback=by_label.get('pullback')
        assert pullback is not None, 'exit the handle before moving laterally'
        assert route.index(release)<route.index(pullback)<route.index(by_label['retreat'])
        assert pullback.pos[0]<=release.pos[0]-.035
        assert abs(pullback.pos[1]-release.pos[1])<1e-9
        assert abs(pullback.pos[2]-release.pos[2])<1e-9
        assert np.linalg.norm(np.asarray(by_label['retreat'].pos)-pullback.pos)>.1


def test_off_pad_control_is_reachable_at_original_door_scale():
    p=subprocess.run([sys.executable,'-B',str(ROOT/'run_native.py'),'--cpu-check','--variant','off_pad'],
                     capture_output=True,text=True,timeout=60)
    assert p.returncode==0,p.stdout+p.stderr
