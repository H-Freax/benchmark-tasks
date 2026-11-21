"""CPU-only YAM feasibility scan; no simulator, task, scorer, or framework patch.

Run with yam_arena on PYTHONPATH, plus numpy, scipy, python-fcl and usd-core.
All distances in the CSV are mm; all angles are degrees unless named otherwise.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
import hashlib
import inspect
import itertools
import json
import math
from pathlib import Path
import platform
import subprocess
import time
import xml.etree.ElementTree as ET

import fcl
import numpy as np
import scipy
from scipy.spatial import ConvexHull

from yam_arena import ASSET_DIR, YAM_URDF, ik
from yam_arena.analysis import read_stl
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, FINGER_CLOSED, PAD_TIP_PAST_TCP, READY, TCP_OFFSET
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import forward_kinematics, parse_joints, rotation_angle_deg, rpy_matrix


HZ = 20
DIAMETER = .067
WALL_LENGTH = .250
WALL_THICKNESS = .018
TABLE_Z = TABLE_TOP['station']
LINK_NAMES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')
G = None
MEASURE = None


@dataclass
class Hull:
    link: str
    index: int
    vertices: np.ndarray
    obj: object
    source: str


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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


def link_points(hulls, poses, names, reference=None):
    inv = np.linalg.inv(reference) if reference is not None else np.eye(4)
    points = []
    for hull in hulls:
        if hull.link in names:
            m = inv @ poses[hull.link]
            points.append(hull.vertices @ m[:3, :3].T + m[:3, 3])
    return np.concatenate(points)


def bounds_record(points):
    return dict(bounds_mm=(np.array([points.min(0), points.max(0)])*1000).tolist(),
                closing_axis_width_mm=float(np.ptp(points[:, 0])*1000),
                transverse_width_mm=float(np.ptp(points[:, 1])*1000),
                approach_axis_length_mm=float(np.ptp(points[:, 2])*1000))


def measurements(can_path):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(str(can_path))
    cache = UsdGeom.XformCache()
    points = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            local = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            m = np.asarray(cache.GetLocalToWorldTransform(prim))
            points.append((np.c_[local, np.ones(len(local))] @ m)[:, :3] * UsdGeom.GetStageMetersPerUnit(stage))
    v = np.concatenate(points)
    raw = np.ptp(v, axis=0)
    scale = DIAMETER / max(raw[:2])
    hulls = read_hulls()
    poses = forward_kinematics(YAM_URDF, joint_map(READY, 0.0))
    fingers0 = link_points(hulls, poses, ('tip_left', 'tip_right'), poses['gripper']) - TCP_OFFSET
    pad0 = fingers0[fingers0[:, 2] >= 0]
    left0 = link_points(hulls, poses, ('tip_left',), poses['gripper']) - TCP_OFFSET
    right0 = link_points(hulls, poses, ('tip_right',), poses['gripper']) - TCP_OFFSET
    aperture0 = float(right0[right0[:, 2] >= 0, 0].min() - left0[left0[:, 2] >= 0, 0].max())
    contact = -(DIAMETER - aperture0)/2
    opening = -(DIAMETER + .004 - aperture0)/2
    poses = forward_kinematics(YAM_URDF, joint_map(READY, contact))
    fingers = link_points(hulls, poses, ('tip_left', 'tip_right'), poses['gripper']) - TCP_OFFSET
    palm = link_points(hulls, poses, ('gripper',), poses['gripper']) - TCP_OFFSET
    wrist = link_points(hulls, poses, ('link5',), poses['gripper']) - TCP_OFFSET
    wrist_xy = wrist[:, :2]
    wrist_diameter = max(np.linalg.norm(a-wrist_xy, axis=1).max() for a in wrist_xy)
    tip_extension = float(fingers[:, 2].max())
    palm_bottom = float(palm[:, 2].max())
    full_open_poses = forward_kinematics(YAM_URDF, joint_map(READY, FINGER_OPEN))
    full_open = link_points(hulls, full_open_poses, ('tip_left', 'tip_right'), full_open_poses['gripper']) - TCP_OFFSET
    entry_poses = forward_kinematics(YAM_URDF, joint_map(READY, opening))
    entry = link_points(hulls, entry_poses, ('tip_left', 'tip_right'), entry_poses['gripper']) - TCP_OFFSET
    report = dict(
        constants=dict(PAD_TIP_PAST_TCP=PAD_TIP_PAST_TCP, TCP_OFFSET=TCP_OFFSET,
                       FINGER_OPEN=FINGER_OPEN, FINGER_CLOSED=FINGER_CLOSED,
                       framework_CONTROL_HZ=__import__('yam_arena.constants', fromlist=['CONTROL_HZ']).CONTROL_HZ,
                       scan_HZ=HZ, station_table_top_m=TABLE_Z, station_bases_m=BASES),
        can=dict(registry_name='tomato_soup_can_ycb_robolab', source_sha256=sha(can_path),
                 source_url='https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/tomato_soup_can.usd',
                 source_release='6.2 public asset mirror; not an Isaac Sim 6.1 runtime',
                 raw_size_mm=(raw*1000).tolist(), uniform_scale=scale,
                 scan_size_mm=(raw*scale*1000).tolist(), diameter_mm=67.0,
                 height_m=float(raw[2]*scale)),
        measured_tip_past_tcp_mm=tip_extension*1000,
        palm_bottom_above_tcp_mm=-palm_bottom*1000,
        L_palm_bottom_to_fingertip_mm=(tip_extension-palm_bottom)*1000,
        L_definition='Lowest collision surface of gripper/TCP link to lowest fingertip, exact downward grip.',
        contact_finger_joint_m=contact, entry_finger_joint_m=opening,
        entry_aperture_mm=71.0, contact_aperture_mm=67.0,
        pad_aperture_at_zero_joint_mm=aperture0*1000,
        finger_contact=bounds_record(fingers), finger_entry=bounds_record(entry), finger_fully_open=bounds_record(full_open),
        palm=bounds_record(palm), wrist_link5_at_READY_relative_to_gripper=bounds_record(wrist),
        wrist_max_yaw_envelope_width_mm=float(wrist_diameter*1000),
        width_rule='d<=L: max(67 mm, finger transverse span); d>L: also include palm and wrist link5 transverse spans at grasp. Actual every-frame geometry is checked even when nominal d<=L.',
        geometry=dict(collision_hulls=len(hulls), by_link=dict(Counter(h.link for h in hulls)),
                      urdf_sha256=sha(YAM_URDF), meshes={h.source: sha(ASSET_DIR/h.source) for h in hulls}),
        limitations=['Geometric rigid can-blocked finger aperture, no contact/friction dynamics.',
                     'URDF collision hulls exclude camera visual attachments not present as URDF collision elements.',
                     'Wrist width depends on joint6; its scan extrema are added after the full scan.'],
    )
    return report


def init_worker(measure):
    global G, MEASURE
    MEASURE = measure
    G = read_hulls()


def orientation(x, y, theta):
    base = BASES['left' if y>0 else 'right']
    radial = math.atan2(y-base[1], x-base[0])
    long_angle = radial + math.copysign(math.radians(theta), y)
    return long_angle, long_angle-math.pi/2


def waypoints(x, y, height, h, yaw):
    start_z = TABLE_Z + height + MEASURE['measured_tip_past_tcp_mm']/1000 + .030
    pick_z = TABLE_Z+h
    clear_z = TABLE_Z+height+h+.003
    final_z = TABLE_Z+height+h+.030
    return [Waypoint(.05, (x,y,start_z), MEASURE['entry_finger_joint_m'], 'above gap', yaw),
            Waypoint(1.5, (x,y,pick_z), MEASURE['entry_finger_joint_m'], 'descend', yaw),
            Waypoint(.8, (x,y,pick_z), FINGER_CLOSED, 'close', yaw),
            Waypoint(1.5, (x,y,clear_z), FINGER_CLOSED, 'vertical extraction', yaw),
            Waypoint(1.0, (x,y,final_z), FINGER_CLOSED, 'rise to 30 mm clearance', yaw)]


def seed_pose(target, rotation, base):
    quat = ik.matrix_to_quat_xyzw(rotation)
    seeds = [np.asarray(READY), np.array([0.,1.,1.,-.6,0.,math.pi/2]),
             np.array([0.,1.,1.,-.6,0.,-math.pi/2])]
    candidates = [ik.solve(target, quat, q, base_pos=base, iters=200) for q in seeds]
    return min(candidates, key=lambda r:max(r[1]/.0005, r[2]/math.radians(.1)))


def trajectory(x, y, height, h, yaw):
    side = 'left' if y>0 else 'right'
    base = np.asarray(BASES[side])
    wps = waypoints(x,y,height,h,yaw)
    q, _, _ = seed_pose(wps[0].pos, grip_rotation(yaw), base)
    plan = ArmPlan(wps, base, q_start=q, finger_start=MEASURE['entry_finger_joint_m'])
    frames = []
    min_margin = math.inf
    at_limit = set()
    max_actual_p = 0.
    max_actual_r = 0.
    for frame in range(round(plan.duration*HZ)+1):
        t = frame/HZ
        action, label = plan.action7(t)
        target, rot, finger, _ = plan.target(t)
        actual, actual_rot = ik.tcp_pose(action[:6], base)
        max_actual_p = max(max_actual_p, float(np.linalg.norm(actual-target))*1000)
        max_actual_r = max(max_actual_r, rotation_angle_deg(rot,actual_rot))
        margin = np.minimum(action[:6]-ik.LOWER, ik.UPPER-action[:6])
        min_margin = min(min_margin,float(margin.min()))
        at_limit.update(ARM_JOINTS[i] for i in np.flatnonzero(margin<=1e-8))
        physical_finger = min(finger, MEASURE['contact_finger_joint_m'])
        poses = forward_kinematics(YAM_URDF,joint_map(action[:6],physical_finger))
        frames.append((t,label,action[:6].copy(),poses))
    above_errors = []
    elapsed = 0.
    for w in wps:
        elapsed += w.duration
        nearest = min(frames, key=lambda f:abs(f[0]-elapsed))
        q2, pe, re = ik.solve(np.array(w.pos)+(0,0,.010),ik.matrix_to_quat_xyzw(grip_rotation(yaw)),
                               nearest[2],base_pos=base,iters=200)
        margin = np.minimum(q2-ik.LOWER, ik.UPPER-q2)
        above_errors.append(dict(label=w.label,ik_mm=pe*1000,ik_deg=math.degrees(re),
                                 joint_margin_deg=math.degrees(float(margin.min()))))
        min_margin=min(min_margin,float(margin.min()))
        at_limit.update(ARM_JOINTS[i] for i in np.flatnonzero(margin<=1e-8))
    pick_frame = min(frames,key=lambda f:abs(f[0]-(wps[0].duration+wps[1].duration)))
    palm = link_points(G,pick_frame[3],('gripper',),pick_frame[3]['gripper'])-TCP_OFFSET
    wrist = link_points(G,pick_frame[3],('link5',),pick_frame[3]['gripper'])-TCP_OFFSET
    return dict(frames=frames,waypoints=wps,ik_mm=max_actual_p,ik_deg=max_actual_r,
                reported_ik_mm=plan.worst_ik_mm,reported_ik_deg=plan.worst_ik_deg,
                joint_margin_deg=math.degrees(min_margin),limit_joints=sorted(at_limit),
                above_errors=above_errors,palm_width_mm=float(np.ptp(palm[:,1])*1000),
                wrist_width_mm=float(np.ptp(wrist[:,1])*1000),
                wrist_closing_width_mm=float(np.ptp(wrist[:,0])*1000),
                yaw=yaw)


def geometry_distances(traj, x, y, height, long_angle, widths):
    side = 'left' if y>0 else 'right'
    other = 'right' if y>0 else 'left'
    ca,sa=math.cos(long_angle),math.sin(long_angle)
    wall_rot=np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]])
    origin=np.array([x,y,TABLE_Z])
    centers=np.array([[0,s*(width/2+WALL_THICKNESS/2),height/2] for width in widths for s in (-1,1)])
    half=np.array([WALL_LENGTH/2,WALL_THICKNESS/2,height/2])
    boxes=[fcl.CollisionObject(fcl.Box(WALL_LENGTH,WALL_THICKNESS,height),fcl.Transform(c)) for c in centers]
    request=fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=False, gjk_solver_type=fcl.GJKSolverType.GST_LIBCCD)
    min_distance=np.full(len(boxes),np.inf)
    limiting=[None]*len(boxes)
    links_below=[set() for _ in widths]
    contact_links=[set() for _ in widths]
    narrow_calls=0
    shape_by_link={name:[(i,g) for i,g in enumerate(G) if g.link==name] for name in LINK_NAMES}
    bundles={}
    for name, items in shape_by_link.items():
        sizes=[len(g.vertices) for _,g in items]
        starts=np.r_[0,np.cumsum(sizes)[:-1]]
        bundles[name]=(np.concatenate([g.vertices for _,g in items]),starts,items)
    static=forward_kinematics(YAM_URDF,joint_map(READY,FINGER_OPEN))
    samples=[(0.,'inactive READY',static,other)]
    samples.extend((t,label,poses,side) for t,label,_,poses in traj['frames'])
    for t,label,poses,arm in samples:
        base=np.asarray(BASES[arm])
        for name,(verts,starts,items) in bundles.items():
            m=poses[name]
            r=wall_rot.T@m[:3,:3]
            p=wall_rot.T@(m[:3,3]+base-origin)
            points=verts@r.T+p
            lo=np.minimum.reduceat(points,starts,axis=0)
            hi=np.maximum.reduceat(points,starts,axis=0)
            sep=np.maximum(centers[None,:,:]-half-hi[:,None,:],lo[:,None,:]-centers[None,:,:]-half)
            lower=np.linalg.norm(np.maximum(sep,0),axis=2)
            candidates=np.argwhere(lower<=np.maximum(min_distance[None,:],.00300001))
            if not len(candidates):
                continue
            transforms={}
            for gi, wi in candidates:
                hull=items[gi][1]
                if gi not in transforms:
                    hull.obj.setTransform(fcl.Transform(r,p))
                    transforms[gi]=True
                result=fcl.DistanceResult()
                gap=fcl.distance(hull.obj,boxes[wi],request,result)
                narrow_calls+=1
                if not np.isfinite(gap):
                    raise ArithmeticError('Nonfinite GJK distance')
                if gap<.003:
                    links_below[wi//2].add(f'{arm}:{name}')
                if gap<=0:
                    contact_links[wi//2].add(f'{arm}:{name}')
                    gap=0.
                if gap<min_distance[wi]:
                    min_distance[wi]=gap
                    limiting[wi]=dict(arm=arm,link=name,hull=hull.index,wall=int(wi%2),time_s=t,segment=label)
    output=[]
    for j in range(len(widths)):
        k=2*j+int(np.argmin(min_distance[2*j:2*j+2]))
        output.append(dict(min_wall_distance_mm=float(min_distance[k]*1000),limiting=limiting[k],
                           wall_failure_links=sorted(links_below[j]),contact_links=sorted(contact_links[j]),
                           gjk_queries=narrow_calls))
    return output


def evaluate_variant(config, axial_flip):
    x,y,theta,delta_mm,h_mm=config
    height=MEASURE['can']['height_m']+delta_mm/1000
    h=h_mm/1000
    long_angle,yaw=orientation(x,y,theta)
    yaw=(yaw+axial_flip*math.pi+math.pi)%(2*math.pi)-math.pi
    traj=trajectory(x,y,height,h,yaw)
    depth=height-h
    section=MEASURE['finger_contact']['transverse_width_mm']/1000
    if depth>MEASURE['L_palm_bottom_to_fingertip_mm']/1000:
        section=max(section,traj['palm_width_mm']/1000,traj['wrist_width_mm']/1000)
    widths=np.maximum(DIAMETER,section)+np.array([4,6,8,10])/1000
    distances=geometry_distances(traj,x,y,height,long_angle,widths)
    above_mm=max(r['ik_mm'] for r in traj['above_errors'])
    above_deg=max(r['ik_deg'] for r in traj['above_errors'])
    rows=[]
    for c,width,distance in zip((4,6,8,10),widths,distances,strict=True):
        geometry_ok=h>=.015 and h-MEASURE['measured_tip_past_tcp_mm']/1000>=.003
        ik_ok=max(traj['ik_mm'],traj['reported_ik_mm'])<.5 and max(traj['ik_deg'],traj['reported_ik_deg'])<.1
        above_ok=above_mm<.5 and above_deg<.1
        limit_ok=traj['joint_margin_deg']>=-1e-7
        wall_ok=distance['min_wall_distance_mm']>=3
        reasons=[]
        if not ik_ok:reasons.append('IK')
        if not above_ok:reasons.append('waypoint_plus_10mm')
        if not limit_ok:reasons.append('joint_limit_violation')
        if traj['limit_joints']:reasons.append('joint_limit_saturation')
        if not wall_ok:reasons.append('wall_clearance')
        if not geometry_ok:reasons.append('fingertip_floor')
        row=dict(x_m=x,y_m=y,arm='left' if y>0 else 'right',theta_deg=theta,
                 long_axis_deg=math.degrees(long_angle),yaw_deg=math.degrees(yaw),axial_flip=axial_flip,
                 h_mm=h_mm,delta_mm=delta_mm,c_mm=c,d_mm=depth*1000,H_mm=height*1000,w_mm=float(width*1000),
                 intrusion='fingers' if depth<=MEASURE['L_palm_bottom_to_fingertip_mm']/1000 else 'fingers_palm_wrist',
                 min_wall_distance_mm=distance['min_wall_distance_mm'],ik_position_mm=traj['ik_mm'],
                 ik_orientation_deg=traj['ik_deg'],armplan_worst_ik_mm=traj['reported_ik_mm'],
                 armplan_worst_ik_deg=traj['reported_ik_deg'],plus10_ik_mm=above_mm,plus10_ik_deg=above_deg,
                 min_ik_position_margin_mm=.5-max(traj['ik_mm'],traj['reported_ik_mm'],above_mm),
                 min_ik_orientation_margin_deg=.1-max(traj['ik_deg'],traj['reported_ik_deg'],above_deg),
                 min_joint_limit_margin_deg=traj['joint_margin_deg'],limit_joints=';'.join(traj['limit_joints']),
                 ik_pass=ik_ok,plus10_pass=above_ok,joint_limits_pass=limit_ok,wall_pass=wall_ok,
                 fingertip_floor_pass=geometry_ok,passed=ik_ok and above_ok and limit_ok and wall_ok and geometry_ok,
                 failure_reasons=';'.join(reasons),wall_failure_links=';'.join(distance['wall_failure_links']),
                 contact_links=';'.join(distance['contact_links']),
                 limiting_arm=distance['limiting']['arm'],limiting_link=distance['limiting']['link'],
                 limiting_hull=distance['limiting']['hull'],limiting_wall=distance['limiting']['wall'],
                 limiting_time_s=distance['limiting']['time_s'],limiting_segment=distance['limiting']['segment'],
                 frames=len(traj['frames']),waypoint_plus10_checks=len(traj['above_errors']),
                 wrist_transverse_width_mm=traj['wrist_width_mm'],wrist_closing_width_mm=traj['wrist_closing_width_mm'],
                 gjk_queries=distance['gjk_queries'])
        rows.append(row)
    return rows


def row_rank(row):
    return (not row['passed'], max(-row['min_ik_position_margin_mm']/.5,-row['min_ik_orientation_margin_deg']/.1,
                                 (3-row['min_wall_distance_mm'])/3),-row['min_joint_limit_margin_deg'])


def scan_one(config):
    variants=[evaluate_variant(config,flip) for flip in (0,1)]
    selected=[]
    for i in range(4):
        row=min((v[i] for v in variants),key=row_rank).copy()
        row['yaw_variants_tested']=2
        row['all_variants_ik_failed']=all(not v[i]['ik_pass'] for v in variants)
        row['all_variants_plus10_failed']=all(not v[i]['plus10_pass'] for v in variants)
        row['all_variants_wall_failed']=all(not v[i]['wall_pass'] for v in variants)
        row['all_variants_at_joint_limit']=all(bool(v[i]['limit_joints']) for v in variants)
        selected.append(row)
    return selected,[r for v in variants for r in v]


def environment():
    def commit(path):
        return subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
    import yam_arena
    return dict(platform=platform.platform(),machine=platform.machine(),python=platform.python_version(),
                numpy=np.__version__,scipy=scipy.__version__,python_fcl=fcl.__version__,
                yam_source=str(Path(yam_arena.__file__).parent),yam_commit=commit(Path(yam_arena.__file__).parent),
                isaac_sim_used=False,gpu_used=False,contact_simulation_used=False,
                framework_sources={name:sha(Path(inspect.getsourcefile(obj))) for name,obj in
                                   [('ArmPlan',ArmPlan),('ik.solve',ik.solve),('urdf.forward_kinematics',forward_kinematics)]})


def summary(rows,variants,elapsed,env):
    passing=[r for r in rows if r['passed']]
    best_by_delta={}
    for delta in (5,15,25,40):
        eligible=[r for r in passing if r['delta_mm']==delta]
        best_by_delta[str(delta)]=min(eligible,key=lambda r:(r['c_mm'],-r['min_wall_distance_mm'],
                                       -r['min_joint_limit_margin_deg']),default=None)
    failure_links=Counter()
    contact_links=Counter()
    for row in rows:
        failure_links.update(filter(None,row['wall_failure_links'].split(';')))
        contact_links.update(filter(None,row['contact_links'].split(';')))
    top=sorted(passing,key=lambda r:(r['c_mm'],-r['min_joint_limit_margin_deg'],
                                    -min(r['min_ik_position_margin_mm']/.5,r['min_ik_orientation_margin_deg']/.1),
                                    r['delta_mm']))
    diverse=[]
    for row in top:
        if all((row['x_m'],row['y_m'],row['theta_deg'],row['h_mm']) !=
               (r['x_m'],r['y_m'],r['theta_deg'],r['h_mm']) for r in diverse):
            diverse.append(row)
        if len(diverse)==3:break
    return dict(complete=len(rows)==3456,combinations=len(rows),yaw_variant_rows=len(variants),
                passing=len(passing),failed=len(rows)-len(passing),elapsed_s=elapsed,environment=env,
                best_by_delta=best_by_delta,best_three=diverse,
                failures_nonexclusive=dict(IK=sum(not r['ik_pass'] for r in rows),
                  waypoint_plus_10mm=sum(not r['plus10_pass'] for r in rows),
                  wall_clearance_below_3mm=sum(not r['wall_pass'] for r in rows),
                  actual_wall_contact=sum(bool(r['contact_links']) for r in rows),
                  joint_limit_violation=sum(not r['joint_limits_pass'] for r in rows),
                  joint_limit_saturation=sum(bool(r['limit_joints']) for r in rows),
                  fingertip_floor=sum(not r['fingertip_floor_pass'] for r in rows)),
                wall_failure_links=dict(failure_links),actual_contact_links=dict(contact_links),
                limit_joints=dict(Counter(j for r in rows for j in filter(None,r['limit_joints'].split(';')))),
                wrist_scanned_widths_mm=dict(transverse=[min(r['wrist_transverse_width_mm'] for r in variants),
                                                        max(r['wrist_transverse_width_mm'] for r in variants)],
                                             closing=[min(r['wrist_closing_width_mm'] for r in variants),
                                                      max(r['wrist_closing_width_mm'] for r in variants)]),
                conventions=['3456 geometric combinations; two 180-degree-equivalent downward yaws tested for each.',
                  'For each row choose a passing yaw if present, else smallest normalized threshold violation. Both yaw results are retained in yaw_variants.csv.',
                  'theta is outward signed: long-axis angle=atan2(y-base_y,x-base_x)+sign(y)*theta. Left and right geometries are mirrored.',
                  'Only the own-side arm moves; inactive robot at READY is also checked against walls.',
                  'Start at a downward hover with fingertips 30 mm above walls; seed IK is initialization, not a READY-to-hover motion.',
                  '98 frames at 20 Hz, includes every waypoint endpoint. The whole can clears the wall by 3 mm before its bottom rises to 30 mm above the wall.',
                  'Physical closure stops at the 67 mm can; commanded finger action closes to zero. No grasp dynamics claimed.',
                  'Every waypoint is also solved at +10 mm with the same orientation and threshold.',
                  'GJK Euclidean convex-body distance, not SAT margin; zero means contact/intersection, penetration depth not reported.',
                  'Per-hull exact transformed AABB distance only prunes candidates whose lower bound exceeds both running minimum and 3 mm.',
                  'Joint saturation at an inclusive URDF bound is reported separately from actual limit violation. IK clips its output to these bounds.',
                  'Failures overlap; counts refer to selected-yaw rows, not unique causes or probabilities.'],
                thresholds=dict(position_mm_lt=.5,orientation_deg_lt=.1,wall_distance_mm_ge=3,
                                fingertip_floor_mm_ge=3,tcp_floor_mm_ge=15),
                script_sha256=sha(__file__))


def write_csv(path,rows):
    import csv
    with path.open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--can',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path(__file__).parent)
    p.add_argument('--workers',type=int,default=6)
    p.add_argument('--pilot',action='store_true')
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    m=measurements(args.can)
    (args.out/'gripper_dims.json').write_text(json.dumps(m,indent=2)+'\n')
    print(json.dumps({k:v for k,v in m.items() if k not in ('geometry',)},indent=2),flush=True)
    env=environment()
    (args.out/'environment.json').write_text(json.dumps(env,indent=2)+'\n')
    configs=list(itertools.product((.16,.20,.24),(.10,.16,.22,-.10,-.16,-.22),
                                   (0,30,60,90),(5,15,25,40),(35,50,65)))
    if args.pilot:configs=[(.16,-.10,60,5,50),(.20,.16,30,25,35)]
    rows=[];variants=[];start=time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers,initializer=init_worker,initargs=(m,)) as pool:
        futures=[pool.submit(scan_one,c) for c in configs]
        for future in as_completed(futures):
            selected,all_rows=future.result()
            rows.extend(selected);variants.extend(all_rows)
            if len(rows)%48==0 or args.pilot:
                print(json.dumps(dict(completed=len(rows),expected=len(configs)*4,
                                      passing=sum(r['passed'] for r in rows),elapsed_s=round(time.monotonic()-start,1))),flush=True)
            if len(rows)%96==0:
                write_csv(args.out/'gap_scan.partial.csv',rows)
    rows.sort(key=lambda r:(r['x_m'],r['y_m'],r['theta_deg'],r['delta_mm'],r['h_mm'],r['c_mm']))
    variants.sort(key=lambda r:(r['x_m'],r['y_m'],r['theta_deg'],r['delta_mm'],r['h_mm'],r['c_mm'],r['axial_flip']))
    write_csv(args.out/'gap_scan.csv',rows)
    write_csv(args.out/'yaw_variants.csv',variants)
    report=summary(rows,variants,time.monotonic()-start,env)
    (args.out/'scan_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    m['wrist_widths_in_scan_mm']=report['wrist_scanned_widths_mm']
    (args.out/'gripper_dims.json').write_text(json.dumps(m,indent=2)+'\n')
    partial=args.out/'gap_scan.partial.csv'
    if partial.exists():partial.unlink()
    print(json.dumps(report),flush=True)


if __name__=='__main__':
    main()
