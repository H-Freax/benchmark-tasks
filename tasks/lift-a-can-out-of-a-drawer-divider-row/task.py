"""A red soup can stands below the tops of two bamboo dividers in an open deep
kitchen drawer on the left side of the station. The dividers enclose a single
narrow row. A pale square outline marks a clear landing area beside the drawer.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path

import numpy as np
from yam_arena.analysis import link_hulls
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, Cylinder, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent
CAN = json.loads((HERE / 'assets' / 'can_info.json').read_text())
LAYOUT = json.loads((HERE / 'assets' / 'drawer_info.json').read_text())

SCENE = 'station'
TABLE_Z = float(TABLE_TOP[SCENE])
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20.0
RUN_LIMIT = 800
CAN_SIZE = np.asarray(CAN['native_size_m'], dtype=float)
CAN_HEIGHT = float(CAN_SIZE[2])
START_XY = np.asarray(LAYOUT['can_xy_m'], dtype=float)
GOAL_XY = np.asarray(LAYOUT['goal_xy_m'], dtype=float)
START_POS = np.r_[START_XY, TABLE_Z + float(CAN['resting_center_above_table_m']) + .001]
GOAL_POS = np.r_[GOAL_XY, TABLE_Z + float(CAN['resting_center_above_table_m'])]
ANGLE = float(LAYOUT['long_axis_rad'])
YAW = float(LAYOUT['yaw_rad'])
ROTATION = np.array([[math.cos(ANGLE), -math.sin(ANGLE), 0.0],
                     [math.sin(ANGLE), math.cos(ANGLE), 0.0], [0.0, 0.0, 1.0]])
WALL_HEIGHT = float(LAYOUT['divider_height_m'])
GAP_WIDTH = float(LAYOUT['gap_width_m'])
PICK_HEIGHT = .035
RELEASE_GAP = .0015
TRAVEL_HEIGHT = WALL_HEIGHT + PICK_HEIGHT + .050
MAX_TILT_DEG = 5.0
SUPPORT_TOL = .002
GOAL_TOL = .015
STILL_TOL = .002
STILL_FRAMES = 10
START_TOL = .005
RESET_JUMP = .10
HAND_CLEARANCE = .030
OPEN_GAP = .085
TABLE_BOUNDS = (-.19, .91, -.65, .65)
TASK_DEBUG = os.environ.get('TASK_DEBUG', '').lower() in ('1', 'true', 'yes')


def floats(values):
    return tuple(float(v) for v in values)


OBJECTS = [Cylinder('soup_can', pos=floats(START_POS),
                    radius=float(CAN['radius_m']), height=float(CAN_HEIGHT),
                    mass=float(CAN['mass_kg']), color=floats((.75, .08, .05)), yaw=0.0)]
for board in LAYOUT['boards_local']:
    p = ROTATION @ np.asarray(board['pos']) + np.r_[START_XY, TABLE_Z]
    OBJECTS.append(Box(board['name'], pos=floats(p), size=floats(board['size']),
                       color=floats((.72, .57, .35) if 'divider' in board['name'] else (.65, .49, .32)),
                       yaw=float(ANGLE)))
for axis, sign, name in [(0,-1,'mark_left'), (0,1,'mark_right'), (1,-1,'mark_near'), (1,1,'mark_far')]:
    xy = GOAL_XY.copy(); xy[axis] += sign * .054
    size = (.002, .110, .0005) if axis == 0 else (.106, .002, .0005)
    OBJECTS.append(Box(name, pos=floats((*xy, TABLE_Z+.00025)), size=floats(size),
                       color=floats((.94, .91, .72)), yaw=0.0))
FIXED = {s.name: s for s in OBJECTS if isinstance(s, Box)}

_history = deque(maxlen=STILL_FRAMES)
_previous = None
_previous_bottom = None
_armed = False
_off_table_latch = False
_lifted = False
_step = 0
_hands = None
_hand_reason = 'not read'
_views = None
_view_identity = None


def _numpy(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value, dtype=float)


def read_grippers():
    """Read public SimulationContext/PhysX views only; unavailable means no release credit."""
    global _views, _view_identity
    try:
        from isaaclab.sim import SimulationContext
        sim = SimulationContext.instance()
        if sim is None or sim.physics_sim_view is None:
            return None, 'no SimulationContext physics view'
        physics = sim.physics_sim_view
        if _views is None or _view_identity != id(physics):
            views = [physics.create_articulation_view('/World/envs/env_0/' + name)
                     for name in ('LeftArm', 'RightArm')]
            if any(v.count != 1 for v in views):
                raise ValueError('expected one articulation per arm')
            _views, _view_identity = views, id(physics)
        readings = []
        hulls = link_hulls()
        for view in _views:
            names = list(view.shared_metatype.dof_names)
            links = list(view.shared_metatype.link_names)
            joints = _numpy(view.get_dof_positions())[0]
            poses = _numpy(view.get_link_transforms())[0]
            points = []
            for name in ('gripper', 'tip_left', 'tip_right'):
                pose = poses[links.index(name)]
                if not np.all(np.isfinite(pose)) or abs(np.linalg.norm(pose[3:7])-1.0) > .01:
                    raise ValueError('invalid hand pose')
                rotation = quat_xyzw_to_matrix(pose[3:7])
                points.extend(h.vertices @ rotation.T + pose[:3] for h in hulls[name])
            points = np.concatenate(points)
            gap = -float(joints[names.index('joint7')] + joints[names.index('joint8')])
            if not np.isfinite(gap) or not -.0001 <= gap <= .0941:
                raise ValueError('invalid finger opening')
            readings.append(dict(low=points.min(0), high=points.max(0), gap=gap))
        return readings, 'ok'
    except Exception as error:
        _views = None
        return None, 'gripper unavailable: ' + type(error).__name__


def _valid_state(s):
    try:
        return (all(np.shape(getattr(s,k)) == (n,) and np.all(np.isfinite(getattr(s,k)))
                    for k,n in [('pos',3),('quat_xyzw',4),('lin_vel',3),('ang_vel',3),('low',3),('high',3)])
                and abs(float(np.linalg.norm(s.quat_xyzw))-1.0) < .01
                and np.all(s.high >= s.low))
    except (AttributeError, TypeError, ValueError):
        return False


def _tilt(s):
    q = np.asarray(s.quat_xyzw, dtype=float); q = q / np.linalg.norm(q)
    return math.degrees(math.acos(float(np.clip(1-2*(q[0]*q[0]+q[1]*q[1]), -1, 1))))


def _fixed_ok(objects):
    for name, spec in FIXED.items():
        state = objects.get(name)
        if not _valid_state(state) or np.max(np.abs(state.pos-np.asarray(spec.pos))) > .0005:
            return False
        actual = quat_xyzw_to_matrix(state.quat_xyzw)
        wanted = quat_xyzw_to_matrix(np.array([0., 0., math.sin(spec.yaw/2), math.cos(spec.yaw/2)]))
        if np.max(np.abs(actual-wanted)) > .001:
            return False
        if not np.allclose(state.high-state.low, np.abs(wanted) @ np.asarray(spec.size), atol=.0005, rtol=0.0):
            return False
    return True


def _nominal(objects):
    s = objects.get('soup_can')
    return (_valid_state(s) and _fixed_ok(objects) and
            np.linalg.norm(s.pos-START_POS) <= START_TOL and _tilt(s) < MAX_TILT_DEG)


def _off_table(s):
    x0,x1,y0,y1 = TABLE_BOUNDS
    return bool(s.low[0]<x0 or s.high[0]>x1 or s.low[1]<y0 or s.high[1]>y1 or s.low[2]<TABLE_Z-.010)


def _outside_drawer(s):
    # Project the world AABB conservatively onto the rotated drawer axes.
    center = ROTATION.T @ (np.asarray(s.pos)-np.r_[START_XY,TABLE_Z])
    half = np.abs(ROTATION.T) @ ((np.asarray(s.high)-np.asarray(s.low))/2)
    lo,hi = center-half,center+half
    L,W = LAYOUT['inner_length_m'], LAYOUT['inner_width_m']
    v = LAYOUT['transverse_offset_m']; t = LAYOUT['frame_thickness_m']
    return bool(lo[2] >= LAYOUT['frame_height_m']+.003 or hi[0] <= -L/2-t-.001 or
                lo[0] >= L/2+t+.001 or hi[1] <= v-W/2-t-.001 or lo[1] >= v+W/2+t+.001)


def _released(s):
    if _hands is None or len(_hands) != 2:
        return False
    for hand in _hands:
        try:
            gap = float(hand['gap']); low = np.asarray(hand['low']); high = np.asarray(hand['high'])
            if low.shape != (3,) or high.shape != (3,) or not np.all(np.isfinite(np.r_[low,high,gap])) or np.any(high<low):
                return False
            separation = np.maximum(np.asarray(s.low)-high, low-np.asarray(s.high))
            if gap < OPEN_GAP or np.linalg.norm(np.maximum(separation,0.0)) < HAND_CLEARANCE:
                return False
        except (KeyError, TypeError, ValueError):
            return False
    return True


def _hand_near_slot():
    if _hands is None:
        return True
    for hand in _hands:
        center = (np.asarray(hand['low'])+np.asarray(hand['high']))/2
        if np.linalg.norm(center[:2]-START_XY) < .12 and hand['low'][2] < TABLE_Z+.25:
            return True
    return False


def _still(s):
    if len(_history) < STILL_FRAMES:
        return s.resting(max_speed=float(.01), max_spin=float(.2))
    p = np.vstack([*_history, s.pos])
    return bool(np.max(np.linalg.norm(p[:,None,:]-p[None,:,:],axis=-1)) <= STILL_TOL)


def _view(objects):
    s = objects.get('soup_can')
    if (not _armed or _off_table_latch or not _valid_state(s) or not _fixed_ok(objects) or
            _off_table(s) or _tilt(s) >= MAX_TILT_DEG or not _lifted or not _outside_drawer(s)):
        return 0.0
    if _previous is not None and np.linalg.norm(s.pos-_previous) > RESET_JUMP:
        return 0.0
    supported = TABLE_Z-.001 <= float(s.low[2]) <= TABLE_Z+SUPPORT_TOL
    if supported and _released(s):
        if np.max(np.abs(s.pos[:2]-GOAL_XY)) <= GOAL_TOL and _still(s):
            return .75
        return .5
    return .25


def success(objects):
    global _previous, _previous_bottom, _armed, _off_table_latch, _lifted, _step, _hands, _hand_reason
    s = objects.get('soup_can'); valid = _valid_state(s)
    jumped = valid and _previous is not None and np.linalg.norm(s.pos-_previous) > RESET_JUMP
    reset = jumped and _nominal(objects)
    if reset:
        _history.clear(); _armed = False; _off_table_latch = False; _lifted = False
        _previous = None; _previous_bottom = None; _step = 0
    if _step == 0:
        _armed = bool(_nominal(objects))
    _step += 1
    _hands, _hand_reason = read_grippers()
    if valid:
        if _off_table(s):
            _off_table_latch = True
        # Witness a continuous upward crossing, not a teleport to a finished pose.
        threshold = TABLE_Z + WALL_HEIGHT + .003
        if (_armed and not jumped and _previous_bottom is not None and
                _previous_bottom < threshold <= float(s.low[2]) and _tilt(s) < MAX_TILT_DEG and _fixed_ok(objects)):
            _lifted = True
        _history.append(np.asarray(s.pos, dtype=float).copy())
        _previous = np.asarray(s.pos, dtype=float).copy()
        _previous_bottom = float(s.low[2])
    value = _view(objects)
    critical = valid and (_tilt(s)>=MAX_TILT_DEG or s.low[2]>TABLE_Z+.005 or
                         np.linalg.norm(s.pos[:2]-START_XY)>.005 or _off_table_latch or _hand_near_slot())
    if TASK_DEBUG and (_step==1 or _step%20==0 or critical):
        print('task_progress '+json.dumps(dict(step=_step, score=value, armed=_armed,
              off_table=_off_table_latch, lifted=_lifted, grippers=_hand_reason,
              released=bool(valid and _released(s)), can_pos=None if not valid else s.pos.tolist(),
              tilt_deg=None if not valid else _tilt(s),
              can_low=None if not valid else s.low.tolist(),
              can_high=None if not valid else s.high.tolist(),
              can_quat=None if not valid else s.quat_xyzw.tolist())), flush=True)
    return bool(value == .75)


def score(objects):
    return float(_view(objects))


def distance(objects):
    s = objects.get('soup_can')
    if not _valid_state(s):
        return 1.0
    return float(np.linalg.norm(s.pos-GOAL_POS))


def demo():
    x,y = floats(START_XY); gx,gy = floats(GOAL_XY)
    travel = float(TABLE_Z + TRAVEL_HEIGHT)
    pick = float(TABLE_Z + PICK_HEIGHT)
    # TCP-to-lowest-can-point = PICK_HEIGHT for a settled, upright grasp.
    tcp_to_can_bottom = float(pick-TABLE_Z)
    release = float(TABLE_Z + RELEASE_GAP + tcp_to_can_bottom)
    def w(duration, pos, finger, label):
        return Waypoint(float(duration), floats(pos), float(finger), label, yaw=float(YAW))
    return {'left': [
        w(1.5,(gx,gy,travel),FINGER_OPEN,'approach outside'),
        w(1.5,(x,y,travel),LAYOUT['entry_finger_joint_m'],'above row'),
        w(.4,(x,y,travel),LAYOUT['entry_finger_joint_m'],'align before descent'),
        w(1.5,(x,y,pick),LAYOUT['entry_finger_joint_m'],'descend'),
        w(.8,(x,y,pick),FINGER_CLOSED,'close'),
        w(1.5,(x,y,travel),FINGER_CLOSED,'extract vertically'),
        w(1.5,(gx,gy,travel),FINGER_CLOSED,'carry'),
        w(.4,(gx,gy,travel),FINGER_CLOSED,'align before placement'),
        w(1.5,(gx,gy,release),FINGER_CLOSED,'lower'),
        w(.8,(gx,gy,release),FINGER_OPEN,'release'),
        w(1.5,(gx,gy,float(TABLE_Z+.155)),FINGER_OPEN,'retreat'),
        w(1.5,(gx,gy,float(TABLE_Z+.155)),FINGER_OPEN,'settle'),
    ]}
