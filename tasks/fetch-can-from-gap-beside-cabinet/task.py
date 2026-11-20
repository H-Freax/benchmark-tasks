"""A red soup can stands between a pale cabinet side panel and a grey wall panel
on the station table, in front of the right arm. The narrow gap is open above.
A light rectangular outline marks a clear landing area beside the gap.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path

import numpy as np
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import Asset, Box, TABLE_TOP

HERE = Path(__file__).resolve().parent
CAN = json.loads((HERE / 'assets' / 'can_info.json').read_text())
P = CAN['selected']
SCENE = 'station'
TABLE_Z = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20
RUN_LIMIT = 400
CAN_SIZE = np.asarray(CAN['size_m'])
CAN_HEIGHT = float(CAN_SIZE[2])
START_XY = np.asarray(P['can_xy_m'])
GOAL_XY = np.asarray(P['goal_xy_m'])
WALL_HEIGHT = CAN_HEIGHT + P['wall_extra_m']
GAP_WIDTH = CAN['transverse_finger_width_m'] + P['gap_extra_m']
PANEL_LENGTH = P['panel_length_m']
PANEL_THICK = P['panel_thickness_m']
START_POS = np.r_[START_XY, TABLE_Z + CAN['resting_center_above_table_m'] + .001]
GOAL_POS = np.r_[GOAL_XY, TABLE_Z + CAN['resting_center_above_table_m']]
MAX_TILT_DEG = 5.0
SUPPORT_TOL = .003
GOAL_TOL = .015
STILL_TOL = .002
STILL_FRAMES = 10
START_TOL = .005
RESET_JUMP = .10
TABLE_BOUNDS = (-.19, .91, -.65, .65)
TASK_DEBUG = os.environ.get('TASK_DEBUG', '').lower() in ('1', 'true', 'yes')

OBJECTS = [Asset('soup_can', CAN['registry_name'], xy=tuple(START_XY),
                 extent=CAN['extent_m'], mass=CAN['mass_kg'])]
for name, sign, color in [('cabinet_side', -1, (.73, .66, .55)), ('wall_panel', 1, (.62, .65, .68))]:
    OBJECTS.append(Box(name, pos=(float(START_XY[0]), float(START_XY[1]+sign*(GAP_WIDTH+PANEL_THICK)/2),
                                  TABLE_Z+WALL_HEIGHT/2),
                       size=(PANEL_LENGTH, PANEL_THICK, WALL_HEIGHT), color=color))
# Four planar marking strips meet at edges without volumetric overlap.
for axis, side, name in [(0,-1,'mark_left'),(0,1,'mark_right'),(1,-1,'mark_near'),(1,1,'mark_far')]:
    xy=GOAL_XY.copy();xy[axis]+=side*.049
    size=(.002,.100,.0005) if axis==0 else (.096,.002,.0005)
    OBJECTS.append(Box(name,pos=(*map(float,xy),TABLE_Z+.00025),size=size,color=(.94,.91,.72)))
FIXED = {s.name:s for s in OBJECTS if isinstance(s,Box)}

# Only success() advances episode history. The framework calls it once per step,
# before score()/distance(), and owns the external one-second success hold.
_history = deque(maxlen=STILL_FRAMES)
_previous = None
_armed = False
_violation = None
_step = 0


def _valid_state(s):
    try:
        return all(np.asarray(getattr(s,k)).shape==(n,) and np.all(np.isfinite(getattr(s,k)))
                   for k,n in [('pos',3),('quat_xyzw',4),('lin_vel',3),('ang_vel',3),('low',3),('high',3)]) and np.linalg.norm(s.quat_xyzw)>1e-9
    except (AttributeError,TypeError,ValueError):
        return False


def _tilt(s):
    q=np.asarray(s.quat_xyzw,dtype=float);q=q/np.linalg.norm(q)
    return math.degrees(math.acos(float(np.clip(1-2*(q[0]*q[0]+q[1]*q[1]),-1,1))))


def _fixed_ok(objects):
    for name,spec in FIXED.items():
        s=objects.get(name)
        if not _valid_state(s) or np.max(np.abs(s.pos-spec.pos))>.001:
            return False
        # Fixed panel orientation changes invalidate its nominal layout too.
        q=np.asarray(s.quat_xyzw)/np.linalg.norm(s.quat_xyzw)
        if abs(q[3])<math.cos(math.radians(.5)/2):
            return False
    return True


def _nominal(objects):
    s=objects.get('soup_can')
    return _valid_state(s) and _fixed_ok(objects) and np.linalg.norm(s.pos-START_POS)<=START_TOL and _tilt(s)<MAX_TILT_DEG


def _outside_gap(s):
    # The can must be wholly outside the finite panel prism, including its top.
    lo=np.asarray(s.low);hi=np.asarray(s.high)
    return bool(lo[2]>=TABLE_Z+WALL_HEIGHT+.001 or
                hi[0]<=START_XY[0]-PANEL_LENGTH/2-.001 or lo[0]>=START_XY[0]+PANEL_LENGTH/2+.001 or
                hi[1]<=START_XY[1]-GAP_WIDTH/2-.001 or lo[1]>=START_XY[1]+GAP_WIDTH/2+.001)


def _still(s):
    if len(_history)<STILL_FRAMES:
        return s.resting(max_speed=.01,max_spin=.2)
    p=np.asarray(_history)
    return bool(max(np.max(np.linalg.norm(p[:,None,:]-p[None,:,:],axis=-1)),
                    np.max(np.linalg.norm(p-np.asarray(s.pos),axis=-1)))<=STILL_TOL)


def _view(objects):
    s=objects.get('soup_can')
    if not _armed or _violation is not None or not _valid_state(s) or not _fixed_ok(objects):
        return 0.0
    if _tilt(s)>=MAX_TILT_DEG:
        return 0.0
    # Recheck off-table, height and fixed geometry at the final frame as well as
    # remembering violations observed by success().
    if (_off_table(s) or not _outside_gap(s) or
            (_previous is not None and np.linalg.norm(s.pos-_previous)>RESET_JUMP)):
        return 0.0
    vertical_settled = (float(np.ptp(np.r_[np.asarray(_history)[:,2],s.pos[2]]))<=STILL_TOL if len(_history)==STILL_FRAMES
                        else abs(float(s.lin_vel[2]))<=.03)
    supported = TABLE_Z-.002<=float(s.low[2])<=TABLE_Z+SUPPORT_TOL and vertical_settled
    target = bool(np.max(np.abs(s.pos[:2]-GOAL_XY))<=GOAL_TOL)
    if supported and target and _still(s):
        return .75
    if supported:
        return .5
    return .25


def _off_table(s):
    x0,x1,y0,y1=TABLE_BOUNDS
    return bool(s.low[0]<x0 or s.high[0]>x1 or s.low[1]<y0 or s.high[1]>y1 or s.low[2]<TABLE_Z-.010)


def success(objects):
    global _previous,_armed,_violation,_step
    s=objects.get('soup_can')
    valid=_valid_state(s)
    jumped=valid and _previous is not None and np.linalg.norm(s.pos-_previous)>RESET_JUMP
    reset=jumped and _nominal(objects)
    if reset:
        _history.clear();_armed=False;_violation=None;_step=0
    if _step==0:
        _armed=_nominal(objects)
        if not _armed:
            _violation='initial_layout'
    _step+=1
    if not valid:
        _violation=_violation or 'invalid_state'
    else:
        if jumped and not reset:
            _violation=_violation or 'discontinuous_motion'
        if not _fixed_ok(objects):
            _violation=_violation or 'fixed_layout'
        if _tilt(s)>=MAX_TILT_DEG:
            _violation=_violation or 'tipped'
        if _off_table(s):
            _violation=_violation or 'off_table'
        _history.append(np.asarray(s.pos,dtype=float).copy())
        _previous=np.asarray(s.pos,dtype=float).copy()
    if TASK_DEBUG and (_step==1 or _step%20==0):
        print('task_progress '+json.dumps(dict(step=_step,score=_view(objects),violation=_violation,
                                               armed=_armed,history=len(_history)),sort_keys=True),flush=True)
    return _view(objects)==.75


def score(objects):
    return _view(objects)


def distance(objects):
    s=objects.get('soup_can')
    if not _valid_state(s):
        return 1.0
    return float(np.linalg.norm(s.pos-GOAL_POS))


def demo():
    x,y=map(float,START_XY);gx,gy=map(float,GOAL_XY)
    travel=TABLE_Z+WALL_HEIGHT+P['tcp_pick_above_table_m']+P['release_gap_m']
    pick=TABLE_Z+P['tcp_pick_above_table_m']
    place=pick+P['release_gap_m'];yaw=P['yaw_rad']
    return {P['arm']:[
        Waypoint(1.5,(x,y,travel),FINGER_OPEN,'hover',yaw),
        Waypoint(1.5,(x,y,pick),FINGER_OPEN,'descend',yaw),
        Waypoint(.8,(x,y,pick),FINGER_CLOSED,'close',yaw),
        Waypoint(1.5,(x,y,travel),FINGER_CLOSED,'lift',yaw),
        Waypoint(1.5,(gx,gy,travel),FINGER_CLOSED,'carry',yaw),
        Waypoint(1.5,(gx,gy,place),FINGER_CLOSED,'lower',yaw),
        Waypoint(.8,(gx,gy,place),FINGER_OPEN,'release',yaw),
        Waypoint(1.2,(gx,gy,travel),FINGER_OPEN,'retreat',yaw),
    ]}
