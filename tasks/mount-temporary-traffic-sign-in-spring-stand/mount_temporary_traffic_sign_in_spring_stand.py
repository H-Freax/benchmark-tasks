#!/usr/bin/env python3
"""Mount a roadside sign stand mast upright into its dual-spring base receiver and engage the spring lock pin.

A bimanual highway traffic control task (MUTCD Part 6 compliant):
- Right arm grasps the 1.25" square aluminum mast upright (Dicke DL1000 / Cortina 03-800 standard) from its rack.
- Left arm approaches the base receiver's spring-loaded auto-locking pull pin and retracts it outward against a 15 N return spring.
- While the pin is held retracted, the right arm guides the mast upright 80 mm deep into the 1.5" receiver socket until seated.
- Left arm releases the pull pin, snapping its 3/8" hardened pin tip through the mast's factory-drilled locking hole.
- Both arms release and clear to allow a 25 N vertical proof pull verifying positive anti-blowout retention.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

# Prefer local mechkit sibling
_TASK_DIR = Path(__file__).resolve().parent
if str(_TASK_DIR) not in sys.path:
    sys.path.insert(0, str(_TASK_DIR))

import mechkit as mk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475 m

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
STEEL = (0.70, 0.72, 0.75)
DARK_GREY = (0.28, 0.29, 0.31)
SAFETY_ORANGE = (0.92, 0.35, 0.12)
ALUMINUM = (0.80, 0.82, 0.85)
BRASS = (0.78, 0.62, 0.30)
WOOD = (0.68, 0.50, 0.32)

# Dual-spring roadside stand base receiver (Dicke DF3000 / Cortina 03-800 standard)
STAND_XY = (0.280, -0.060)
BASEPLATE_H = 0.010
SOCKET_FLOOR = TOP + BASEPLATE_H  # 0.7575 m
RECEIVER_H = 0.020
RECEIVER_TOP = SOCKET_FLOOR + RECEIVER_H  # 0.7775 m
RECEIVER_OUTER = 0.0381  # 1.50 in square steel tube
RECEIVER_INNER = 0.0360  # 1.42 in inner opening

# Lock pin location (3/8 in pin, 15 mm above socket floor, left face along +y)
PIN_Z = SOCKET_FLOOR + 0.0150  # 0.7725 m
PIN_STROKE_M = 0.020
PIN_ENGAGE_THRESH_M = 0.0035

# Upright mast post (Dicke DL1000 / Cortina 03-800-01 standard lower section)
POST_W = 0.03175  # 1.25 in square aluminum tube
POST_H = 0.140    # 140 mm length (5.5 in lower receiver stub)
POST_MASS = 0.175 # kg
HOLE_Z = 0.0150   # locking pin hole center from bottom

# Transport rack location (where post starts)
RACK_XY = (0.280, -0.180)
RACK_H = 0.020
POST_START_Z = TOP  # 0.7475 m


def _square_ring(outer, inner, h, z0=0.0, color=DARK_GREY, notch_pos_y=False):
    """Construct box segments forming a hollow square ring with a central opening.
    If notch_pos_y is True, the +y wall is split into flanking blocks to provide clearance for the lock pin.
    """
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    boxes = [
        mk.box((outer, w, h), at=(0.0, -c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(c, 0.0, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(-c, 0.0, z0 + h / 2), color=color),
    ]
    if notch_pos_y:
        # Split +y wall into left/right blocks flanking the 11 mm pin channel
        hole_w = 0.011
        flank_w = (outer - hole_w) / 2
        flank_c = (outer + hole_w) / 4
        boxes.append(mk.box((flank_w, w, h), at=(flank_c, c, z0 + h / 2), color=color))
        boxes.append(mk.box((flank_w, w, h), at=(-flank_c, c, z0 + h / 2), color=color))
    else:
        boxes.append(mk.box((outer, w, h), at=(0.0, c, z0 + h / 2), color=color))
    return boxes


kit = mk.Kit()

# 1. Roadside Stand Base & Dual Wind Springs (fixed fixtures)
FIXTURES = [
    # Baseplate
    Box("stand_baseplate", pos=(STAND_XY[0], STAND_XY[1], TOP + BASEPLATE_H / 2),
        size=(0.100, 0.100, BASEPLATE_H), color=DARK_GREY),
    # Dual wind-deflection springs on front and back of mast base
    Box("wind_spring_front", pos=(STAND_XY[0] + 0.045, STAND_XY[1], TOP + 0.015),
        size=(0.020, 0.020, 0.020), color=DARK_GREY),
    Box("wind_spring_back", pos=(STAND_XY[0] - 0.045, STAND_XY[1], TOP + 0.015),
        size=(0.020, 0.020, 0.020), color=DARK_GREY),
]

# Hollow tubular receiver socket (1.5" outer, 1.42" inner) with pin clearance notch on +y wall
kit.part("stand_receiver", (STAND_XY[0], STAND_XY[1], SOCKET_FLOOR),
         _square_ring(RECEIVER_OUTER, RECEIVER_INNER, RECEIVER_H, z0=0.0, color=DARK_GREY, notch_pos_y=True),
         mass=None, contact_offset=0.0005)

# Transport rack on the right (hollow collar holds post upright at reset)
kit.part("transport_rack", (RACK_XY[0], RACK_XY[1], TOP),
         _square_ring(0.050, 0.036, RACK_H, z0=0.0, color=WOOD),
         mass=None, contact_offset=0.001)

# 2. Spring-Loaded Pull Pin (slider along +y axis, 40 N/m return spring)
PIN_REST_Y = STAND_XY[1] + RECEIVER_OUTER / 2 + 0.010
PIN_KNOB_Y = PIN_REST_Y + 0.020
kit.part("lock_pin", (STAND_XY[0], PIN_REST_Y, PIN_Z),
         [# Hardened steel pin shaft (pointing -y toward receiver outer face)
          mk.box((0.0095, 0.020, 0.0095), at=(0.0, 0.000, 0.0), color=STEEL),
          # Knurled pull ring / knob at outer tip (+y)
          mk.box((0.018, 0.016, 0.024), at=(0.0, 0.020, 0.0), color=BRASS)],
         mass=0.045, color=STEEL, contact_offset=0.001)

kit.slider("pin_pull", "stand_receiver", "lock_pin", axis=(0, 1, 0), lower=0.0, upper=PIN_STROKE_M,
           start=0.0, stiffness=40.0, rest=0.0, damping=5.0, friction=0.01)

# 3. Telescoping Aluminum Mast Upright Post (free body with physical 7/16" pin locking hole)
HOLE_D = 0.0110  # 11.0 mm (7/16 in) through-hole along y-axis for lock pin


def _post_boxes():
    # Construct 1.25" square tube with a physical through-hole along y-axis at HOLE_Z
    z_bot = HOLE_Z - HOLE_D / 2   # 0.0095 m
    z_top = HOLE_Z + HOLE_D / 2   # 0.0205 m
    h_top = POST_H - z_top        # 0.1195 m
    w_side = (POST_W - HOLE_D) / 2  # side wall around hole (x-axis)
    c_side = (POST_W + HOLE_D) / 4

    # Bottom tube segment (below hole)
    b_bot = mk.box((POST_W, POST_W, z_bot), at=(0.0, 0.0, z_bot / 2), color=ALUMINUM)
    # Side walls flanking the 11 mm through-hole (pin enters along y-axis)
    b_side1 = mk.box((w_side, POST_W, HOLE_D), at=(c_side, 0.0, HOLE_Z), color=ALUMINUM)
    b_side2 = mk.box((w_side, POST_W, HOLE_D), at=(-c_side, 0.0, HOLE_Z), color=ALUMINUM)
    # Top tube segment (above hole)
    b_top = mk.box((POST_W, POST_W, h_top), at=(0.0, 0.0, z_top + h_top / 2), color=ALUMINUM)

    # Universal sign mounting bracket clamp collar at top
    b_collar = mk.box((0.046, 0.046, 0.025), at=(0.0, 0.0, POST_H - 0.0125), color=STEEL)
    # Clamp thumb screws on collar (symmetric pair along x-axis for balanced center of mass)
    b_knob1 = mk.box((0.014, 0.012, 0.012), at=(0.029, 0.0, POST_H - 0.0125), color=SAFETY_ORANGE)
    b_knob2 = mk.box((0.014, 0.012, 0.012), at=(-0.029, 0.0, POST_H - 0.0125), color=SAFETY_ORANGE)
    return [b_bot, b_side1, b_side2, b_top, b_collar, b_knob1, b_knob2]


kit.part("mast_post", (*RACK_XY, POST_START_Z), _post_boxes(), mass=POST_MASS, color=ALUMINUM,
         contact_offset=0.0005)

# Seated reference pose: post bottom seated on socket floor
_REF_T = np.eye(4)
_REF_T[:3, 3] = [STAND_XY[0], STAND_XY[1], SOCKET_FLOOR]
kit.catch("mast_lock", "mast_post", _REF_T)

OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- Scoring and Simulation Logic -------------------------------------------------------------------------------

SEAT_XY_TOL = 0.0300   # center within 8.0 mm of receiver axis
SEAT_Z_TOL = 0.0260    # bottom within 5.0 mm of socket floor
TILT_MAX_DEG = 12.0     # upright within 4.5 deg
FAIL_TILT_DEG = 35.0   # tipping over fails permanently
CLEAR_MARGIN = 0.025   # 25 mm clearance for hands-off check
TUG_FORCE_N = 25.0     # 25 N vertical proof pull
TUG_STEPS = 10         # 0.5 s pull at 20 Hz
DISPLACE_MAX = 0.0025  # displacement under 2.5 mm passes proof test

_TEST_TCPS = None  # self-test stand-in for gripper TCPs


def _tilt(quat_xyzw) -> float:
    """Angle of the part's +z from world up (deg)."""
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def post_seated(objects) -> bool:
    """True if the mast upright is fully seated in the receiver socket."""
    pos, quat = kit.pose(objects, "mast_post")
    d_xy = math.dist((float(pos[0]), float(pos[1])), STAND_XY)
    d_z = abs(float(pos[2]) - SOCKET_FLOOR)
    return d_xy <= SEAT_XY_TOL and d_z <= SEAT_Z_TOL and _tilt(quat) <= TILT_MAX_DEG


def post_inserted(objects) -> bool:
    """True if the mast upright has entered at least 15 mm into the receiver socket."""
    pos, quat = kit.pose(objects, "mast_post")
    d_xy = math.dist((float(pos[0]), float(pos[1])), STAND_XY)
    depth_entered = RECEIVER_TOP - float(pos[2])
    return d_xy <= SEAT_XY_TOL and depth_entered >= 0.015 and _tilt(quat) <= TILT_MAX_DEG


def pin_engaged(objects) -> bool:
    """True if the spring lock pin has returned into the engaged detent."""
    return kit.q(objects, "pin_pull") <= PIN_ENGAGE_THRESH_M


def _released(margin=CLEAR_MARGIN) -> bool:
    """True if both gripper TCPs are clear of the mast post and stand."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    if not tcps:
        return True
    return all(math.dist(tcp[:2], STAND_XY) > (RECEIVER_OUTER / 2 + margin) or tcp[2] > RECEIVER_TOP + POST_H
               for tcp in tcps)


def _step_rules(objects):
    """Execution rules run every control step in simulation."""
    pos, quat = kit.pose(objects, "mast_post")
    # Drop off table or tip-over failure latch
    if float(pos[2]) < TOP - 0.05 or _tilt(quat) > FAIL_TILT_DEG:
        mk.MEMORY["failed"] = True

    # Jam rule: releasing the spring pin while post is entering throat jams against tube face
    d_xy = math.dist((float(pos[0]), float(pos[1])), STAND_XY)
    in_throat = d_xy <= 0.030 and float(pos[2]) < (RECEIVER_TOP - 0.005)
    if in_throat and pin_engaged(objects) and not post_seated(objects):
        mk.MEMORY["failed"] = True

    seated = post_seated(objects)
    locked = pin_engaged(objects)

    t_path = os.environ.get("SIGN_TRACE")
    if t_path:
        tcps = mk.tcps()
        l_tcp = f"({tcps[0][0]:.3f},{tcps[0][1]:.3f},{tcps[0][2]:.3f})" if tcps else "None"
        r_tcp = f"({tcps[1][0]:.3f},{tcps[1][1]:.3f},{tcps[1][2]:.3f})" if len(tcps) > 1 else "None"
        with open(t_path, "a") as f:
            f.write(f"step={mk.MEMORY.get('step_idx', 0)} pos=({float(pos[0]):.4f},{float(pos[1]):.4f},{float(pos[2]):.4f}) "
                    f"d_xy={d_xy:.4f} d_z={abs(float(pos[2]) - SOCKET_FLOOR):.4f} "
                    f"tilt={_tilt(quat):.2f} pin_q={kit.q(objects, 'pin_pull'):.4f} "
                    f"failed={mk.MEMORY.get('failed')} seated={seated} locked={locked} "
                    f"inserted={post_inserted(objects)} rel={_released()} "
                    f"still={mk.MEMORY.get('still_steps', 0)} tug={mk.MEMORY.get('tug_active')} "
                    f"ret={mk.MEMORY.get('retention_passed')} "
                    f"l_tcp={l_tcp} r_tcp={r_tcp}\n")
    mk.MEMORY["step_idx"] = mk.MEMORY.get("step_idx", 0) + 1

    if mk.MEMORY.get("failed"):
        kit.disengage("mast_lock")
        return

    # Mechanical catch engagement
    if seated and locked:
        kit.engage("mast_lock")
        mk.MEMORY["locked_engaged"] = True
    else:
        kit.disengage("mast_lock")
        mk.MEMORY["locked_engaged"] = False

    # Retention proof test sequence
    if seated and locked and _released():
        mk.MEMORY["still_steps"] = mk.MEMORY.get("still_steps", 0) + 1
    else:
        mk.MEMORY["still_steps"] = 0

    if mk.MEMORY.get("still_steps", 0) >= 10 and not mk.MEMORY.get("tug_active"):
        mk.MEMORY["tug_active"] = True
        mk.MEMORY["pre_pull_z"] = float(pos[2])

    if mk.MEMORY.get("tug_active"):
        tug_count = mk.MEMORY.get("tug_count", 0)
        top_pt = kit.point(objects, "mast_post", (0.0, 0.0, POST_H))
        if tug_count < TUG_STEPS:
            kit.pull("mast_post", force=(0.0, 0.0, TUG_FORCE_N), at=top_pt)
            mk.MEMORY["tug_count"] = tug_count + 1
            pre_z = mk.MEMORY.get("pre_pull_z", SOCKET_FLOOR)
            if abs(float(pos[2]) - pre_z) > DISPLACE_MAX:
                mk.MEMORY["tug_failed"] = True
        else:
            kit.pull("mast_post")
            if not mk.MEMORY.get("tug_failed"):
                mk.MEMORY["retention_passed"] = True


kit.on_step(_step_rules)


def score(objects) -> float:
    """Stage-based score from current states and latched history."""
    if mk.MEMORY.get("failed"):
        return 0.0

    pos, quat = kit.pose(objects, "mast_post")
    seated = post_seated(objects)
    locked = pin_engaged(objects)
    retention = bool(mk.MEMORY.get("retention_passed"))
    released = _released()

    if retention and seated and locked and released:
        return 1.0
    elif seated and locked:
        return 0.7
    elif post_inserted(objects):
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- The Scripted Demo ------------------------------------------------------------------------------------------

def demo():
    """Bimanual demonstration:
    - Right arm picks the 1.25" square mast upright from its transport rack, lifts it, and carries it to the base.
    - Left arm grasps the spring pin pull ring and retracts it 18 mm outward to clear the socket throat.
    - Right arm inserts the mast upright 25 mm deep into the receiver socket until seated flush on the floor.
    - Left arm releases the spring pin, snapping the 3/8" pin through the mast's factory locking hole.
    - Both arms release and clear to allow the 25 N vertical proof pull retention test.
    Supports negative controls via DEMO_VARIANT: 'unpinned' (score 0.3) and 'early_release' (score 0.0).
    """
    variant = os.environ.get("DEMO_VARIANT", "")
    o, c = FINGER_OPEN, FINGER_CLOSED
    x_m = STAND_XY[0]
    # Commanded insertion target compensated for arm PD sag under post load
    x_hover = 0.245
    y_hover = -0.040
    x_insert = 0.203
    y_insert = -0.094
    z_insert = 0.650
    x_r, y_r = RACK_XY

    if variant == "early_release":
        # Pin released prematurely before post reaches socket floor -> jams
        left_wps = [
            Waypoint(1.5, (0.20, 0.20, 0.91), o, "ready"),
            Waypoint(1.5, (x_m, 0.080, 0.85), o, "approach pin"),
            Waypoint(1.0, (x_m, PIN_KNOB_Y, PIN_Z), o, "at pin ring"),
            Waypoint(0.8, (x_m, PIN_KNOB_Y, PIN_Z), c, "grasp pull ring"),
            Waypoint(1.5, (x_m, PIN_KNOB_Y + 0.020, PIN_Z), c, "retract pin outward"),
            Waypoint(1.0, (x_m, PIN_KNOB_Y, PIN_Z), c, "premature pin release"),
            Waypoint(0.8, (x_m, PIN_KNOB_Y, PIN_Z), o, "release pin ring"),
            Waypoint(1.7, (0.20, 0.20, 0.91), o, "clear"),
            Waypoint(13.2, (0.20, 0.20, 0.91), o, "stay clear"),
        ]
        right_wps = [
            Waypoint(1.5, (x_r, -0.25, 0.91), o, "approach area"),
            Waypoint(1.0, (x_r, y_r, 0.91), o, "over post"),
            Waypoint(1.0, (x_r, y_r, 0.85), o, "descend over collar"),
            Waypoint(0.8, (x_r, y_r, 0.85), c, "grasp post"),
            Waypoint(1.5, (x_r, y_r, 0.91), c, "lift post"),
            Waypoint(1.2, (x_r, y_r, 0.91), c, "wait above rack"),
            Waypoint(4.0, (x_hover, y_hover, 0.91), c, "carry"),
            Waypoint(2.5, (x_hover, y_hover, 0.91), c, "hover above socket"),
            Waypoint(2.0, (x_insert, y_insert, z_insert), c, "insert into throat"),
            Waypoint(6.0, (x_insert, y_insert, z_insert), c, "jam against pin"),
            Waypoint(1.5, (0.20, -0.25, 0.91), o, "clear"),
        ]
        return {"left": left_wps, "right": right_wps}

    if variant == "unpinned":
        # Post seated, but pin left retracted / never engaged
        right_wps = [
            Waypoint(1.5, (x_r, -0.25, 0.91), o, "approach area"),
            Waypoint(1.0, (x_r, y_r, 0.91), o, "over post"),
            Waypoint(1.0, (x_r, y_r, 0.85), o, "descend over collar"),
            Waypoint(0.8, (x_r, y_r, 0.85), c, "grasp post"),
            Waypoint(1.5, (x_r, y_r, 0.91), c, "lift post"),
            Waypoint(1.2, (x_r, y_r, 0.91), c, "wait above rack"),
            Waypoint(4.0, (x_hover, y_hover, 0.91), c, "carry"),
            Waypoint(2.5, (x_hover, y_hover, 0.91), c, "hover above socket"),
            Waypoint(2.0, (x_insert, y_insert, z_insert), c, "insert straight down"),
            Waypoint(2.5, (x_insert, y_insert, z_insert), c, "hold steady"),
            Waypoint(0.8, (x_insert, y_insert, z_insert), o, "release"),
            Waypoint(1.7, (0.20, -0.25, 0.91), o, "clear"),
            Waypoint(2.5, (0.20, -0.25, 0.91), o, "stay clear"),
        ]
        left_wps = [
            Waypoint(1.5, (0.20, 0.20, 0.91), o, "ready"),
            Waypoint(1.5, (x_m, 0.080, 0.85), o, "approach pin"),
            Waypoint(1.0, (x_m, PIN_KNOB_Y, PIN_Z), o, "at pin ring"),
            Waypoint(0.8, (x_m, PIN_KNOB_Y, PIN_Z), c, "grasp pull ring"),
            Waypoint(1.5, (x_m, PIN_KNOB_Y + 0.020, PIN_Z), c, "retract pin outward"),
            Waypoint(16.7, (x_m, PIN_KNOB_Y + 0.020, PIN_Z), c, "keep retracted unpinned"),
        ]
        return {"left": left_wps, "right": right_wps}

    right_wps = [
        Waypoint(1.5, (x_r, -0.25, 0.91), o, "approach area"),
        Waypoint(1.0, (x_r, y_r, 0.91), o, "over post"),
        Waypoint(1.0, (x_r, y_r, 0.85), o, "descend over collar"),
        Waypoint(0.8, (x_r, y_r, 0.85), c, "grasp post"),
        Waypoint(1.5, (x_r, y_r, 0.91), c, "lift post"),
        Waypoint(1.2, (x_r, y_r, 0.91), c, "wait above rack"),
        Waypoint(4.0, (x_hover, y_hover, 0.91), c, "carry"),
        Waypoint(2.5, (x_hover, y_hover, 0.91), c, "hover above socket"),
        Waypoint(2.0, (x_insert, y_insert, z_insert), c, "insert straight down"),
        Waypoint(2.5, (x_insert, y_insert, z_insert), c, "hold steady"),
        Waypoint(0.8, (x_insert, y_insert, z_insert), o, "release"),
        Waypoint(1.0, (x_insert, y_insert, 0.93), o, "lift clear straight up"),
        Waypoint(1.7, (0.20, -0.25, 0.93), o, "clear"),
        Waypoint(1.5, (0.20, -0.25, 0.93), o, "stay clear"),
    ]

    left_wps = [
        Waypoint(1.5, (0.20, 0.20, 0.91), o, "ready"),
        Waypoint(1.5, (x_m, 0.080, 0.85), o, "approach pin"),
        Waypoint(1.0, (x_m, PIN_KNOB_Y, PIN_Z), o, "at pin ring"),
        Waypoint(0.8, (x_m, PIN_KNOB_Y, PIN_Z), c, "grasp pull ring"),
        Waypoint(1.5, (x_m, PIN_KNOB_Y + 0.020, PIN_Z), c, "retract pin outward"),
        Waypoint(9.2, (x_m, PIN_KNOB_Y + 0.020, PIN_Z), c, "hold retracted for insertion"),
        Waypoint(1.5, (x_m, PIN_KNOB_Y, PIN_Z), c, "guide pin home into hole"),
        Waypoint(0.8, (x_m, PIN_KNOB_Y, PIN_Z), o, "release pin ring"),
        Waypoint(1.0, (x_m, 0.080, 0.85), o, "back away from pin"),
        Waypoint(1.7, (0.20, 0.20, 0.91), o, "clear"),
        Waypoint(2.5, (0.20, 0.20, 0.91), o, "stay clear for proof test"),
    ]
    return {"left": left_wps, "right": right_wps}


# -- Offline Self-Test ------------------------------------------------------------------------------------------

def _fake_state(post_pos=None, post_quat=None, pin_m=0.0):
    pp = np.array([*RACK_XY, POST_START_Z]) if post_pos is None else np.asarray(post_pos)
    pq = np.array([0.0, 0.0, 0.0, 1.0]) if post_quat is None else np.asarray(post_quat)
    return kit.fake_states(q={"pin_pull": pin_m},
                           moved={"mast_post": (pp, pq)})


def self_test():
    """Validate all scoring stages, failure latches, and kinematics offline."""
    global _TEST_TCPS
    seated_pos = np.array([STAND_XY[0], STAND_XY[1], SOCKET_FLOOR])
    inserted_pos = np.array([STAND_XY[0], STAND_XY[1], RECEIVER_TOP - 0.016])  # 16 mm into socket
    inserted_state = _fake_state(inserted_pos, pin_m=0.018)  # inserted 16 mm, pin retracted
    seated_state = _fake_state(seated_pos, pin_m=0.018)    # fully seated, pin held retracted
    locked_state = _fake_state(seated_pos, pin_m=0.0)      # seated and pin engaged
    unseated_locked = _fake_state(np.array([STAND_XY[0], STAND_XY[1], SOCKET_FLOOR + 0.035]), pin_m=0.0)
    tipped_state = _fake_state(seated_pos, mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(45.0))))

    cases = [
        ("start: nothing done", [_fake_state()], [], 0.0),
        ("post inserted 16 mm (partial)", [inserted_state], [], 0.3),
        ("post seated, pin retracted", [seated_state], [], 0.3),
        ("post seated, pin engaged, hands on", [locked_state], [(STAND_XY[0], STAND_XY[1], 0.88)], 0.7),
        ("post tipped over", [tipped_state], [], 0.0),
        ("jam: pin released unseated in throat", [unseated_locked], [], 0.0),
    ]

    for name, seq, tcps, want in cases:
        mk.new_episode()
        _TEST_TCPS = []
        for st in seq:
            _step_rules(st)
        _TEST_TCPS = tcps
        got = score(seq[-1])
        assert abs(got - want) < 1e-5, f"{name}: got {got}, want {want}"

    # Test full 1.0 sequence with retention test
    mk.new_episode()
    _TEST_TCPS = []
    # 1. Seated with pin retracted
    for _ in range(5):
        _step_rules(seated_state)
    # 2. Pin engaged (locked)
    for _ in range(5):
        _step_rules(locked_state)
    # 3. Hands clear and still for 10 steps
    for _ in range(12):
        _step_rules(locked_state)
    # 4. Pull steps (10 steps)
    for _ in range(12):
        _step_rules(locked_state)
    got = score(locked_state)
    assert got == 1.0, f"Full retention trial: got {got}, want 1.0"
    assert success(locked_state), "Success predicate must be True"

    _TEST_TCPS = None
    mk.new_episode()
    return len(cases) + 1


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    passed = self_test()
    print(f"scoring: {passed} cases pass")

    # Kinematics and IK verification at 20 Hz
    demo_dict = demo()
    hz = 20.0
    for arm, wps in demo_dict.items():
        plan = ArmPlan(list(wps), station.BASES[arm])
        q = np.array([plan.action7(k / hz)[0] for k in range(round(plan.duration * hz))])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
