#!/usr/bin/env python3
"""Mount an automotive brake hose banjo union with copper crush washers and thread hollow bolt into caliper bore.

An automotive brake service procedure (DOT / SAE J1401 / DIN 7603 compliant):
- Brake caliper casting is mounted rigidly on an inspection fixture bench.
- Right arm grasps an M10 hollow hex-head banjo bolt pre-fitted with the upper copper crush washer.
- Left arm grasps the reinforced flexible brake hose collar (banjo fitting eyelet block, 10 mm bore, 20 mm OD, 10 mm thickness).
- Left arm positions and holds the banjo eyelet block aligned concentric over the caliper's threaded fluid inlet bore (resting on the lower copper crush washer).
- Right arm guides the banjo bolt downward through the banjo eyelet and into the threaded fluid port.
- Right arm executes clockwise rotation, threading the M10 bolt securely home against the upper crush washer.
- Left arm applies a 10 N axial proof pull along the brake hose line to verify mechanical coupling and zero slip.
- Both arms release and clear to confirm stable, fluid-tight mounting without tool or hand obstruction.
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

# Appearance colors
CAST_IRON = (0.35, 0.36, 0.38)
COPPER = (0.75, 0.45, 0.25)
STEEL = (0.70, 0.72, 0.75)
DARK_RUBBER = (0.15, 0.15, 0.16)
SILVER_METAL = (0.82, 0.84, 0.86)
FIXTURE_BLUE = (0.20, 0.35, 0.55)

# Bench & Caliper Fixture Geometry
# Tabletop: z = 0.7475 m
# Caliper mounting fixture plate: centered at (0.280, 0.000)
CALIPER_XY = (0.280, 0.000)
FIXTURE_BASE_H = 0.025
CALIPER_BASE_Z = TOP + FIXTURE_BASE_H  # 0.7725 m

# Caliper Fluid Port:
# Machined inlet boss on the caliper body.
PORT_BOSS_H = 0.040
PORT_BOSS_TOP = CALIPER_BASE_Z + PORT_BOSS_H  # 0.8125 m
PORT_XY = (0.280, 0.000)

# Lower Copper Crush Washer (pre-seated on the caliper port boss):
# DIN 7603A / SAE standard copper sealing washer: 10.2 mm ID, 15.0 mm OD, 1.5 mm thickness.
WASHER_THICKNESS = 0.0015
LOWER_WASHER_Z = PORT_BOSS_TOP + WASHER_THICKNESS / 2  # 0.81325 m

# Banjo Eyelet Fitting Block (SAE J1401 standard brake hose terminal):
# 20 mm OD circular block with 10.2 mm inner bore, 10 mm axial height (thickness).
BANJO_H = 0.010
BANJO_OD = 0.020
BANJO_ID = 0.0102
# In assembled state, banjo rests on lower washer:
BANJO_SEATED_Z = PORT_BOSS_TOP + WASHER_THICKNESS + BANJO_H / 2  # 0.8190 m

# Banjo initial pickup position (staged on a fixture support block to the left, along +y):
BANJO_INIT_XY = (0.280, 0.090)
BANJO_INIT_Z = TOP + 0.025  # 0.7725 m

# M10 Banjo Bolt:
BOLT_INIT_XY = (0.280, -0.090)
BOLT_INIT_Z = TOP + 0.025  # 0.7725 m
BOLT_SEATED_Z = 0.8295

# Robust Tolerances (conforming with standard benchmark task norms e.g. PR #3137)
BANJO_POS_XY_TOL = 0.035
BANJO_POS_Z_TOL = 0.030
BANJO_SEAT_XY_TOL = 0.018
BANJO_SEAT_Z_TOL = 0.015
BOLT_IN_XY_TOL = 0.020
BOLT_IN_Z_TOL = 0.030
BOLT_SEAT_XY_TOL = 0.010
BOLT_SEAT_Z_TOL = 0.006
TILT_MAX_DEG = 35.0
FAIL_TILT_DEG = 60.0
CLEAR_MARGIN = 0.025

# Physical retention proof pull settings
TUG_FORCE_N = 10.0
TUG_STEPS = 10
DISPLACE_MAX = 0.006

kit = mk.Kit()

# -----------------------------------------------------------------------------
# 1. Fixed Fixtures & Caliper Body
# -----------------------------------------------------------------------------
FIXTURES = [
    # Heavy steel mounting fixture plate bolted to table
    Box("fixture_plate", pos=(CALIPER_XY[0], CALIPER_XY[1], TOP + FIXTURE_BASE_H / 2),
        size=(0.140, 0.160, FIXTURE_BASE_H), color=FIXTURE_BLUE),
    # Brake caliper main casting body (rugged automotive iron casting)
    Box("caliper_body_main", pos=(CALIPER_XY[0] - 0.025, CALIPER_XY[1], CALIPER_BASE_Z + 0.025),
        size=(0.080, 0.120, 0.050), color=CAST_IRON),
    # Caliper piston cylinder housing
    Box("caliper_piston_housing", pos=(CALIPER_XY[0] + 0.025, CALIPER_XY[1], CALIPER_BASE_Z + 0.020),
        size=(0.050, 0.090, 0.040), color=CAST_IRON),
    # Caliper fluid inlet port boss (concentric with M10 threaded bore)
    Box("caliper_inlet_boss", pos=(PORT_XY[0], PORT_XY[1], CALIPER_BASE_Z + PORT_BOSS_H / 2),
        size=(0.024, 0.024, PORT_BOSS_H), color=SILVER_METAL),
    # Lower copper crush washer pre-seated on the caliper inlet boss
    Box("caliper_lower_washer", pos=(PORT_XY[0], PORT_XY[1], LOWER_WASHER_Z),
        size=(0.015, 0.015, WASHER_THICKNESS), color=COPPER),
    # Banjo fitting staging cradle (left side, supports hose fitting at ready height)
    Box("cradle_banjo_support", pos=(BANJO_INIT_XY[0], BANJO_INIT_XY[1], TOP + 0.010),
        size=(0.035, 0.040, 0.020), color=FIXTURE_BLUE),
    # Bolt caddy cup (right side, holds upright banjo bolt ready for grasp)
    Box("bolt_caddy", pos=(BOLT_INIT_XY[0], BOLT_INIT_XY[1], TOP + 0.010),
        size=(0.035, 0.035, 0.020), color=FIXTURE_BLUE),
]

# -----------------------------------------------------------------------------
# 2. Dynamic Movable Parts
# -----------------------------------------------------------------------------

def _make_banjo_boxes():
    w = 0.005
    od = 0.020
    h = 0.010
    boxes = [
        # Eyelet ring walls
        mk.box((od, w, h), at=(0.0, 0.0075, 0.0), color=STEEL),
        mk.box((od, w, h), at=(0.0, -0.0075, 0.0), color=STEEL),
        mk.box((w, 0.010, h), at=(0.0075, 0.0, 0.0), color=STEEL),
        mk.box((w, 0.010, h), at=(-0.0075, 0.0, 0.0), color=STEEL),
        # Crimped collar extending in +y direction (grasp handle for left gripper)
        mk.box((0.014, 0.025, 0.012), at=(0.0, 0.0225, 0.0), color=SILVER_METAL),
        # Reinforced flexible hydraulic brake hose extending further in +y
        mk.box((0.010, 0.050, 0.010), at=(0.0, 0.060, 0.0), color=DARK_RUBBER),
    ]
    return boxes

kit.part(
    "banjo_hose",
    pos=(BANJO_INIT_XY[0], BANJO_INIT_XY[1], BANJO_INIT_Z),
    boxes=_make_banjo_boxes(),
    mass=0.085,
    contact_offset=0.0005,
)

def _make_bolt_boxes():
    boxes = [
        # Hex head (14 mm across flats, 8 mm height) - grasped by right gripper
        mk.box((0.014, 0.014, 0.008), at=(0.0, 0.0, 0.010), color=STEEL),
        # Upper copper crush washer captive under bolt flange (15 mm OD, 1.5 mm thick)
        mk.box((0.015, 0.015, 0.0015), at=(0.0, 0.0, 0.00525), color=COPPER),
        # Hollow shank (M10 OD, 18 mm length) extending down through banjo into caliper
        mk.box((0.0098, 0.0098, 0.018), at=(0.0, 0.0, -0.0045), color=SILVER_METAL),
    ]
    return boxes

kit.part(
    "banjo_bolt",
    pos=(BOLT_INIT_XY[0], BOLT_INIT_XY[1], BOLT_INIT_Z),
    boxes=_make_bolt_boxes(),
    mass=0.045,
    contact_offset=0.0005,
)

# Reference 4x4 matrix for bolt seated in caliper port
_BOLT_REF_T = np.eye(4)
_BOLT_REF_T[:3, 3] = [PORT_XY[0], PORT_XY[1], BOLT_SEATED_Z]
kit.catch("thread_lock", "banjo_bolt", _BOLT_REF_T)

OBJECTS = [*FIXTURES, *kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_TEST_TCPS = None


# -----------------------------------------------------------------------------
# Geometric Helpers & Step Rules
# -----------------------------------------------------------------------------
def _tilt(quat) -> float:
    """Tilt angle from vertical Z axis in degrees."""
    x, y, z, w = quat
    z_axis_z = 1.0 - 2.0 * (x * x + y * y)
    z_axis_z = max(-1.0, min(1.0, float(z_axis_z)))
    return math.degrees(math.acos(z_axis_z))


def banjo_positioned(objects) -> bool:
    pos, quat = kit.pose(objects, "banjo_hose")
    d_xy = math.dist((float(pos[0]), float(pos[1])), PORT_XY)
    d_z = abs(float(pos[2]) - BANJO_SEATED_Z)
    return d_xy <= BANJO_POS_XY_TOL and d_z <= BANJO_POS_Z_TOL and _tilt(quat) <= TILT_MAX_DEG


def banjo_seated(objects) -> bool:
    pos, quat = kit.pose(objects, "banjo_hose")
    d_xy = math.dist((float(pos[0]), float(pos[1])), PORT_XY)
    d_z = abs(float(pos[2]) - BANJO_SEATED_Z)
    return d_xy <= BANJO_SEAT_XY_TOL and d_z <= BANJO_SEAT_Z_TOL and _tilt(quat) <= TILT_MAX_DEG


def bolt_inserted(objects) -> bool:
    pos, quat = kit.pose(objects, "banjo_bolt")
    d_xy = math.dist((float(pos[0]), float(pos[1])), PORT_XY)
    d_z = abs(float(pos[2]) - BOLT_SEATED_Z)
    return d_xy <= BOLT_IN_XY_TOL and d_z <= BOLT_IN_Z_TOL and _tilt(quat) <= TILT_MAX_DEG


def bolt_threaded(objects) -> bool:
    pos, quat = kit.pose(objects, "banjo_bolt")
    d_xy = math.dist((float(pos[0]), float(pos[1])), PORT_XY)
    d_z = abs(float(pos[2]) - BOLT_SEATED_Z)
    return d_xy <= BOLT_SEAT_XY_TOL and d_z <= BOLT_SEAT_Z_TOL and _tilt(quat) <= TILT_MAX_DEG


def _released(margin=CLEAR_MARGIN) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    if not tcps:
        return True
    return all(math.dist(tcp[:2], PORT_XY) > (0.025 + margin) or tcp[2] > (BOLT_SEATED_Z + 0.050)
               for tcp in tcps)


def _step_rules(objects):
    """Step execution rules run every control step in simulation."""
    b_pos, b_quat = kit.pose(objects, "banjo_hose")
    bolt_pos, bolt_quat = kit.pose(objects, "banjo_bolt")

    # Drop / severe tilt failure latch
    if float(b_pos[2]) < TOP - 0.05 or float(bolt_pos[2]) < TOP - 0.05:
        mk.MEMORY["failed"] = True
    if _tilt(b_quat) > FAIL_TILT_DEG or _tilt(bolt_quat) > FAIL_TILT_DEG:
        mk.MEMORY["failed"] = True

    b_ok = banjo_seated(objects)
    bolt_ok = bolt_threaded(objects)

    # Thread latching when bolt reaches seated position over seated banjo
    if b_ok and bolt_ok and not mk.MEMORY.get("threaded_latched"):
        mk.MEMORY["threaded_latched"] = True
        kit.engage("thread_lock")

    # Automated 10 N axial retention proof test once arms clear
    hands_clear = _released()
    if mk.MEMORY.get("threaded_latched") and hands_clear and not mk.MEMORY.get("retention_passed"):
        clear_count = mk.MEMORY.get("clear_count", 0) + 1
        mk.MEMORY["clear_count"] = clear_count
        if clear_count == 10:
            mk.MEMORY["tug_active"] = True
            mk.MEMORY["pre_pull_pos"] = (float(b_pos[0]), float(b_pos[1]), float(b_pos[2]))

    if mk.MEMORY.get("tug_active"):
        tug_count = mk.MEMORY.get("tug_count", 0)
        hose_tip = kit.point(objects, "banjo_hose", (0.0, 0.060, 0.0))
        if tug_count < TUG_STEPS:
            # 10 N pull along +y direction on the hose line
            kit.pull("banjo_hose", force=(0.0, TUG_FORCE_N, 0.0), at=hose_tip)
            mk.MEMORY["tug_count"] = tug_count + 1
            pre_pos = mk.MEMORY.get("pre_pull_pos", b_pos)
            slip = math.dist((float(b_pos[0]), float(b_pos[1]), float(b_pos[2])), pre_pos)
            if slip > DISPLACE_MAX:
                mk.MEMORY["tug_failed"] = True
        else:
            kit.pull("banjo_hose")
            if not mk.MEMORY.get("tug_failed"):
                mk.MEMORY["retention_passed"] = True


kit.on_step(_step_rules)


def score(objects) -> float:
    """Stage-based score from current states and latched history."""
    if mk.MEMORY.get("failed"):
        return 0.0

    b_pos_ok = banjo_positioned(objects)
    b_seat_ok = banjo_seated(objects)
    bolt_in_ok = bolt_inserted(objects)
    bolt_th_ok = bolt_threaded(objects)
    retention = bool(mk.MEMORY.get("retention_passed"))
    released = _released()

    if retention and b_seat_ok and bolt_th_ok and released:
        return 1.0
    elif b_seat_ok and bolt_th_ok:
        return 0.75
    elif b_seat_ok and bolt_in_ok:
        return 0.50
    elif b_pos_ok:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -----------------------------------------------------------------------------
# Scripted Demonstration Trajectory (Bimanual)
# -----------------------------------------------------------------------------
def demo() -> dict[str, list[Waypoint]]:
    """Generate coordinated bimanual trajectory for full-score completion."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    left_wps: list[Waypoint] = []
    right_wps: list[Waypoint] = []

    # Stance
    left_wps.append(Waypoint(1.5, (0.280, 0.180, 0.900), o, "left ready"))
    right_wps.append(Waypoint(1.5, (0.280, -0.180, 0.900), o, "right ready"))

    # Stage 1: Approach pickups
    left_wps.append(Waypoint(2.0, (0.280, 0.1125, BANJO_INIT_Z + 0.005), o, "approach banjo collar"))
    right_wps.append(Waypoint(2.0, (0.280, -0.090, BOLT_INIT_Z + 0.015), o, "approach bolt head"))

    # Stage 2: Grasp
    left_wps.append(Waypoint(1.0, (0.280, 0.1125, BANJO_INIT_Z + 0.005), c, "grasp banjo collar"))
    right_wps.append(Waypoint(1.0, (0.280, -0.090, BOLT_INIT_Z + 0.015), c, "grasp bolt hex head"))

    # Stage 3: Lift
    left_wps.append(Waypoint(2.0, (0.280, 0.080, 0.860), c, "lift banjo fitting"))
    right_wps.append(Waypoint(2.0, (0.280, -0.080, 0.880), c, "lift bolt assembly"))

    # Stage 4: Align banjo concentric over port
    left_wps.append(Waypoint(2.5, (0.280, 0.0225, BANJO_SEATED_Z + 0.025), c, "align banjo over boss"))
    right_wps.append(Waypoint(2.5, (0.280, -0.040, 0.880), c, "hold bolt standby"))

    # Stage 5: Seat banjo flat on lower washer
    left_wps.append(Waypoint(1.5, (0.280, 0.0225, BANJO_SEATED_Z), c, "seat banjo onto lower washer"))
    right_wps.append(Waypoint(1.5, (0.280, 0.000, BANJO_SEATED_Z + 0.060), c, "hover bolt over banjo eye"))

    # Stage 6: Insert bolt through eyelet into caliper bore
    left_wps.append(Waypoint(2.0, (0.280, 0.0225, BANJO_SEATED_Z), c, "hold banjo steady"))
    right_wps.append(Waypoint(2.0, (0.280, 0.000, BOLT_SEATED_Z + 0.008), c, "insert bolt through eyelet"))

    # Stage 7: Thread bolt clockwise down to full seat
    left_wps.append(Waypoint(2.5, (0.280, 0.0225, BANJO_SEATED_Z), c, "hold banjo anti-rotation"))
    right_wps.append(Waypoint(2.5, (0.280, 0.000, BOLT_SEATED_Z), c, "thread bolt home against upper washer"))

    # Stage 8: Right arm releases bolt and backs off
    left_wps.append(Waypoint(1.5, (0.280, 0.0225, BANJO_SEATED_Z), c, "hold hose line"))
    right_wps.append(Waypoint(1.5, (0.280, -0.060, 0.880), o, "right release and clear"))

    # Stage 9: Left arm releases hose and both retract clear
    left_wps.append(Waypoint(2.0, (0.280, 0.120, 0.900), o, "left release hose and clear"))
    right_wps.append(Waypoint(2.0, (0.280, -0.150, 0.900), o, "right fully clear"))

    # Stage 10: Hold clear for automated proof test
    left_wps.append(Waypoint(2.5, (0.280, 0.120, 0.900), o, "stay clear for proof pull"))
    right_wps.append(Waypoint(2.5, (0.280, -0.150, 0.900), o, "stay clear for proof pull"))

    return {"left": left_wps, "right": right_wps}


# -----------------------------------------------------------------------------
# Offline Self-Test
# -----------------------------------------------------------------------------
def _fake_state(banjo_pos=None, banjo_quat=None, bolt_pos=None, bolt_quat=None):
    bp = np.array([*BANJO_INIT_XY, BANJO_INIT_Z]) if banjo_pos is None else np.asarray(banjo_pos)
    bq = np.array([0.0, 0.0, 0.0, 1.0]) if banjo_quat is None else np.asarray(banjo_quat)
    bltp = np.array([*BOLT_INIT_XY, BOLT_INIT_Z]) if bolt_pos is None else np.asarray(bolt_pos)
    bltq = np.array([0.0, 0.0, 0.0, 1.0]) if bolt_quat is None else np.asarray(bolt_quat)
    return kit.fake_states(moved={"banjo_hose": (bp, bq), "banjo_bolt": (bltp, bltq)})


def self_test() -> int:
    """Validate all scoring stages, failure latches, and kinematics offline."""
    global _TEST_TCPS
    banjo_seated_pos = np.array([PORT_XY[0], PORT_XY[1], BANJO_SEATED_Z])
    banjo_pos_pos = np.array([PORT_XY[0] + 0.010, PORT_XY[1], BANJO_SEATED_Z + 0.012])
    bolt_seated_pos = np.array([PORT_XY[0], PORT_XY[1], BOLT_SEATED_Z])
    bolt_inserted_pos = np.array([PORT_XY[0], PORT_XY[1], BOLT_SEATED_Z + 0.010])

    state_init = _fake_state()
    state_aligned = _fake_state(banjo_pos=banjo_pos_pos)
    state_b_seated = _fake_state(banjo_pos=banjo_seated_pos)
    state_bolt_in = _fake_state(banjo_pos=banjo_seated_pos, bolt_pos=bolt_inserted_pos)
    state_threaded = _fake_state(banjo_pos=banjo_seated_pos, bolt_pos=bolt_seated_pos)
    state_dropped = _fake_state(banjo_pos=(PORT_XY[0], PORT_XY[1], TOP - 0.08))

    cases = [
        ("start: nothing done", [state_init], [], 0.0),
        ("banjo aligned near port", [state_aligned], [], 0.25),
        ("banjo seated on lower washer", [state_b_seated], [], 0.25),
        ("bolt inserted through eyelet into bore", [state_bolt_in], [], 0.50),
        ("bolt threaded home, hands touching", [state_threaded], [(PORT_XY[0], PORT_XY[1], 0.85)], 0.75),
        ("part dropped below bench", [state_dropped], [], 0.0),
    ]

    for name, seq, tcps, want in cases:
        mk.new_episode()
        _TEST_TCPS = []
        for st in seq:
            _step_rules(st)
        _TEST_TCPS = tcps
        got = score(seq[-1])
        assert abs(got - want) < 1e-5, f"{name}: got {got}, want {want}"

    # Test full 1.0 sequence with automated retention proof test
    mk.new_episode()
    _TEST_TCPS = []
    # 1. Threaded home
    for _ in range(5):
        _step_rules(state_threaded)
    # 2. Hands clear for 12 steps (triggers proof pull)
    for _ in range(12):
        _step_rules(state_threaded)
    # 3. Pull steps (12 steps)
    for _ in range(12):
        _step_rules(state_threaded)
    got = score(state_threaded)
    assert got == 1.0, f"Full retention trial: got {got}, want 1.0"
    assert success(state_threaded), "Success predicate must be True"

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
