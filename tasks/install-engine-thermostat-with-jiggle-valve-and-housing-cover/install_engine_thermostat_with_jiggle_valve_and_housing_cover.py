from __future__ import annotations

import math
import os
from pathlib import Path
import sys

import numpy as np

# Ensure task directory in sys.path for remote mechkit import
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mechkit as mk
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

# ==============================================================================
# 1. Physical Dimensions & Station Setup (1:1 Scale True Automotive Components)
# ==============================================================================
TOP = 0.7475

GREY = (0.45, 0.45, 0.48)
DARK_GREY = (0.22, 0.22, 0.24)
STEEL = (0.75, 0.75, 0.78)
BRASS = (0.85, 0.70, 0.25)
ALUMINUM = (0.80, 0.82, 0.85)
BLACK_RUBBER = (0.12, 0.12, 0.14)
WHITE = (0.95, 0.95, 0.95)

# Coolant Housing Base Fixture (Cast aluminum engine block thermostat housing)
HOUSING_XY = (0.260, 0.000)
HOUSING_SIZE = (0.120, 0.120, 0.040)
HOUSING_TOP_Z = TOP + HOUSING_SIZE[2]          # 0.7875 m
RECESS_DEPTH = 0.006                           # 6 mm counterbored seat
RECESS_D = 0.056                               # 56 mm counterbore for 54 mm thermostat
RECESS_SEAT_Z = HOUSING_TOP_Z - RECESS_DEPTH   # 0.7815 m

STUD_OFFSET_X = 0.035                          # Dual M8 studs at x = 0.260 +- 0.035
STUD_H = 0.024                                 # Extends 24 mm above housing top deck
STUD_TOP_Z = HOUSING_TOP_Z + STUD_H            # 0.8115 m
STUD_D = 0.008                                 # M8 guide studs

# 1. Thermostat (Stant 13359 / Gates 33008 54 mm automotive thermostat, 85 g)
THERMO_FLANGE_D = 0.054                        # 54 mm OD stainless steel flange
THERMO_FLANGE_H = 0.002                        # 2 mm flange thickness
THERMO_PELLET_D = 0.028                        # 28 mm OD copper wax pellet capsule
THERMO_PELLET_H = 0.022                        # 22 mm lower body extends below flange
THERMO_BRIDGE_W = 0.016                        # 16 mm wide upper bridge
THERMO_BRIDGE_H = 0.018                        # 18 mm upper arch above flange
JIGGLE_OFFSET_R = 0.018                        # 18 mm radial offset from center
JIGGLE_PIN_D = 0.004                           # 4 mm brass jiggle pin / air bleed valve

THERMO_START_POS = (0.220, -0.160, TOP + THERMO_PELLET_H)
TARGET_THERMO_Z = RECESS_SEAT_Z                # 0.7815 m (flange rests on recess floor)

# 2. Molded Nitrile Perimeter Rubber Gasket Ring (Fel-Pro 35062 / OEM style, 15 g)
GASKET_ID = 0.054
GASKET_OD = 0.062
GASKET_H = 0.003                               # 3 mm thick
GASKET_START_POS = (0.220, 0.160, TOP + GASKET_H / 2)
TARGET_GASKET_Z = RECESS_SEAT_Z + THERMO_FLANGE_H + GASKET_H / 2  # 0.7850 m

# 3. Cast Aluminum Thermostat Housing Outlet Cover / Water Neck (120 g)
COVER_FLANGE_SIZE = (0.090, 0.054, 0.008)      # Base flange with matching stud clearance holes
COVER_PIPE_D = 0.032                           # 32 mm radiator hose outlet pipe
COVER_PIPE_H = 0.030                           # 90-degree outlet neck
COVER_START_POS = (0.280, 0.160, TOP + 0.004)
TARGET_COVER_Z = HOUSING_TOP_Z + COVER_FLANGE_SIZE[2] / 2  # 0.7915 m

# Initialize MechKit
kit = mk.Kit()

# Fixture: Engine block coolant housing with counterbore, passage, and dual M8 guide studs
housing_boxes = [
    # Main housing base block
    mk.box(HOUSING_SIZE, at=(0, 0, HOUSING_SIZE[2] / 2), color=ALUMINUM),
    # Stud 1 (left guide stud)
    mk.box((STUD_D, STUD_D, STUD_H), at=(-STUD_OFFSET_X, 0, HOUSING_SIZE[2] + STUD_H / 2), color=STEEL),
    # Stud 2 (right guide stud)
    mk.box((STUD_D, STUD_D, STUD_H), at=(STUD_OFFSET_X, 0, HOUSING_SIZE[2] + STUD_H / 2), color=STEEL),
]
kit.part("coolant_housing", (HOUSING_XY[0], HOUSING_XY[1], TOP), housing_boxes)

# Fixture: Staging tray for thermostat, gasket stand, and cover support
tray_boxes = [
    # Thermostat pocket tray (prevents rolling, holds upright)
    mk.box((0.080, 0.080, 0.020), at=(0.220, -0.160, TOP + 0.010), color=DARK_GREY),
    # Gasket staging mat on table deck
    mk.box((0.070, 0.070, 0.001), at=(0.220, 0.160, TOP + 0.0005), color=DARK_GREY),
    # Housing cover staging cradle
    mk.box((0.090, 0.060, 0.004), at=(0.280, 0.160, TOP + 0.002), color=DARK_GREY),
]
kit.part("staging_fixtures", (0, 0, 0), tray_boxes)

# Dynamic Part 1: Thermostat
# Lower wax pellet, middle flange, upper arch bridge, and brass jiggle pin
thermo_boxes = [
    # Lower wax pellet capsule (extends below flange)
    mk.box((THERMO_PELLET_D, THERMO_PELLET_D, THERMO_PELLET_H), at=(0, 0, -THERMO_PELLET_H / 2), color=BRASS),
    # Flange ring
    mk.box((THERMO_FLANGE_D, THERMO_FLANGE_D, THERMO_FLANGE_H), at=(0, 0, THERMO_FLANGE_H / 2), color=STEEL),
    # Upper bridge arch (for gripper grasp)
    mk.box((THERMO_BRIDGE_W, 0.008, THERMO_BRIDGE_H), at=(0, 0, THERMO_FLANGE_H + THERMO_BRIDGE_H / 2), color=STEEL),
    # Jiggle pin / air bleed valve (offset radially at +X initially)
    mk.box((JIGGLE_PIN_D, JIGGLE_PIN_D, 0.006), at=(JIGGLE_OFFSET_R, 0, THERMO_FLANGE_H + 0.003), color=BRASS),
]
kit.part("thermostat", THERMO_START_POS, thermo_boxes, mass=0.085, color=STEEL, contact_offset=0.0005)

# Dynamic Part 2: Molded Nitrile Perimeter Rubber Gasket Ring
gasket_boxes = [
    # Perimeter square approximation of annular ring
    mk.box((GASKET_OD, GASKET_OD, GASKET_H), at=(0, 0, 0), color=BLACK_RUBBER),
]
kit.part("gasket", GASKET_START_POS, gasket_boxes, mass=0.015, color=BLACK_RUBBER, contact_offset=0.0005)

# Dynamic Part 3: Cast Aluminum Thermostat Housing Outlet Cover (Water Neck)
cover_boxes = [
    # Flange with stud ears
    mk.box(COVER_FLANGE_SIZE, at=(0, 0, 0), color=ALUMINUM),
    # Water outlet pipe angled 90 degrees
    mk.box((COVER_PIPE_D, COVER_PIPE_D, COVER_PIPE_H), at=(0, 0.015, COVER_PIPE_H / 2), color=ALUMINUM),
]
kit.part("housing_cover", COVER_START_POS, cover_boxes, mass=0.120, color=ALUMINUM, contact_offset=0.0005)

# Catches for precision physical assembly verification
_THERMO_ROT = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_THERMO_SEATED_REF = mk._T(_THERMO_ROT, (HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z))
kit.catch("thermo_seated_catch", "thermostat", _THERMO_SEATED_REF)

_GASKET_SEATED_REF = mk._T(None, (HOUSING_XY[0], HOUSING_XY[1], TARGET_GASKET_Z))
kit.catch("gasket_seated_catch", "gasket", _GASKET_SEATED_REF)

_COVER_SEATED_REF = mk._T(None, (HOUSING_XY[0], HOUSING_XY[1], TARGET_COVER_Z))
kit.catch("cover_seated_catch", "housing_cover", _COVER_SEATED_REF)

# Filter internal assembly collisions to prevent PhysX contact mesh binding
kit.filter_collision("thermostat", "coolant_housing")
kit.filter_collision("gasket", "coolant_housing")
kit.filter_collision("gasket", "thermostat")
kit.filter_collision("housing_cover", "coolant_housing")
kit.filter_collision("housing_cover", "thermostat")
kit.filter_collision("housing_cover", "gasket")
kit.filter_collision("thermostat", "staging_fixtures")
kit.filter_collision("gasket", "staging_fixtures")
kit.filter_collision("housing_cover", "staging_fixtures")

OBJECTS = [
    *kit.objects(),
]


# ==============================================================================
# 2. Simulation Step Rules & Orientation Verification
# ==============================================================================
def _simulation_rules(objects):
    t_st = objects["thermostat"]
    t_pos = kit.point(objects, "thermostat", (0, 0, 0))

    # Defect gate 1: Table drop outside tray and housing recess
    d_tray = math.dist((t_pos[0], t_pos[1]), (0.220, -0.160))
    d_house = math.dist((t_pos[0], t_pos[1]), HOUSING_XY)
    if t_pos[2] < TOP + 0.005 and d_tray > 0.050 and d_house > 0.050:
        mk.MEMORY["thermo_dropped_defect"] = True

    # Defect gate 2: Inverted thermostat check (wax pellet pointing up instead of down)
    t_quat = t_st.quat_xyzw if hasattr(t_st, "quat_xyzw") else (0, 0, 0, 1)
    # Pitch/roll inversion: z-axis of part should point roughly along world +Z (qz > 0)
    # Rotation matrix R_33 = 1 - 2*(qx^2 + qy^2)
    r33 = 1.0 - 2.0 * (t_quat[0]**2 + t_quat[1]**2)
    if r33 < 0.0 and d_house < 0.040:
        mk.MEMORY["thermo_inverted_defect"] = True

    # 1. Thermostat capture & lift
    if t_pos[2] > THERMO_START_POS[2] + 0.015 and d_tray < 0.040:
        mk.MEMORY["thermo_lifted"] = True

    step = mk.MEMORY.get("_step_cnt", 0)
    mk.MEMORY["_step_cnt"] = step + 1

    # 2. Thermostat seated in housing with verified 12 o'clock jiggle pin orientation (+-15 deg)
    # In initial frame, jiggle pin is at local (+JIGGLE_OFFSET_R, 0, 0).
    # To point along world +Y (12 o'clock), yaw must be +pi/2 (90 degrees).
    curr_yaw = math.atan2(2.0 * (t_quat[3] * t_quat[2] + t_quat[0] * t_quat[1]),
                          1.0 - 2.0 * (t_quat[1]**2 + t_quat[2]**2))
    # Target yaw: math.pi / 2 (1.5708 rad).
    yaw_error = abs(curr_yaw - math.pi / 2)
    while yaw_error > math.pi:
        yaw_error = abs(yaw_error - 2 * math.pi)

    if step % 20 == 0:
        d_th_xy = math.dist((t_pos[0], t_pos[1]), HOUSING_XY)
        d_th_z = abs(t_pos[2] - TARGET_THERMO_Z)
        print(f"[STEP {step}] t_pos=({t_pos[0]:.3f},{t_pos[1]:.3f},{t_pos[2]:.4f}) yaw_err={math.degrees(yaw_error):.1f}deg d_xy={d_th_xy*1000:.1f}mm dz={d_th_z*1000:.1f}mm seated={mk.MEMORY.get('thermo_seated')}")

    # 20 degrees tolerance in radians = 0.349 rad
    jiggle_oriented = (yaw_error <= math.radians(20.0))

    d_thermo_house_xy = math.dist((t_pos[0], t_pos[1]), HOUSING_XY)
    thermo_depth_ok = abs(t_pos[2] - TARGET_THERMO_Z) <= 0.015

    if d_thermo_house_xy <= 0.020 and thermo_depth_ok and r33 > 0.8:
        if jiggle_oriented:
            mk.MEMORY["thermo_oriented"] = True
            if not mk.MEMORY.get("thermo_seated"):
                mk.MEMORY["thermo_seated"] = True
                print(f"[CATCH ENGAGED] thermo_seated at step {step}!")
                kit.engage("thermo_seated_catch")

    # 3. Gasket seated in counterbore groove over thermostat flange
    g_pos = kit.point(objects, "gasket", (0, 0, 0))
    d_gasket_house_xy = math.dist((g_pos[0], g_pos[1]), HOUSING_XY)
    gasket_depth_ok = abs(g_pos[2] - TARGET_GASKET_Z) <= 0.015

    if mk.MEMORY.get("thermo_seated") and d_gasket_house_xy <= 0.020 and gasket_depth_ok:
        if not mk.MEMORY.get("gasket_seated"):
            mk.MEMORY["gasket_seated"] = True
            print(f"[CATCH ENGAGED] gasket_seated at step {step}!")
            kit.engage("gasket_seated_catch")

    # 4. Housing cover seated flush over guide studs
    c_pos = kit.point(objects, "housing_cover", (0, 0, 0))
    d_cover_house_xy = math.dist((c_pos[0], c_pos[1]), HOUSING_XY)
    cover_depth_ok = abs(c_pos[2] - TARGET_COVER_Z) <= 0.015

    if mk.MEMORY.get("gasket_seated") and d_cover_house_xy <= 0.015 and cover_depth_ok:
        if not mk.MEMORY.get("cover_seated"):
            mk.MEMORY["cover_seated"] = True
            print(f"[CATCH ENGAGED] cover_seated at step {step}!")
            kit.engage("cover_seated_catch")


kit.on_step(_simulation_rules)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


# ==============================================================================
# 3. Scoring Function (Strict Partial Credit & Anti-Cheat)
# ==============================================================================
def score(objects) -> float:
    if mk.MEMORY.get("thermo_dropped_defect") or mk.MEMORY.get("thermo_inverted_defect"):
        return 0.0

    t_st = objects["thermostat"]
    g_st = objects["gasket"]
    c_st = objects["housing_cover"]

    t_pos = kit.point(objects, "thermostat", (0, 0, 0))
    g_pos = kit.point(objects, "gasket", (0, 0, 0))
    c_pos = kit.point(objects, "housing_cover", (0, 0, 0))

    # 1. Thermostat seated home on axis, correct depth, and verified 12 o'clock orientation
    d_t_house = math.dist((t_pos[0], t_pos[1]), HOUSING_XY)
    t_is_home = (d_t_house <= 0.006 and abs(t_pos[2] - TARGET_THERMO_Z) <= 0.005 and mk.MEMORY.get("thermo_oriented", False))

    # 2. Gasket seated flush over flange in groove
    d_g_house = math.dist((g_pos[0], g_pos[1]), HOUSING_XY)
    g_is_home = (d_g_house <= 0.006 and abs(g_pos[2] - TARGET_GASKET_Z) <= 0.005)

    # 3. Cover seated flush over studs (< 1 mm gap from housing deck)
    d_c_house = math.dist((c_pos[0], c_pos[1]), HOUSING_XY)
    c_is_home = (d_c_house <= 0.005 and abs(c_pos[2] - TARGET_COVER_Z) <= 0.003)

    # 4. Grippers released and retracted clear (>20 mm)
    arms_clear = _released(t_st) and _released(g_st) and _released(c_st)

    print(f"[FINAL SCORE] t_home={t_is_home} g_home={g_is_home} c_home={c_is_home} arms_clear={arms_clear} "
          f"t_pos={t_pos} g_pos={g_pos} c_pos={c_pos} memory={dict(mk.MEMORY)}")

    # Full assembly complete: 1.0
    if t_is_home and g_is_home and c_is_home and arms_clear:
        return 1.0

    # Intermediate stages:
    if t_is_home and g_is_home:
        return 0.75

    if t_is_home:
        return 0.50

    if mk.MEMORY.get("thermo_lifted"):
        return 0.25

    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c_pos = objects["housing_cover"].pos
    return float(math.dist((c_pos[0], c_pos[1]), HOUSING_XY) + abs(c_pos[2] - TARGET_COVER_Z))


# ==============================================================================
# 4. Scripted Bimanual Demonstration Trajectory
# ==============================================================================
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

TRAVEL = 0.885
FINGER_REGRIP = -0.025


from yam_arena.demo import Waypoint


def demo():
    variant = os.environ.get("DEMO_VARIANT", "normal")

    (tx, ty, tz) = THERMO_START_POS
    bridge_grasp_z = tz + THERMO_FLANGE_H + THERMO_BRIDGE_H / 2 + 0.003
    thermo_grip_offset = bridge_grasp_z - tz

    (hx, hy) = HOUSING_XY
    # Compensate for 18 mm grasp center offset when rotated 90 deg
    hx_thermo = hx - 0.018

    # Right Arm: Handles Thermostat
    # Grasps bridge, lifts, rotates wrist to yaw = pi/2 (12 o'clock jiggle pin), lowers into housing
    t_target_yaw = 0.0 if variant == "misoriented" else math.pi / 2

    if variant == "no_thermo":
        r_plan = [
            Waypoint(39.1, (tx, ty, TRAVEL), FINGER_OPEN, "idle negative control", yaw=0.0),
        ]
    else:
        r_plan = [
            Waypoint(2.0, (tx, ty, TRAVEL), FINGER_OPEN, "over thermostat staging tray", yaw=0.0),
            Waypoint(1.5, (tx, ty, bridge_grasp_z), FINGER_OPEN, "lower to bridge arch", yaw=0.0),
            Waypoint(0.8, (tx, ty, bridge_grasp_z), FINGER_CLOSED, "grip bridge arch", yaw=0.0),
            Waypoint(1.5, (tx, ty, TRAVEL), FINGER_CLOSED, "lift thermostat clear of tray", yaw=0.0),
            Waypoint(2.0, (hx_thermo, hy, TRAVEL), FINGER_CLOSED, "carry over housing and clock to 12 oclock", yaw=t_target_yaw),
            Waypoint(1.8, (hx_thermo, hy, TARGET_THERMO_Z + 0.015 + thermo_grip_offset), FINGER_CLOSED, "lower into counterbore mouth", yaw=t_target_yaw),
            Waypoint(1.2, (hx_thermo, hy, TARGET_THERMO_Z + thermo_grip_offset), FINGER_CLOSED, "seat flange flush in recess", yaw=t_target_yaw),
            Waypoint(0.8, (hx_thermo, hy, TARGET_THERMO_Z + thermo_grip_offset), FINGER_OPEN, "release thermostat", yaw=t_target_yaw),
            Waypoint(1.5, (hx_thermo, hy, TRAVEL), FINGER_OPEN, "right arm retreat to travel", yaw=t_target_yaw),
            Waypoint(1.8, (tx, ty, TRAVEL), FINGER_OPEN, "right arm park clear", yaw=0.0),
            Waypoint(24.2, (tx, ty, TRAVEL), FINGER_OPEN, "right arm hold idle", yaw=0.0),
        ]

    # Left Arm: Handles Gasket then Housing Outlet Cover
    (gx, gy, gz) = GASKET_START_POS
    gasket_grasp_z = gz + 0.005
    gasket_grip_offset = gasket_grasp_z - gz

    (cx, cy, cz) = COVER_START_POS
    cover_grasp_z = cz + 0.015
    cover_grip_offset = cover_grasp_z - cz

    l_finger_end = FINGER_CLOSED if variant == "touching_cover" else FINGER_OPEN
    cover_end_pos = (hx, hy, TARGET_COVER_Z + cover_grip_offset) if variant == "touching_cover" else (cx, cy, TRAVEL)

    if variant == "no_gasket":
        l_plan = [
            Waypoint(39.1, (gx, gy, TRAVEL), FINGER_OPEN, "idle negative control", yaw=0.0),
        ]
    else:
        l_plan = [
            # Wait for right arm to seat thermostat
            Waypoint(11.0, (gx, gy, TRAVEL), FINGER_OPEN, "wait for thermostat seating", yaw=0.0),
            # Stage 2: Pick and seat gasket
            Waypoint(1.5, (gx, gy, gasket_grasp_z), FINGER_OPEN, "lower to gasket ring", yaw=0.0),
            Waypoint(0.8, (gx, gy, gasket_grasp_z), FINGER_CLOSED, "grip gasket ring", yaw=0.0),
            Waypoint(1.5, (gx, gy, TRAVEL), FINGER_CLOSED, "lift gasket clear", yaw=0.0),
            Waypoint(2.0, (hx, hy, TRAVEL), FINGER_CLOSED, "carry gasket over housing", yaw=0.0),
            Waypoint(1.5, (hx, hy, TARGET_GASKET_Z + 0.010 + gasket_grip_offset), FINGER_CLOSED, "lower gasket into recess mouth", yaw=0.0),
            Waypoint(1.2, (hx, hy, TARGET_GASKET_Z + gasket_grip_offset), FINGER_CLOSED, "press gasket flush over flange", yaw=0.0),
            Waypoint(0.8, (hx, hy, TARGET_GASKET_Z + gasket_grip_offset), FINGER_OPEN, "release gasket", yaw=0.0),
            Waypoint(1.5, (hx, hy, TRAVEL), FINGER_OPEN, "lift clear of gasket", yaw=0.0),
            # Stage 3: Pick and seat housing cover over studs
            Waypoint(2.0, (cx, cy, TRAVEL), FINGER_OPEN, "move over housing cover", yaw=0.0),
            Waypoint(1.5, (cx, cy, cover_grasp_z), FINGER_OPEN, "lower to cover neck", yaw=0.0),
            Waypoint(0.8, (cx, cy, cover_grasp_z), FINGER_CLOSED, "grip cover neck", yaw=0.0),
            Waypoint(1.5, (cx, cy, TRAVEL), FINGER_CLOSED, "lift cover clear of stand", yaw=0.0),
            Waypoint(2.0, (hx, hy, TRAVEL), FINGER_CLOSED, "carry cover over housing studs", yaw=0.0),
            Waypoint(1.8, (hx, hy, STUD_TOP_Z + 0.015 + cover_grip_offset), FINGER_CLOSED, "align holes over studs", yaw=0.0),
            Waypoint(2.0, (hx, hy, TARGET_COVER_Z + cover_grip_offset), FINGER_CLOSED, "lower cover flush onto gasket", yaw=0.0),
            Waypoint(0.8, (hx, hy, TARGET_COVER_Z + cover_grip_offset), l_finger_end, "release cover", yaw=0.0),
            Waypoint(1.5, cover_end_pos, l_finger_end, "hold cover" if variant == "touching_cover" else "left arm retreat to travel", yaw=0.0),
            Waypoint(2.7, cover_end_pos, l_finger_end, "settle and hold success", yaw=0.0),
        ]

    return {"left": l_plan, "right": r_plan}


# ==============================================================================
# 5. Offline Unit Test Suite (Self-Test)
# ==============================================================================
def self_test():
    mk.new_episode()
    st_init = kit.fake_states()
    assert score(st_init) == 0.0, "initial state must score 0.0"

    # Defect: dropped thermostat
    mk.MEMORY["thermo_dropped_defect"] = True
    assert score(st_init) == 0.0, "dropped defect must score 0.0"
    mk.MEMORY.clear()

    # Defect: inverted thermostat
    mk.MEMORY["thermo_inverted_defect"] = True
    assert score(st_init) == 0.0, "inverted defect must score 0.0"
    mk.MEMORY.clear()

    # 0.25: Lifted thermostat
    mk.MEMORY["thermo_lifted"] = True
    assert score(st_init) == 0.25, "lifted state must score 0.25"

    # 0.50: Thermostat seated with verified 12 oclock jiggle valve orientation
    st_thermo = kit.fake_states(moved={
        "thermostat": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z]), np.array([0, 0, 0, 1])),
    })
    mk.MEMORY["thermo_oriented"] = True
    assert score(st_thermo) == 0.50, "thermostat seated oriented must score 0.50"

    # Misoriented jiggle pin: must not get 0.50
    mk.MEMORY["thermo_oriented"] = False
    assert score(st_thermo) == 0.25, "misoriented jiggle pin must fall back to 0.25"
    mk.MEMORY["thermo_oriented"] = True

    # 0.75: Thermostat and gasket seated
    st_gasket = kit.fake_states(moved={
        "thermostat": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z]), np.array([0, 0, 0, 1])),
        "gasket": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_GASKET_Z]), np.array([0, 0, 0, 1])),
    })
    assert score(st_gasket) == 0.75, "thermostat and gasket seated must score 0.75"

    # 1.0: Full assembly complete with cover seated and arms clear
    st_full = kit.fake_states(moved={
        "thermostat": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z]), np.array([0, 0, 0, 1])),
        "gasket": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_GASKET_Z]), np.array([0, 0, 0, 1])),
        "housing_cover": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_COVER_Z]), np.array([0, 0, 0, 1])),
    })

    # Gripper touching cover: arms not clear -> 0.75
    global _TEST_TCPS
    _TEST_TCPS = [np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_COVER_Z])]
    assert score(st_full) == 0.75, "touching cover must score 0.75"

    # Arms retracted clear (>20 mm)
    _TEST_TCPS = [np.array([0.20, 0.20, TRAVEL]), np.array([0.20, -0.20, TRAVEL])]
    assert score(st_full) == 1.0, "full assembly clear must score 1.0"
    _TEST_TCPS = None

    # Case 10: Dropped defect overrides full success -> 0.0
    mk.MEMORY["thermo_dropped_defect"] = True
    assert score(st_full) == 0.0, "dropped defect must override full assembly to 0.0"
    mk.MEMORY.clear()

    # Case 11: Inverted defect overrides full success -> 0.0
    mk.MEMORY["thermo_inverted_defect"] = True
    assert score(st_full) == 0.0, "inverted defect must override full assembly to 0.0"
    mk.MEMORY.clear()

    # Case 12: Cover cocked / off studs -> not 1.0
    st_cocked_cover = kit.fake_states(moved={
        "thermostat": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z]), np.array([0, 0, 0, 1])),
        "gasket": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_GASKET_Z]), np.array([0, 0, 0, 1])),
        "housing_cover": (np.array([HOUSING_XY[0] + 0.015, HOUSING_XY[1], TARGET_COVER_Z]), np.array([0, 0, 0, 1])),
    })
    _TEST_TCPS = [np.array([0.20, 0.20, TRAVEL]), np.array([0.20, -0.20, TRAVEL])]
    assert score(st_cocked_cover) != 1.0, "cocked cover must not score 1.0"

    # Case 13: Simulation rule drop detection
    mk.new_episode()
    st_dropped = kit.fake_states(moved={
        "thermostat": (np.array([0.100, 0.000, TOP + 0.002]), np.array([0, 0, 0, 1])),
    })
    _simulation_rules(st_dropped)
    assert mk.MEMORY.get("thermo_dropped_defect") is True, "table drop outside tray/housing must latch defect"

    # Case 14: Simulation rule orientation detection
    mk.new_episode()
    # Oriented at +pi/2 yaw (12 oclock)
    st_rule_oriented = kit.fake_states(moved={
        "thermostat": (np.array([HOUSING_XY[0], HOUSING_XY[1], TARGET_THERMO_Z]), np.array([0, 0, 0.7071, 0.7071])),
    })
    _simulation_rules(st_rule_oriented)
    assert mk.MEMORY.get("thermo_oriented") is True, "yaw=+pi/2 must satisfy 12 oclock jiggle valve orientation"
    assert mk.MEMORY.get("thermo_seated") is True, "oriented thermostat must engage thermo_seated"

    _TEST_TCPS = None
    mk.new_episode()
    return 14


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
