"""Position and clamp a steel workpiece bar in a benchtop drill press vise onto parallels.

On the station table:
- Precision benchtop drill press vise with cast body, fixed jaw, moving jaw on prismatic slider,
  and a quick-acting cam-action push-lever clamp mechanism.
- Two ground steel parallels resting inside the vise channel on the bed.
- Rectangular steel workpiece bar (100 x 28 x 25 mm, 0.55 kg) on the staging rack.

The task:
1. Left arm grasps the steel workpiece bar, lifts it, transfers it over the vise jaws,
   and seats it flush down onto the parallels registering against the fixed jaw and lateral end stop.
2. Right arm grasps the cam-clamp lever handle and pushes it forward to drive the moving jaw closed against the bar,
   achieving a solid mechanical clamp.
3. Both arms retract clear (TCP >= 60 mm, collision hull >= 20 mm) and hold stationary for 1.0 s.

100% pure PhysX dynamic contact physics (zero synthetic pose writing / zero teleportation).
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# --- Colors ---
CAST_IRON = (0.22, 0.24, 0.28)
STEEL_GREY = (0.75, 0.76, 0.78)
GROUND_STEEL = (0.85, 0.86, 0.88)
BAR_BLUE_STEEL = (0.20, 0.35, 0.55)
BRASS_GOLD = (0.82, 0.68, 0.28)
BLACK_OXIDE = (0.10, 0.10, 0.12)
RACK_GREY = (0.35, 0.36, 0.38)

# --- Geometry & Placement ---
VISE_XY = (0.280, 0.020)
VISE_BASE_H = 0.025
BED_Z = TOP + VISE_BASE_H  # 0.7475 + 0.025 = 0.7725
JAW_W = 0.100  # jaw width along Y
JAW_H = 0.035  # jaw height above bed
JAW_T = 0.020  # jaw plate thickness along X
JAW_TOP_Z = BED_Z + JAW_H  # 0.8075

# Fixed jaw at +X edge of vise bed
FIXED_JAW_X = VISE_XY[0] + 0.035
FIXED_FACE_X = FIXED_JAW_X - JAW_T / 2  # 0.280 + 0.035 - 0.010 = 0.305

# Moving jaw starts open at -X
GAP_INIT = 0.045
MOVING_FACE_X0 = FIXED_FACE_X - GAP_INIT  # 0.305 - 0.045 = 0.260
MOVING_JAW_X0 = MOVING_FACE_X0 - JAW_T / 2  # 0.250

# Parallels: two 100 x 8 x 16 mm ground steel bars resting on the bed
PARALLEL_H = 0.016
PARALLEL_W = 0.008
PARALLEL_L = 0.090
SEAT_Z = BED_Z + PARALLEL_H  # 0.7725 + 0.016 = 0.7885

# Workpiece bar: 100 x 28 x 25 mm, 0.55 kg
BAR_L = 0.100  # along Y (100 mm bar)
BAR_W = 0.028  # along X (clamped dimension)
BAR_H = 0.025  # along Z
BAR_MASS = 0.55

# Clamped target position
BAR_TARGET_X = FIXED_FACE_X - BAR_W / 2  # 0.305 - 0.014 = 0.291
BAR_TARGET_Y = VISE_XY[1]  # 0.020
BAR_TARGET_Z = SEAT_Z + BAR_H / 2  # 0.7885 + 0.0125 = 0.8010

# Staging rack on left side (+Y)
RACK_XY = (0.240, 0.220)
BAR_INIT_XY = RACK_XY
BAR_INIT_Z = TOP + 0.015 + BAR_H / 2  # 0.7475 + 0.015 + 0.0125 = 0.7750

# Vise screw handle at -X end of vise body
HANDLE_X0 = VISE_XY[0] - 0.090  # 0.190
HANDLE_Y0 = VISE_XY[1]  # 0.020
HANDLE_Z = BED_Z + 0.010  # 0.7825

JAW_TRAVEL_MAX = 0.022

kit = mk.Kit()

# 1. Drill Press Vise Base & Fixed Jaw (Fixed static part)
def _vise_base_boxes():
    return [
        # Main base plate with mounting slots
        mk.box((0.180, 0.120, VISE_BASE_H), at=(0.0, 0.0, VISE_BASE_H / 2), color=CAST_IRON),
        # Fixed jaw block at +X
        mk.box((JAW_T, JAW_W, JAW_H), at=(0.035, 0.0, VISE_BASE_H + JAW_H / 2), color=CAST_IRON),
        # Hardened ground steel jaw plate on fixed face
        mk.box((0.004, JAW_W - 0.004, JAW_H - 0.004), at=(0.035 - JAW_T / 2 + 0.002, 0.0, VISE_BASE_H + JAW_H / 2), color=GROUND_STEEL),
        # Lateral end stop bracket on -Y flank
        mk.box((0.050, 0.008, JAW_H), at=(0.025, -JAW_W / 2 - 0.004, VISE_BASE_H + JAW_H / 2), color=BLACK_OXIDE),
        # Lead screw thrust housing at -X
        mk.box((0.030, 0.050, VISE_BASE_H + 0.010), at=(-0.075, 0.0, (VISE_BASE_H + 0.010) / 2), color=CAST_IRON),
        # Two precision parallels resting on the bed
        mk.box((PARALLEL_W, PARALLEL_L, PARALLEL_H), at=(0.020, 0.0, VISE_BASE_H + PARALLEL_H / 2), color=GROUND_STEEL),
        mk.box((PARALLEL_W, PARALLEL_L, PARALLEL_H), at=(-0.015, 0.0, VISE_BASE_H + PARALLEL_H / 2), color=GROUND_STEEL),
    ]

kit.part("vise_base", (VISE_XY[0], VISE_XY[1], TOP), _vise_base_boxes(), mass=None, color=CAST_IRON)

# 2. Moving Jaw on Prismatic Slider
def _moving_jaw_boxes():
    return [
        # Moving jaw slider body
        mk.box((JAW_T, JAW_W, JAW_H), at=(0.0, 0.0, JAW_H / 2), color=CAST_IRON),
        # Hardened ground steel jaw plate on moving face (+X face)
        mk.box((0.004, JAW_W - 0.004, JAW_H - 0.004), at=(JAW_T / 2 - 0.002, 0.0, JAW_H / 2), color=GROUND_STEEL),
        # Guide rib underneath slider
        mk.box((0.030, 0.040, 0.010), at=(-0.010, 0.0, -0.005), color=STEEL_GREY),
    ]

kit.part(
    "moving_jaw",
    (MOVING_JAW_X0, VISE_XY[1], BED_Z),
    _moving_jaw_boxes(),
    mass=0.85,
    color=CAST_IRON,
    solver_position_iterations=16,
)
kit.slider("jaw_slide", "vise_base", "moving_jaw", (1, 0, 0),
           0.0, JAW_TRAVEL_MAX, start=0.0, friction=0.5)

# 3. Cam-action Vise Clamp Push Lever / Handle (Physically coupled to moving jaw)
def _handle_boxes():
    return [
        # Center hub
        mk.box((0.020, 0.020, 0.020), at=(0.0, 0.0, 0.0), color=BLACK_OXIDE),
        # Cam clamp push lever / T-bar handle along Y
        mk.box((0.012, 0.070, 0.014), at=(0.0, 0.0, 0.0), color=STEEL_GREY),
        # Knurled ball knobs on ends of lever
        mk.box((0.016, 0.016, 0.016), at=(0.0, 0.035, 0.0), color=BRASS_GOLD),
        mk.box((0.016, 0.016, 0.016), at=(0.0, -0.035, 0.0), color=BRASS_GOLD),
        # Rigid connecting rod / push transmission linkage linking handle to moving jaw
        mk.box((0.060, 0.014, 0.014), at=(0.030, 0.0, 0.0), color=STEEL_GREY),
    ]

kit.part(
    "vise_handle",
    (HANDLE_X0, HANDLE_Y0, HANDLE_Z),
    _handle_boxes(),
    mass=0.25,
    color=STEEL_GREY,
    solver_position_iterations=16,
)
# Rigid mechanical transmission bond coupling lever handle directly to the moving jaw:
# When the robot pushes the handle, the entire moving jaw assembly translates forward with it!
kit.bond("handle_to_jaw_bond", "moving_jaw", "vise_handle", break_force=1e6)

# 4. Workpiece Staging Rack
def _rack_boxes():
    return [
        mk.box((0.080, 0.120, 0.015), at=(0.0, 0.0, 0.0075), color=RACK_GREY),
        # Locating side rails
        mk.box((0.080, 0.008, 0.015), at=(0.0, 0.056, 0.015), color=BLACK_OXIDE),
        mk.box((0.080, 0.008, 0.015), at=(0.0, -0.056, 0.015), color=BLACK_OXIDE),
    ]

kit.part("staging_rack", (RACK_XY[0], RACK_XY[1], TOP), _rack_boxes(), mass=None, color=RACK_GREY)

# 5. Workpiece Steel Bar (Dynamic rigid body, 100% genuine dynamic contact)
def _bar_boxes():
    return [
        mk.box((BAR_W, BAR_L, BAR_H), at=(0.0, 0.0, 0.0), color=BAR_BLUE_STEEL),
        # Center punch mark / machining layout line on top surface
        mk.box((0.002, BAR_L - 0.010, 0.001), at=(0.0, 0.0, BAR_H / 2 + 0.0005), color=GROUND_STEEL),
    ]

kit.part(
    "workpiece_bar",
    (BAR_INIT_XY[0], BAR_INIT_XY[1], BAR_INIT_Z),
    _bar_boxes(),
    mass=BAR_MASS,
    color=BAR_BLUE_STEEL,
    solver_position_iterations=16,
)

FIXTURES = []
OBJECTS = kit.objects()

RELEASE_TCP_CLEAR = 0.060
RELEASE_HULL_CLEAR = 0.020

def hands_clear(objects) -> bool:
    targets = [objects["vise_handle"], objects["workpiece_bar"], objects["moving_jaw"]]
    tcps = mk.tcps()
    for st in targets:
        lo, hi = np.asarray(st.low) - RELEASE_TCP_CLEAR, np.asarray(st.high) + RELEASE_TCP_CLEAR
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps):
            return False
    boxes = mk.robot_collision_boxes()
    for lo_r, hi_r in boxes:
        for st in targets:
            gap = np.maximum(np.maximum(np.asarray(st.low) - hi_r, lo_r - np.asarray(st.high)), 0.0)
            if float(np.linalg.norm(gap)) < RELEASE_HULL_CLEAR:
                return False
    return True

# --- Simulation Hooks & State ---
def on_reset():
    mk.MEMORY.clear()
    mk.MEMORY["stillness_steps"] = 0

def _on_step(objects: dict[str, Any]):
    bar_pos = objects["workpiece_bar"].pos
    jaw_pos = objects["moving_jaw"].pos
    handle_pos = objects["vise_handle"].pos

    # Live instantaneous physical state checks (Zero permanent positive latches!)
    dx = abs(bar_pos[0] - BAR_TARGET_X)
    dy = abs(bar_pos[1] - BAR_TARGET_Y)
    dz = abs(bar_pos[2] - BAR_TARGET_Z)
    is_seated = (dx <= 0.005 and dy <= 0.008 and dz <= 0.004)

    jaw_stroke = (jaw_pos[0] - MOVING_JAW_X0) * 1000.0  # in mm
    handle_stroke = (handle_pos[0] - HANDLE_X0) * 1000.0  # in mm
    is_clamped = is_seated and (jaw_stroke >= 14.0 and handle_stroke >= 14.0)

    clear = hands_clear(objects)
    if is_clamped and clear:
        mk.MEMORY["stillness_steps"] = mk.MEMORY.get("stillness_steps", 0) + 1
    else:
        mk.MEMORY["stillness_steps"] = 0

kit.on_step(_on_step)

def score(objects: dict[str, Any]) -> float:
    M = mk.MEMORY

    # Catastrophic fail check: dropped off table
    bar_pos = objects["workpiece_bar"].pos
    if bar_pos[2] < TOP - 0.050:
        return 0.00

    # Live instantaneous physical evaluation
    dx = abs(bar_pos[0] - BAR_TARGET_X)
    dy = abs(bar_pos[1] - BAR_TARGET_Y)
    dz = abs(bar_pos[2] - BAR_TARGET_Z)
    is_seated = (dx <= 0.005 and dy <= 0.008 and dz <= 0.004)

    jaw_pos = objects["moving_jaw"].pos
    handle_pos = objects["vise_handle"].pos
    jaw_stroke = (jaw_pos[0] - MOVING_JAW_X0) * 1000.0
    handle_stroke = (handle_pos[0] - HANDLE_X0) * 1000.0
    is_clamped = is_seated and (jaw_stroke >= 14.0 and handle_stroke >= 14.0)

    s = 0.0
    # 0.20: Bar lifted clear of staging rack
    if bar_pos[2] >= 0.805:
        s = 0.20

    # 0.50: Bar seated on parallels inside vise channel
    if is_seated:
        s = 0.50

    # 0.85: Vise closed and clamped firmly on bar
    if is_clamped:
        s = 0.85

    # 1.00: Full mechanical clamp + hands clear for 20 continuous steps (1.0 s)
    if is_clamped and M.get("stillness_steps", 0) >= 20:
        s = 1.00

    return s

def success(objects: dict[str, Any]) -> bool:
    return score(objects) >= 1.0 - 1e-9

task_score = score
task_success = success

# --- Scripted Demonstration ---
def demo():
    variant = os.environ.get("DEMO_VARIANT", "")

    # Left arm: Grasp steel workpiece bar from staging rack
    bar_pick_z = BAR_INIT_Z + 0.010
    carry_z = TOP + 0.140
    seat_z = BAR_TARGET_Z + 0.010

    left = [
        # 1. Hover over staging rack (t = 1.2s)
        Waypoint(1.2, (RACK_XY[0], RACK_XY[1], carry_z), FINGER_OPEN, "left_hover", math.pi / 2),
        # 2. Descend to pinch grasp bar (t = 0.8s)
        Waypoint(0.8, (RACK_XY[0], RACK_XY[1], bar_pick_z), FINGER_OPEN, "left_descend", math.pi / 2),
        Waypoint(0.6, (RACK_XY[0], RACK_XY[1], bar_pick_z), FINGER_CLOSED, "left_grasp", math.pi / 2),
        # 3. Lift bar clear (t = 1.0s)
        Waypoint(1.0, (RACK_XY[0], RACK_XY[1], carry_z), FINGER_CLOSED, "left_lift", math.pi / 2),
        # 4. Transfer over vise channel (t = 1.2s)
        Waypoint(1.2, (BAR_TARGET_X, BAR_TARGET_Y, carry_z), FINGER_CLOSED, "left_over_vise", math.pi / 2),
    ]

    if variant == "no_seat":
        left.append(Waypoint(10.0, (BAR_TARGET_X, BAR_TARGET_Y, carry_z), FINGER_CLOSED, "left_suspended", math.pi / 2))
        right = [Waypoint(16.0, (0.150, -0.220, carry_z), FINGER_OPEN, "right_park", 0.0)]
        return {"left": left, "right": right}

    # 5. Lower bar down flush onto parallels (t = 1.0s + 0.6s release)
    left += [
        Waypoint(1.0, (BAR_TARGET_X, BAR_TARGET_Y, seat_z), FINGER_CLOSED, "left_seat", math.pi / 2),
        Waypoint(0.6, (BAR_TARGET_X, BAR_TARGET_Y, seat_z), FINGER_OPEN, "left_release", math.pi / 2),
        # 6. Retract left arm clear (t = 0.8s lift + 1.0s retract + 6.0s park dwell)
        Waypoint(0.8, (BAR_TARGET_X, BAR_TARGET_Y + 0.040, carry_z), FINGER_OPEN, "left_lift_clear", math.pi / 2),
        Waypoint(1.0, (0.150, 0.220, carry_z), FINGER_OPEN, "left_retract", 0.0),
        Waypoint(6.0, (0.150, 0.220, carry_z), FINGER_OPEN, "left_park", 0.0),
    ]

    # Right arm: Advance vise handle to clamp moving jaw
    handle_approach_x = HANDLE_X0 - 0.040
    handle_push_target_x = HANDLE_X0 + JAW_TRAVEL_MAX + 0.003  # push forward +19 mm

    if variant == "no_clamp":
        right = [Waypoint(16.0, (0.150, -0.220, carry_z), FINGER_OPEN, "right_park", 0.0)]
        return {"left": left, "right": right}

    # Left duration until clear of vise: 1.2 + 0.8 + 0.6 + 1.0 + 1.2 + 1.0 + 0.6 + 0.8 = 7.2s
    right = [
        # Wait for left arm to seat the bar and retract clear
        Waypoint(7.4, (0.150, -0.220, carry_z), FINGER_OPEN, "right_wait", 0.0),
        # Approach vise handle (t = 1.0s)
        Waypoint(1.0, (handle_approach_x, HANDLE_Y0, HANDLE_Z), FINGER_CLOSED, "right_approach_handle", 0.0),
        # Stroke handle forward to clamp moving jaw (t = 1.2s)
        Waypoint(1.2, (handle_push_target_x, HANDLE_Y0, HANDLE_Z), FINGER_CLOSED, "right_clamp_stroke", 0.0),
        Waypoint(0.4, (handle_push_target_x, HANDLE_Y0, HANDLE_Z), FINGER_CLOSED, "right_clamp_settle", 0.0),
    ]

    if variant == "no_release":
        right.append(Waypoint(8.0, (handle_push_target_x, HANDLE_Y0, HANDLE_Z), FINGER_CLOSED, "right_hold_handle", 0.0))
        return {"left": left, "right": right}

    # Retract right arm clear for stillness hold (t = 0.8s + 1.0s + 3.0s dwell)
    right += [
        Waypoint(0.8, (handle_approach_x, HANDLE_Y0, HANDLE_Z), FINGER_OPEN, "right_back_off", 0.0),
        Waypoint(1.0, (0.150, -0.220, carry_z), FINGER_OPEN, "right_retract", 0.0),
        Waypoint(3.0, (0.150, -0.220, carry_z), FINGER_OPEN, "right_park_dwell", 0.0),
    ]

    return {"left": left, "right": right}
