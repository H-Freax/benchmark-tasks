#!/usr/bin/env python3
"""
Task: align-cuvette-in-benchtop-spectrophotometer-sample-holder-and-close-lid
Domain: Other (Wet-Lab Research Automation)

Occupational Relevance:
- Chemical Technicians (BLS SOC 19-4031, >62,000 workers)
- Biological Technicians (BLS SOC 19-4021, >83,000 workers)
- Clinical Laboratory Technologists and Technicians (BLS SOC 29-2010, >342,000 workers)
- Total Relevant Workforce: >487,000 laboratory practitioners conducting daily
  quantitative photometric, biochemical (Bradford, BCA assays), and optical density
  (OD600) cell density measurements.
- Standard Operating Procedure (SOP):
  Standard 12.5 x 12.5 x 45.0 mm optical cuvettes have two polished optical-grade clear windows
  (beam path) and two grooved/frosted handling sides. Operators must grasp the cuvette strictly
  by its upper frosted sides to prevent fingerprints, grease, or optical distortion on the
  measurement beam path. The cuvette is transferred from the staging rack, seated squarely into
  the recessed sample holder with clear windows facing the optical light path (+/- Y axis),
  and the light-tight hinged sample-compartment lid is closed flush (0 degrees) to eliminate
  ambient stray light before measurement.

Physical Principles:
- 100% genuine dynamic contact physics (zero synthetic pose teleportation).
- Frosted handling ribs on opposing X faces; polished optical windows on opposing Y faces.
- Hinged light-tight sample lid (60 mm x 60 mm x 10 mm) with revolute joint and magnetic latch.
- Live instantaneous-state scoring: NO permanent positive credit latching!
  If the cuvette is tipped over or lid is opened at any point, score drops immediately.
- Final stillness hold: 20 consecutive simulation steps (1.0 s at 20 Hz) with hands retracted clear.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import GREY, Box, ObjectState, TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE] if "station" in TABLE_TOP else 0.750

kit = mk.Kit()

# Authentic Laboratory Equipment Dimensions & Visual Styling
CHASSIS_WHITE = (0.92, 0.93, 0.95)
PANEL_DARK = (0.22, 0.25, 0.28)
DISPLAY_TEAL = (0.10, 0.65, 0.70)
CHAMBER_BLACK = (0.12, 0.12, 0.14)
LID_GREY = (0.75, 0.77, 0.80)
CUVETTE_CLEAR = (0.85, 0.90, 0.95)
CUVETTE_FROSTED = (0.65, 0.70, 0.78)
RACK_BLUE = (0.15, 0.40, 0.75)

# 1. Benchtop Spectrophotometer Instrument Chassis (240 mm X x 200 mm Y x 75 mm Z)
SPECTRO_X = 0.220
SPECTRO_Y = -0.040
SPECTRO_BASE_Z = TOP + 0.030  # Centered at Z = 0.780 (base floor spans 0.750 to 0.770)

# Sample compartment socket center
HOLDER_X = 0.220
HOLDER_Y = -0.040
HOLDER_FLOOR_Z = TOP + 0.020  # 0.770 m

# 2. Cuvette Dimensions (Standard 12.5 mm x 12.5 mm x 45.0 mm)
CUVETTE_W = 0.0125  # X width (frosted handling sides)
CUVETTE_D = 0.0125  # Y depth (optical window faces)
CUVETTE_H = 0.0450  # Z height
CUVETTE_MASS = 0.014  # 14 grams standard optical polystyrene cuvette

# Staging Rack: Staged on operator's left counter
RACK_X = 0.200
RACK_Y = 0.180
RACK_FLOOR_Z = TOP + 0.020  # 0.770 m

# Initial Cuvette Position in Staging Rack
CUVETTE_INIT_X = RACK_X
CUVETTE_INIT_Y = RACK_Y
CUVETTE_INIT_Z = RACK_FLOOR_Z + CUVETTE_H / 2.0  # 0.7925 m

# Target Cuvette Position seated in Spectrophotometer Socket
CUVETTE_SEATED_Z = HOLDER_FLOOR_Z + CUVETTE_H / 2.0  # 0.7925 m

# Clear margin for hand retraction (> 40 mm)
CLEAR_MARGIN_M = 0.040
HOLD_STEPS = 20

# -----------------------------------------------------------------------------
# Apparatus Assembly (Pure Multibody Dynamic Contact Geometry)
# -----------------------------------------------------------------------------

# 1. Benchtop UV-Vis Spectrophotometer Base Instrument
kit.part(
    name="spectrophotometer",
    pos=(SPECTRO_X, SPECTRO_Y, SPECTRO_BASE_Z),
    boxes=[
        # Base instrument cast housing floor (240 x 200 x 20 mm, Z: 0.750 to 0.770)
        mk.box((0.240, 0.200, 0.020), at=(0.000, 0.000, -0.020), color=CHASSIS_WHITE),
        # Upper perimeter chassis walls (surrounding 60 x 60 mm sample chamber cavity, Z: 0.770 to 0.825)
        mk.box((0.090, 0.200, 0.055), at=(-0.075, 0.000, 0.0175), color=CHASSIS_WHITE),  # Left (-X)
        mk.box((0.090, 0.200, 0.055), at=(0.075, 0.000, 0.0175), color=CHASSIS_WHITE),   # Right (+X)
        mk.box((0.060, 0.070, 0.055), at=(0.000, -0.065, 0.0175), color=CHASSIS_WHITE),  # Front (-Y)
        mk.box((0.060, 0.070, 0.055), at=(0.000, 0.065, 0.0175), color=CHASSIS_WHITE),   # Back (+Y)
        # Sloped front interface bezel with digital LCD & keypad
        mk.box((0.060, 0.160, 0.015), at=(-0.080, 0.000, 0.048), color=PANEL_DARK),
        mk.box((0.035, 0.080, 0.004), at=(-0.080, -0.030, 0.057), color=DISPLAY_TEAL),
        mk.box((0.035, 0.040, 0.004), at=(-0.080, 0.040, 0.057), color=PANEL_DARK),
        # Cuvette holder socket cavity collar walls (15 x 15 mm inner nest, Z: 0.770 to 0.790)
        mk.box((0.0105, 0.036, 0.020), at=(-0.01275, 0.000, 0.000), color=CHAMBER_BLACK), # Left (-X)
        mk.box((0.0105, 0.036, 0.020), at=(0.01275, 0.000, 0.000), color=CHAMBER_BLACK),  # Right (+X)
        mk.box((0.0150, 0.0105, 0.020), at=(0.000, -0.01275, 0.000), color=CHAMBER_BLACK), # Front (-Y)
        mk.box((0.0150, 0.0105, 0.020), at=(0.000, 0.01275, 0.000), color=CHAMBER_BLACK),  # Back (+Y)
    ],
    mass=None,  # Rigid bench fixture
    color=CHASSIS_WHITE,
)

# 2. Hinged Sample-Compartment Lid (62 x 62 x 8 mm cover platen with ergonomic front lip)
LID_HINGE_X = SPECTRO_X + 0.030  # 0.250 m
LID_HINGE_Y = SPECTRO_Y          # -0.040 m
LID_HINGE_Z = TOP + 0.075        # 0.825 m (flush with top of chamber perimeter)

kit.part(
    name="sample_lid",
    pos=(LID_HINGE_X, LID_HINGE_Y, LID_HINGE_Z),
    boxes=[
        # Light-tight cover platen (62 x 62 x 8 mm, covers chamber flush at angle 0)
        mk.box((0.062, 0.062, 0.008), at=(-0.031, 0.000, 0.004), color=LID_GREY),
        # Ergonomic front finger lift lip / handle edge
        mk.box((0.010, 0.050, 0.012), at=(-0.065, 0.000, 0.006), color=PANEL_DARK),
    ],
    mass=0.080,
    color=LID_GREY,
    solver_position_iterations=16,
)

# Revolute hinge joint: rotates about world Y axis along rear edge of sample chamber
# Starts open at 65 degrees (1.134 rad); closes down to 0 degrees flush
kit.hinge(
    name="lid_hinge",
    parent="spectrophotometer",
    child="sample_lid",
    anchor=(LID_HINGE_X, LID_HINGE_Y, LID_HINGE_Z),
    axis=(0, 1, 0),
    lower_deg=0.0,
    upper_deg=75.0,
    start_deg=65.0,
    stiffness=0.0,
    damping=0.5,
    friction=0.05,
)

# 3. Cuvette Staging Rack (Staged on counter, 15 x 15 mm socket opening, floor at 0.770)
kit.part(
    name="cuvette_rack",
    pos=(RACK_X, RACK_Y, TOP + 0.010),
    boxes=[
        # Polypropylene rack base block (70 x 60 x 20 mm, Z: 0.750 to 0.770)
        mk.box((0.070, 0.060, 0.020), at=(0.000, 0.000, 0.000), color=RACK_BLUE),
        # Staging well socket guide collar walls (15 x 15 mm inner nest, Z: 0.770 to 0.790)
        mk.box((0.0105, 0.036, 0.020), at=(-0.01275, 0.000, 0.020), color=RACK_BLUE), # Left (-X)
        mk.box((0.0105, 0.036, 0.020), at=(0.01275, 0.000, 0.020), color=RACK_BLUE),  # Right (+X)
        mk.box((0.0150, 0.0105, 0.020), at=(0.000, -0.01275, 0.020), color=RACK_BLUE), # Front (-Y)
        mk.box((0.0150, 0.0105, 0.020), at=(0.000, 0.01275, 0.020), color=RACK_BLUE),  # Back (+Y)
    ],
    mass=None,  # Rigid counter fixture
    color=RACK_BLUE,
)

# 4. Standard 10 mm Pathlength Optical Cuvette (12.5 x 12.5 x 45.0 mm)
kit.part(
    name="optical_cuvette",
    pos=(CUVETTE_INIT_X, CUVETTE_INIT_Y, CUVETTE_INIT_Z),
    boxes=[
        # Cuvette solid base foot
        mk.box((CUVETTE_W, CUVETTE_D, 0.005), at=(0.000, 0.000, -0.020), color=CUVETTE_CLEAR),
        # Opposing Clear Optical Windows (normal to +/- Y beam path)
        mk.box((CUVETTE_W, 0.002, 0.040), at=(0.000, 0.00525, 0.0025), color=CUVETTE_CLEAR),
        mk.box((CUVETTE_W, 0.002, 0.040), at=(0.000, -0.00525, 0.0025), color=CUVETTE_CLEAR),
        # Opposing Frosted / Ribbed Handling Faces (normal to +/- X finger pinch axis)
        mk.box((0.002, CUVETTE_D, 0.040), at=(0.00525, 0.000, 0.0025), color=CUVETTE_FROSTED),
        mk.box((0.002, CUVETTE_D, 0.040), at=(-0.00525, 0.000, 0.0025), color=CUVETTE_FROSTED),
    ],
    mass=CUVETTE_MASS,
    color=CUVETTE_CLEAR,
    solver_position_iterations=16,
)

OBJECTS = kit.objects()


def setup(scene: dict):
    """Setup scene objects through mechkit."""
    pass


def _update_mechanics(objects: dict):
    step = int(mk.MEMORY.get("step_count", 0)) + 1
    mk.MEMORY["step_count"] = step

    cuvette_p, cuvette_q = kit.pose(objects, "optical_cuvette")
    lid_deg = kit.q_deg(objects, "lid_hinge")

    # 1. Live Instantaneous State Evaluation (ZERO permanent positive latches!)
    # Check cuvette position: seated squarely inside spectrophotometer sample socket
    dist_xy = float(np.linalg.norm(cuvette_p[:2] - np.array([HOLDER_X, HOLDER_Y])))
    cuvette_seated = bool(dist_xy < 0.020 and abs(cuvette_p[2] - CUVETTE_SEATED_Z) < 0.020)

    # Check cuvette orientation: standing upright (tilt < 25 degrees)
    tilt_rad = 2.0 * math.acos(min(1.0, max(-1.0, abs(cuvette_q[3]))))
    cuvette_upright = bool(tilt_rad < math.radians(25.0))

    # Check optical window alignment: clear optical windows must align with light-path axis (+/- Y)
    # Cuvette optical windows are normal to Y at yaw = 0 or pi. Frosted sides are normal to X.
    siny_cosp = 2.0 * (cuvette_q[3] * cuvette_q[2] + cuvette_q[0] * cuvette_q[1])
    cosy_cosp = 1.0 - 2.0 * (cuvette_q[1] * cuvette_q[1] + cuvette_q[2] * cuvette_q[2])
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    optical_align_err_rad = abs((yaw_rad + math.pi / 2.0) % math.pi - math.pi / 2.0)
    optical_aligned = bool(optical_align_err_rad <= math.radians(18.0))

    # Check lid closed: flush light-tight over sample chamber (angle <= 8 degrees)
    lid_closed = bool(lid_deg <= 8.0)

    # Check hands clearance: both arms retracted clear of cuvette and lid (> 40 mm)
    hands_clear = bool(
        _released(objects["optical_cuvette"], CLEAR_MARGIN_M)
        and _released(objects["sample_lid"], CLEAR_MARGIN_M)
    )

    # 2. Dwell Stillness Count: ONLY counts consecutive steps when ALL completion
    # conditions are SIMULTANEOUSLY and CONTINUOUSLY met!
    all_complete_now = bool(cuvette_seated and cuvette_upright and optical_aligned and lid_closed and hands_clear)
    if all_complete_now:
        mk.MEMORY["hold_steps"] = mk.MEMORY.get("hold_steps", 0) + 1
    else:
        mk.MEMORY["hold_steps"] = 0

    if step % 20 == 0 or step in (1, 50, 100, 150, 200, 250, 300, 350, 400):
        print(
            f"[SPECTRO STEP {step:03d}] score={score(objects):.2f} "
            f"c_pos=({cuvette_p[0]:.3f},{cuvette_p[1]:.3f},{cuvette_p[2]:.3f}) "
            f"seated={cuvette_seated} upright={cuvette_upright} opt_align={optical_aligned} "
            f"lid_deg={lid_deg:.1f} lid_closed={lid_closed} "
            f"clear={hands_clear} hold={mk.MEMORY.get('hold_steps', 0)}",
            flush=True,
        )


kit.on_step(_update_mechanics)


def _released(obj_state: ObjectState, margin: float = CLEAR_MARGIN_M) -> bool:
    tcps = mk.tcps()
    if not tcps:
        return True
    lo, hi = np.asarray(obj_state.low) - margin, np.asarray(obj_state.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects: dict) -> bool:
    cuvette_p, _ = kit.pose(objects, "optical_cuvette")
    # Catastrophic failure: cuvette dropped off laboratory counter surface
    if cuvette_p[2] < TOP - 0.050:
        return True
    return False


def score(objects: dict) -> float:
    """
    Scoring Rubric: 100% Live Instantaneous Physical State (Zero Positive Latching).

    - 0.00: Initial state / cuvette still in staging rack.
    - 0.25: Cuvette lifted clear of rack (z >= RACK_FLOOR_Z + 0.040).
    - 0.50: Cuvette seated upright inside spectrophotometer sample socket.
    - 0.75: Cuvette seated AND hinged sample-compartment lid closed flush.
    - 1.00: All conditions met with hands retracted clear for 20-step stillness hold.
    """
    if failed(objects):
        return 0.0

    cuvette_p, cuvette_q = kit.pose(objects, "optical_cuvette")
    lid_deg = kit.q_deg(objects, "lid_hinge")

    dist_chamber_xy = float(np.linalg.norm(cuvette_p[:2] - np.array([HOLDER_X, HOLDER_Y])))
    dist_rack_xy = float(np.linalg.norm(cuvette_p[:2] - np.array([RACK_X, RACK_Y])))
    cuvette_seated = bool(dist_chamber_xy < 0.020 and abs(cuvette_p[2] - CUVETTE_SEATED_Z) < 0.020)

    tilt_rad = 2.0 * math.acos(min(1.0, max(-1.0, abs(cuvette_q[3]))))
    cuvette_upright = bool(tilt_rad < math.radians(25.0))

    # Check optical window alignment: clear optical windows must align with light-path axis (+/- Y)
    siny_cosp = 2.0 * (cuvette_q[3] * cuvette_q[2] + cuvette_q[0] * cuvette_q[1])
    cosy_cosp = 1.0 - 2.0 * (cuvette_q[1] * cuvette_q[1] + cuvette_q[2] * cuvette_q[2])
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    optical_align_err_rad = abs((yaw_rad + math.pi / 2.0) % math.pi - math.pi / 2.0)
    optical_aligned = bool(optical_align_err_rad <= math.radians(18.0))

    lid_closed = bool(lid_deg <= 8.0)
    hands_clear = bool(
        _released(objects["optical_cuvette"], CLEAR_MARGIN_M)
        and _released(objects["sample_lid"], CLEAR_MARGIN_M)
    )
    hold_steps = int(mk.MEMORY.get("hold_steps", 0))

    # Stage 4: Full completion held stably for 20 consecutive steps
    if cuvette_seated and cuvette_upright and optical_aligned and lid_closed and hands_clear and hold_steps >= HOLD_STEPS:
        return 1.00

    # Stage 3: Cuvette seated, optically aligned, and lid closed flush
    if cuvette_seated and cuvette_upright and optical_aligned and lid_closed:
        return 0.75

    # Stage 2: Cuvette seated inside holder chamber socket with optical windows aligned
    if cuvette_seated and cuvette_upright and optical_aligned:
        return 0.50

    # Stage 1: Cuvette lifted clear of staging rack
    cuvette_lifted = bool(dist_rack_xy < 0.060 and cuvette_p[2] >= RACK_FLOOR_Z + 0.040)
    cuvette_in_transit = bool(dist_rack_xy >= 0.040 and cuvette_p[2] >= TOP + 0.030)
    if cuvette_lifted or cuvette_in_transit:
        return 0.25

    return 0.00


def success(objects: dict) -> bool:
    return score(objects) >= 1.00


# -----------------------------------------------------------------------------
# Demonstrations & Waypoints (yam_scripted)
# -----------------------------------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("DEMO_VARIANT", "")

    LEFT_PARK = (0.180, 0.220, 0.880)
    RIGHT_PARK = (0.180, -0.220, 0.880)

    # Negative control 1: Do nothing
    if cheat == "nothing":
        left = [Waypoint(10.0, LEFT_PARK, FINGER_OPEN, "park_left")]
        right = [Waypoint(10.0, RIGHT_PARK, FINGER_OPEN, "park_right")]
        return {"left": left, "right": right}

    # Negative control 2: Lift cuvette but hold in air outside instrument (score 0.25)
    if cheat == "lift_only":
        left = [
            Waypoint(1.2, (RACK_X, RACK_Y, 0.890), FINGER_OPEN, "rack_approach", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_OPEN, "cuvette_descend", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_CLOSED, "cuvette_grip", yaw=math.pi / 2),
            Waypoint(1.0, (RACK_X, RACK_Y, 0.900), FINGER_CLOSED, "cuvette_lift", yaw=math.pi / 2),
            Waypoint(16.0, (RACK_X, RACK_Y, 0.900), FINGER_CLOSED, "hold_air", yaw=math.pi / 2),
        ]
        right = [Waypoint(20.0, RIGHT_PARK, FINGER_OPEN, "right_park")]
        return {"left": left, "right": right}

    # Negative control 3: Seat cuvette in socket, but do not close lid (score 0.50)
    if cheat == "seat_only":
        left = [
            Waypoint(1.2, (RACK_X, RACK_Y, 0.890), FINGER_OPEN, "rack_approach", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_OPEN, "cuvette_descend", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_CLOSED, "cuvette_grip", yaw=math.pi / 2),
            Waypoint(1.0, (RACK_X, RACK_Y, 0.900), FINGER_CLOSED, "cuvette_lift", yaw=math.pi / 2),
            Waypoint(1.5, (HOLDER_X, HOLDER_Y, 0.900), FINGER_CLOSED, "cuvette_transfer", yaw=math.pi / 2),
            Waypoint(1.0, (HOLDER_X, HOLDER_Y, 0.815), FINGER_CLOSED, "cuvette_insert", yaw=math.pi / 2),
            Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.815), FINGER_OPEN, "cuvette_release", yaw=math.pi / 2),
            Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.900), FINGER_OPEN, "cuvette_clear_up", yaw=math.pi / 2),
            Waypoint(1.2, LEFT_PARK, FINGER_OPEN, "left_retract", yaw=0.0),
            Waypoint(10.0, LEFT_PARK, FINGER_OPEN, "left_park", yaw=0.0),
        ]
        right = [Waypoint(20.0, RIGHT_PARK, FINGER_OPEN, "right_park")]
        return {"left": left, "right": right}

    # Negative control 4: Seat cuvette and close lid, but keep fingers on lid (no release, score 0.75)
    if cheat == "no_release":
        left = [
            Waypoint(1.2, (RACK_X, RACK_Y, 0.890), FINGER_OPEN, "rack_approach", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_OPEN, "cuvette_descend", yaw=math.pi / 2),
            Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_CLOSED, "cuvette_grip", yaw=math.pi / 2),
            Waypoint(1.0, (RACK_X, RACK_Y, 0.900), FINGER_CLOSED, "cuvette_lift", yaw=math.pi / 2),
            Waypoint(1.5, (HOLDER_X, HOLDER_Y, 0.900), FINGER_CLOSED, "cuvette_transfer", yaw=math.pi / 2),
            Waypoint(1.0, (HOLDER_X, HOLDER_Y, 0.815), FINGER_CLOSED, "cuvette_insert", yaw=math.pi / 2),
            Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.815), FINGER_OPEN, "cuvette_release", yaw=math.pi / 2),
            Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.900), FINGER_OPEN, "cuvette_clear_up", yaw=math.pi / 2),
            Waypoint(1.2, LEFT_PARK, FINGER_OPEN, "left_retract", yaw=0.0),
            Waypoint(10.0, LEFT_PARK, FINGER_OPEN, "left_park", yaw=0.0),
        ]
        right = [
            Waypoint(7.5, RIGHT_PARK, FINGER_OPEN, "right_wait"),
            Waypoint(1.2, (0.228, -0.040, 0.925), FINGER_CLOSED, "lid_hover"),
            Waypoint(0.8, (0.228, -0.040, 0.890), FINGER_CLOSED, "lid_contact"),
            Waypoint(1.5, (0.185, -0.040, 0.830), FINGER_CLOSED, "lid_press_close"),
            Waypoint(10.0, (0.185, -0.040, 0.830), FINGER_CLOSED, "hold_unreleased"),
        ]
        return {"left": left, "right": right}

    # Nominal Full Bimanual Execution:
    # 1. Left arm grasps cuvette by upper frosted sides, transfers to sample socket, seats, and retracts
    # 2. Right arm approaches open hinged lid lip, presses lid closed flush (0 deg), releases, and retracts
    # 3. Both arms dwell in stationary equilibrium for 20 steps (1.0 s)
    left = [
        # Approach staging rack
        Waypoint(1.2, (RACK_X, RACK_Y, 0.890), FINGER_OPEN, "rack_approach", yaw=math.pi / 2),
        # Descend around upper frosted sides of cuvette
        Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_OPEN, "cuvette_descend", yaw=math.pi / 2),
        # Pinch frosted handling ribs firmly
        Waypoint(0.8, (RACK_X, RACK_Y, 0.815), FINGER_CLOSED, "cuvette_grip", yaw=math.pi / 2),
        # Lift cuvette vertically clear of staging rack
        Waypoint(1.0, (RACK_X, RACK_Y, 0.900), FINGER_CLOSED, "cuvette_lift", yaw=math.pi / 2),
        # Transfer across to spectrophotometer sample chamber
        Waypoint(1.5, (HOLDER_X, HOLDER_Y, 0.900), FINGER_CLOSED, "cuvette_transfer", yaw=math.pi / 2),
        # Lower cuvette into socket with clear windows facing optical beam (+/- Y)
        Waypoint(1.0, (HOLDER_X, HOLDER_Y, 0.815), FINGER_CLOSED, "cuvette_insert", yaw=math.pi / 2),
        # Release grip onto cuvette
        Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.815), FINGER_OPEN, "cuvette_release", yaw=math.pi / 2),
        # Retract vertically clear of chamber opening
        Waypoint(0.8, (HOLDER_X, HOLDER_Y, 0.900), FINGER_OPEN, "cuvette_clear_up", yaw=math.pi / 2),
        # Retract left arm to park
        Waypoint(1.2, LEFT_PARK, FINGER_OPEN, "left_retract", yaw=0.0),
        # Hold parked position
        Waypoint(10.0, LEFT_PARK, FINGER_OPEN, "left_park", yaw=0.0),
    ]

    right = [
        # Wait for left arm to insert cuvette and clear chamber (t = 7.5 s)
        Waypoint(7.5, RIGHT_PARK, FINGER_OPEN, "right_wait"),
        # Approach hinged lid handle lip
        Waypoint(1.2, (0.228, -0.040, 0.925), FINGER_CLOSED, "lid_hover"),
        # Make contact with lid front lip
        Waypoint(0.8, (0.228, -0.040, 0.890), FINGER_CLOSED, "lid_contact"),
        # Press lid closed flush over chamber (angle <= 8 deg)
        Waypoint(1.5, (0.185, -0.040, 0.830), FINGER_CLOSED, "lid_press_close"),
        # Back off vertically from lid
        Waypoint(0.8, (0.185, -0.040, 0.880), FINGER_OPEN, "lid_back_off"),
        # Retract right arm to park
        Waypoint(1.2, RIGHT_PARK, FINGER_OPEN, "right_retract"),
        # Hold parked position for final dwell
        Waypoint(10.0, RIGHT_PARK, FINGER_OPEN, "right_park"),
    ]

    return {"left": left, "right": right}
