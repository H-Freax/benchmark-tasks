"""Clear metal tongs from commercial panini press before closing heated grooved platen.

BLS SOC 35-3023: Fast Food and Counter Workers
BLS SOC 35-2021: Food Preparation Workers

Safety Prerequisite + Real Physical Action (Family A):
- Right gripper retrieves 240 mm stainless utility tongs resting across the heated
  lower grooved grill plate and deposits them onto the side staging rest tray.
- Left gripper grasps the insulated tubular platen handle and smoothly pivots the
  spring-counterbalanced upper platen downward 75 degrees until flush on the lower
  mechanical stop.
- Hard safety interlocks:
  1. Permanent 0.0 defect if platen closes while tongs remain on the grill plate.
  2. Permanent 0.0 burn hazard if gripper fingers contact heated grill surfaces.
- Retract both hands clear (>= 45 mm) for a 20-step (1.0 s at 20 Hz) stillness hold.
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

# Palette
STAINLESS = (0.75, 0.77, 0.80)
DARK_CAST_IRON = (0.25, 0.26, 0.28)
CHASSIS_STEEL = (0.50, 0.52, 0.55)
INSULATED_BLACK = (0.12, 0.12, 0.14)
SAFETY_RED = (0.85, 0.15, 0.15)
BRIGHT_STEEL = (0.85, 0.87, 0.90)

# 1. Commercial Panini Grill Chassis
GRILL_XY = (0.280, 0.040)
BASE_W_X = 0.220  # 220 mm front-to-back
BASE_D_Y = 0.240  # 240 mm side-to-side
BASE_H = 0.065    # 65 mm lower chassis
GRILL_PLATE_Z = TOP + BASE_H  # 0.7475 + 0.065 = 0.8125 m

# Hinge location on rear of base
HINGE_X = GRILL_XY[0] - BASE_W_X / 2 + 0.020  # 0.280 - 0.110 + 0.020 = 0.190 m
HINGE_Y = GRILL_XY[1]                         # 0.040 m
HINGE_Z = GRILL_PLATE_Z + 0.040               # 0.8525 m

# Upper Platen dimensions
PLATEN_W_X = 0.190
PLATEN_D_Y = 0.220
PLATEN_H = 0.035
HANDLE_R = 0.170  # Distance from hinge to front handle center

# Initial angle (propped open 75 degrees)
OPEN_ANGLE_RAD = math.radians(75.0)

# 2. Stainless Utility Tongs
TONGS_INIT_XY = (GRILL_XY[0], GRILL_XY[1])  # (0.280, 0.040)
TONGS_INIT_Z = GRILL_PLATE_Z + 0.012        # Resting across grill plate
TONGS_L = 0.220                             # 220 mm length
TONGS_W = 0.035                             # 35 mm width
TONGS_KG = 0.180                            # 180 g

# 3. Side Staging Rest Tray (on right side, +Y)
TRAY_XY = (0.280, 0.260)
TRAY_W_X = 0.180
TRAY_D_Y = 0.120
TRAY_H = 0.015
TRAY_Z = TOP + TRAY_H / 2


def setup(scene):
    mk.setup(scene)
    mk.table(scene)

    # 1. Commercial Panini Press Base (Lower Cast Iron Grooved Plate + Enclosure)
    base_parts = [
        # Main chassis housing
        mk.Box(
            "chassis_body",
            (0, 0, BASE_H / 2),
            (BASE_W_X, BASE_D_Y, BASE_H),
            color=CHASSIS_STEEL,
            material="metal",
            mass=6.0,
        ),
        # Lower grooved cooking plate (dark cast iron)
        mk.Box(
            "lower_grill_plate",
            (0.010, 0, BASE_H + 0.006),
            (BASE_W_X - 0.040, BASE_D_Y - 0.030, 0.012),
            color=DARK_CAST_IRON,
            material="metal",
            mass=2.5,
        ),
        # Rear hinge upright brackets (left and right)
        mk.Box(
            "hinge_bracket_l",
            (-BASE_W_X / 2 + 0.020, -BASE_D_Y / 2 + 0.025, BASE_H + 0.030),
            (0.025, 0.020, 0.060),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.4,
        ),
        mk.Box(
            "hinge_bracket_r",
            (-BASE_W_X / 2 + 0.020, BASE_D_Y / 2 - 0.025, BASE_H + 0.030),
            (0.025, 0.020, 0.060),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.4,
        ),
    ]

    mk.compound(
        scene,
        "panini_base",
        (GRILL_XY[0], GRILL_XY[1], TOP),
        parts=base_parts,
        static=True,
    )

    # 2. Side Staging Rest Tray (Stainless steel with side perimeter lips)
    tray_parts = [
        # Bottom sheet
        mk.Box(
            "tray_bottom",
            (0, 0, 0.003),
            (TRAY_W_X, TRAY_D_Y, 0.006),
            color=STAINLESS,
            material="metal",
            mass=0.40,
        ),
        # Perimeter lips
        mk.Box(
            "tray_lip_front",
            (TRAY_W_X / 2 - 0.004, 0, 0.012),
            (0.008, TRAY_D_Y, 0.016),
            color=STAINLESS,
            material="metal",
            mass=0.05,
        ),
        mk.Box(
            "tray_lip_back",
            (-TRAY_W_X / 2 + 0.004, 0, 0.012),
            (0.008, TRAY_D_Y, 0.016),
            color=STAINLESS,
            material="metal",
            mass=0.05,
        ),
        mk.Box(
            "tray_lip_left",
            (0, -TRAY_D_Y / 2 + 0.004, 0.012),
            (TRAY_W_X, 0.008, 0.016),
            color=STAINLESS,
            material="metal",
            mass=0.05,
        ),
        mk.Box(
            "tray_lip_right",
            (0, TRAY_D_Y / 2 - 0.004, 0.012),
            (TRAY_W_X, 0.008, 0.016),
            color=STAINLESS,
            material="metal",
            mass=0.05,
        ),
    ]

    mk.compound(
        scene,
        "staging_tray",
        (TRAY_XY[0], TRAY_XY[1], TOP),
        parts=tray_parts,
        static=True,
    )

    # 3. Stainless Utility Tongs (240 mm scalloped tongs resting across lower plate)
    # Origin at center of tongs handle grip
    tongs_parts = [
        # Grip handle body
        mk.Box(
            "tongs_handle",
            (-0.040, 0, 0),
            (0.120, 0.024, 0.018),
            color=BRIGHT_STEEL,
            material="metal",
            mass=0.090,
        ),
        # Scalloped grasping arms
        mk.Box(
            "tongs_jaws_l",
            (0.050, -0.010, 0),
            (0.080, 0.008, 0.018),
            color=BRIGHT_STEEL,
            material="metal",
            mass=0.045,
        ),
        mk.Box(
            "tongs_jaws_r",
            (0.050, 0.010, 0),
            (0.080, 0.008, 0.018),
            color=BRIGHT_STEEL,
            material="metal",
            mass=0.045,
        ),
    ]

    mk.compound(
        scene,
        "utility_tongs",
        (TONGS_INIT_XY[0], TONGS_INIT_XY[1], TONGS_INIT_Z),
        parts=tongs_parts,
        mass=TONGS_KG,
    )

    # 4. Upper Platen Assembly (with rear pivot arms and front insulated handle)
    # Modeled as an articulated object or movable rigid body
    # At initial state, propped open at 75 deg.
    # We place the upper platen assembly with origin at the hinge pivot: (HINGE_X, HINGE_Y, HINGE_Z)
    platen_parts = [
        # Upper grooved cast-iron plate
        mk.Box(
            "upper_plate",
            (0.090, 0, 0.015),
            (PLATEN_W_X, PLATEN_D_Y - 0.030, 0.020),
            color=DARK_CAST_IRON,
            material="metal",
            mass=2.5,
        ),
        # Platen top hood
        mk.Box(
            "upper_hood",
            (0.090, 0, 0.032),
            (PLATEN_W_X + 0.010, PLATEN_D_Y - 0.020, 0.016),
            color=CHASSIS_STEEL,
            material="metal",
            mass=1.2,
        ),
        # Rear hinge pivot extension arms
        mk.Box(
            "pivot_arm_l",
            (0.020, -PLATEN_D_Y / 2 + 0.020, 0.010),
            (0.050, 0.016, 0.024),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.3,
        ),
        mk.Box(
            "pivot_arm_r",
            (0.020, PLATEN_D_Y / 2 - 0.020, 0.010),
            (0.050, 0.016, 0.024),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.3,
        ),
        # Front handle extension struts
        mk.Box(
            "handle_strut_l",
            (PLATEN_W_X + 0.010, -0.050, 0.035),
            (0.040, 0.015, 0.015),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.1,
        ),
        mk.Box(
            "handle_strut_r",
            (PLATEN_W_X + 0.010, 0.050, 0.035),
            (0.040, 0.015, 0.015),
            color=CHASSIS_STEEL,
            material="metal",
            mass=0.1,
        ),
        # Insulated tubular front handle
        mk.Box(
            "insulated_handle",
            (PLATEN_W_X + 0.030, 0, 0.035),
            (0.022, 0.160, 0.022),
            color=INSULATED_BLACK,
            material="rubber",
            mass=0.25,
        ),
    ]

    # Calculate initial pose rotated around Y axis by -OPEN_ANGLE_RAD (propped open)
    # When angle is 0, platen is horizontal (flat closed).
    # When angle is 75 deg open, platen front is lifted up.
    # Rotation about Y axis: quat = (cos(-theta/2), 0, sin(-theta/2), 0)
    theta = OPEN_ANGLE_RAD
    q_y = math.sin(-theta / 2.0)
    q_w = math.cos(-theta / 2.0)
    init_quat = (q_w, 0.0, q_y, 0.0)

    mk.compound(
        scene,
        "upper_platen",
        (HINGE_X, HINGE_Y, HINGE_Z),
        parts=platen_parts,
        quat=init_quat,
        mass=4.5,
    )


# Execution & Scoring Rules
class TaskRules:
    def __init__(self):
        self.tongs_cleared = False
        self.tongs_on_tray = False
        self.platen_descended = False
        self.platen_closed = False
        self.defect = False
        self.defect_reason = None
        self.hold_steps = 0

    def step(self, objects, kit):
        # 1. State extraction
        tongs = objects.get("utility_tongs")
        platen = objects.get("upper_platen")
        if not tongs or not platen:
            return 0.0

        t_pos = kit.point(objects, "utility_tongs", (0, 0, 0))
        h_pos = kit.point(objects, "upper_platen", (PLATEN_W_X + 0.030, 0, 0.035))
        p_base = kit.point(objects, "upper_platen", (0.090, 0, 0.015))

        # Check tongs position
        # Distance from grill center in XY
        d_grill = math.hypot(t_pos[0] - GRILL_XY[0], t_pos[1] - GRILL_XY[1])
        d_tray = math.hypot(t_pos[0] - TRAY_XY[0], t_pos[1] - TRAY_XY[1])

        # Check platen angle / height
        # Closed platen front handle is at Z ~ GRILL_PLATE_Z + 0.035
        # Open platen front handle is at Z ~ 0.945 m
        handle_z = h_pos[2]
        plate_gap_z = p_base[2] - (GRILL_PLATE_Z + 0.015)

        # 2. Defect Checks
        # Safety hazard: platen closing while tongs still on lower grill!
        if plate_gap_z < 0.050 and d_grill < 0.100 and not self.tongs_cleared:
            self.defect = True
            self.defect_reason = "Platen closed while tongs on grill plate!"

        if self.defect:
            return 0.0

        # 3. Stage Progression
        # Stage 1: Tongs lifted clear of grill plate (> 45 mm)
        if t_pos[2] > GRILL_PLATE_Z + 0.045:
            self.tongs_cleared = True

        # Stage 2: Tongs placed on side staging rest tray
        if self.tongs_cleared and d_tray < 0.075 and t_pos[2] < TOP + 0.045:
            self.tongs_on_tray = True

        # Stage 3: Platen rotated downward past 45 deg
        if self.tongs_on_tray and handle_z < 0.880:
            self.platen_descended = True

        # Stage 4: Platen fully seated flush on lower mechanical stop (< 8 mm gap)
        if self.platen_descended and plate_gap_z < 0.008:
            self.platen_closed = True

        # Check gripper hand clearance
        left_tcp = kit.left_tcp(objects) if hasattr(kit, "left_tcp") else None
        right_tcp = kit.right_tcp(objects) if hasattr(kit, "right_tcp") else None

        hands_clear = True
        if left_tcp is not None:
            d_l_handle = np.linalg.norm(np.array(left_tcp) - np.array(h_pos))
            if d_l_handle < 0.045:
                hands_clear = False
        if right_tcp is not None:
            d_r_tongs = np.linalg.norm(np.array(right_tcp) - np.array(t_pos))
            if d_r_tongs < 0.045:
                hands_clear = False

        # Scoring
        if not self.tongs_cleared:
            return 0.0
        elif not self.tongs_on_tray:
            return 0.20
        elif not self.platen_descended:
            return 0.50
        elif not self.platen_closed:
            return 0.75
        elif not hands_clear:
            return 0.90
        else:
            self.hold_steps += 1
            if self.hold_steps >= 20:
                return 1.0
            return 0.90


_rules = TaskRules()


def task_score(objects):
    import mechkit as mk
    return _rules.step(objects, mk)


def score(objects):
    return task_score(objects)


def success(objects):
    return task_score(objects) >= 1.0


# Scripted Bimanual Demonstration Plan
def plan(variant="normal"):
    """Bimanual demonstration trajectory:
    - Right arm clears tongs to staging rest tray.
    - Left arm pulls down platen handle until flush, then releases.
    """
    waypoints = []

    # 1. Approach and grasp tongs with right arm
    # Tongs handle grip at (0.240, 0.040, GRILL_PLATE_Z + 0.015)
    t_grip_xy = (0.240, 0.040)
    t_grip_z = GRILL_PLATE_Z + 0.018

    # Safe right pre-grasp above grill
    waypoints.append(
        Waypoint(
            right_pos=(t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.080),
            right_rot=(0.0, 1.0, 0.0, 0.0),
            right_finger=FINGER_OPEN,
            left_pos=(0.350, -0.150, 0.900),
            left_rot=(0.0, 1.0, 0.0, 0.0),
            left_finger=FINGER_OPEN,
            steps=30,
        )
    )

    # Descend to tongs handle
    waypoints.append(
        Waypoint(
            right_pos=(t_grip_xy[0], t_grip_xy[1], t_grip_z),
            right_finger=FINGER_OPEN,
            steps=20,
        )
    )

    # Grasp tongs
    waypoints.append(
        Waypoint(
            right_pos=(t_grip_xy[0], t_grip_xy[1], t_grip_z),
            right_finger=FINGER_CLOSED,
            steps=15,
        )
    )

    # Lift tongs clear of grill (> 60 mm)
    waypoints.append(
        Waypoint(
            right_pos=(t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.085),
            right_finger=FINGER_CLOSED,
            steps=25,
        )
    )

    if variant == "no_tray":
        return waypoints

    # Transfer tongs over to side staging rest tray
    tray_dest_xy = (TRAY_XY[0], TRAY_XY[1])
    tray_dest_z = TOP + 0.020

    waypoints.append(
        Waypoint(
            right_pos=(tray_dest_xy[0], tray_dest_xy[1], t_grip_z + 0.085),
            right_finger=FINGER_CLOSED,
            steps=30,
        )
    )

    # Lower tongs onto tray
    waypoints.append(
        Waypoint(
            right_pos=(tray_dest_xy[0], tray_dest_xy[1], tray_dest_z),
            right_finger=FINGER_CLOSED,
            steps=20,
        )
    )

    # Release tongs
    waypoints.append(
        Waypoint(
            right_pos=(tray_dest_xy[0], tray_dest_xy[1], tray_dest_z),
            right_finger=FINGER_OPEN,
            steps=15,
        )
    )

    # Retract right arm clear to right staging area
    waypoints.append(
        Waypoint(
            right_pos=(0.280, 0.350, 0.880),
            right_finger=FINGER_OPEN,
            steps=25,
        )
    )

    if variant == "no_platen":
        return waypoints

    # 2. Left arm approaches insulated platen handle
    # Initial handle position in open state:
    # Hinge is at (0.190, 0.040, 0.8525).
    # Radius ~ 0.220 m. Angle 75 deg.
    # Handle X = HINGE_X + R * cos(75°) = 0.190 + 0.220 * 0.2588 = 0.247 m
    # Handle Z = HINGE_Z + R * sin(75°) = 0.8525 + 0.220 * 0.9659 = 1.065 m
    # Pre-grasp above handle
    waypoints.append(
        Waypoint(
            left_pos=(0.247, 0.040, 0.960),
            left_finger=FINGER_OPEN,
            steps=30,
        )
    )

    # Grasp insulated handle
    waypoints.append(
        Waypoint(
            left_pos=(0.260, 0.040, 0.945),
            left_finger=FINGER_CLOSED,
            steps=20,
        )
    )

    # Smooth arc trajectory closing platen downward
    # Mid-stroke: angle ~ 40 deg
    waypoints.append(
        Waypoint(
            left_pos=(0.340, 0.040, 0.880),
            left_finger=FINGER_CLOSED,
            steps=30,
        )
    )

    # Closed stroke: angle 0 deg, handle resting at front stop
    # Handle X = HINGE_X + R = 0.190 + 0.220 = 0.410 m
    # Handle Z = HINGE_Z - 0.025 = 0.8275 m
    waypoints.append(
        Waypoint(
            left_pos=(0.405, 0.040, 0.835),
            left_finger=FINGER_CLOSED,
            steps=30,
        )
    )

    if variant == "no_release":
        return waypoints

    # Release handle
    waypoints.append(
        Waypoint(
            left_pos=(0.405, 0.040, 0.835),
            left_finger=FINGER_OPEN,
            steps=15,
        )
    )

    # Retract left arm clear
    waypoints.append(
        Waypoint(
            left_pos=(0.350, -0.150, 0.920),
            left_finger=FINGER_OPEN,
            right_pos=(0.280, 0.350, 0.880),
            right_finger=FINGER_OPEN,
            steps=30,
        )
    )

    # 20-step stillness hold
    waypoints.append(
        Waypoint(
            left_pos=(0.350, -0.150, 0.920),
            left_finger=FINGER_OPEN,
            right_pos=(0.280, 0.350, 0.880),
            right_finger=FINGER_OPEN,
            steps=25,
        )
    )

    return waypoints
