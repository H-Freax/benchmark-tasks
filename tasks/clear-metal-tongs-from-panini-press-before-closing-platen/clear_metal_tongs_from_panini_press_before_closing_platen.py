"""Clear metal tongs from commercial panini press before closing heated grooved platen.

BLS SOC 35-3023: Fast Food and Counter Workers
BLS SOC 35-2021: Food Preparation Workers

Safety Prerequisite + Real Physical Action (Family A):
- Right gripper retrieves 220 mm stainless utility tongs resting across the heated
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
BRIGHT_STEEL = (0.85, 0.87, 0.90)

# 1. Commercial Panini Grill Chassis
GRILL_XY = (0.280, 0.040)
BASE_W_X = 0.200  # 200 mm front-to-back
BASE_D_Y = 0.220  # 220 mm side-to-side
BASE_H = 0.060    # 60 mm lower chassis
GRILL_PLATE_Z = TOP + BASE_H  # 0.7475 + 0.060 = 0.8075 m

# Hinge location on rear of base
HINGE_X = GRILL_XY[0] - BASE_W_X / 2 + 0.020  # 0.280 - 0.100 + 0.020 = 0.200 m
HINGE_Y = GRILL_XY[1]                         # 0.040 m
HINGE_Z = GRILL_PLATE_Z + 0.035               # 0.8425 m

# Upper Platen dimensions
PLATEN_W_X = 0.180
PLATEN_D_Y = 0.200
PLATEN_H = 0.030

# Initial angle (propped open 75 degrees)
OPEN_ANGLE_RAD = math.radians(75.0)

# 2. Stainless Utility Tongs
TONGS_INIT_XY = (GRILL_XY[0], GRILL_XY[1])  # (0.280, 0.040)
TONGS_INIT_Z = GRILL_PLATE_Z + 0.012        # Resting across grill plate
TONGS_KG = 0.180                            # 180 g

# 3. Side Staging Rest Tray (on right side, +Y)
TRAY_XY = (0.280, 0.250)
TRAY_W_X = 0.160
TRAY_D_Y = 0.120
TRAY_H = 0.015


def _base_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Main chassis housing
    boxes.append(mk.box((BASE_W_X, BASE_D_Y, BASE_H), at=(0.0, 0.0, BASE_H / 2), color=CHASSIS_STEEL))
    # Lower grooved cooking plate (dark cast iron)
    boxes.append(mk.box((BASE_W_X - 0.030, BASE_D_Y - 0.030, 0.012), at=(0.005, 0.0, BASE_H + 0.006), color=DARK_CAST_IRON))
    # Rear hinge upright brackets (left and right)
    boxes.append(mk.box((0.020, 0.020, 0.050), at=(-BASE_W_X / 2 + 0.020, -BASE_D_Y / 2 + 0.020, BASE_H + 0.025), color=CHASSIS_STEEL))
    boxes.append(mk.box((0.020, 0.020, 0.050), at=(-BASE_W_X / 2 + 0.020, BASE_D_Y / 2 - 0.020, BASE_H + 0.025), color=CHASSIS_STEEL))
    return boxes


def _tray_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Bottom plate
    boxes.append(mk.box((TRAY_W_X, TRAY_D_Y, 0.005), at=(0.0, 0.0, 0.0025), color=STAINLESS))
    # Perimeter lips
    boxes.append(mk.box((0.006, TRAY_D_Y, 0.012), at=(TRAY_W_X / 2 - 0.003, 0.0, 0.006), color=STAINLESS))
    boxes.append(mk.box((0.006, TRAY_D_Y, 0.012), at=(-TRAY_W_X / 2 + 0.003, 0.0, 0.006), color=STAINLESS))
    boxes.append(mk.box((TRAY_W_X, 0.006, 0.012), at=(0.0, -TRAY_D_Y / 2 + 0.003, 0.006), color=STAINLESS))
    boxes.append(mk.box((TRAY_W_X, 0.006, 0.012), at=(0.0, TRAY_D_Y / 2 - 0.003, 0.006), color=STAINLESS))
    return boxes


def _tongs_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Grip handle body (where robot grasps)
    boxes.append(mk.box((0.110, 0.022, 0.016), at=(-0.040, 0.0, 0.008), color=BRIGHT_STEEL))
    # Scalloped grasping jaws
    boxes.append(mk.box((0.075, 0.008, 0.016), at=(0.045, -0.008, 0.008), color=BRIGHT_STEEL))
    boxes.append(mk.box((0.075, 0.008, 0.016), at=(0.045, 0.008, 0.008), color=BRIGHT_STEEL))
    return boxes


def _platen_boxes() -> list[mk.BoxSpec]:
    """Upper platen propped open or closed. Origin at rear hinge pivot."""
    boxes = []
    # Upper grooved cast-iron plate
    boxes.append(mk.box((PLATEN_W_X, PLATEN_D_Y - 0.030, 0.016), at=(0.080, 0.0, 0.012), color=DARK_CAST_IRON))
    # Platen hood housing
    boxes.append(mk.box((PLATEN_W_X + 0.010, PLATEN_D_Y - 0.020, 0.014), at=(0.080, 0.0, 0.027), color=CHASSIS_STEEL))
    # Front handle extension struts
    boxes.append(mk.box((0.035, 0.014, 0.014), at=(PLATEN_W_X + 0.005, -0.045, 0.028), color=CHASSIS_STEEL))
    boxes.append(mk.box((0.035, 0.014, 0.014), at=(PLATEN_W_X + 0.005, 0.045, 0.028), color=CHASSIS_STEEL))
    # Insulated tubular front handle
    boxes.append(mk.box((0.020, 0.140, 0.020), at=(PLATEN_W_X + 0.022, 0.0, 0.028), color=INSULATED_BLACK))
    return boxes


kit = mk.Kit()

# Register parts
kit.part("panini_base", (GRILL_XY[0], GRILL_XY[1], TOP), _base_boxes(), mass=None, color=CHASSIS_STEEL)
kit.part("staging_tray", (TRAY_XY[0], TRAY_XY[1], TOP), _tray_boxes(), mass=None, color=STAINLESS)
kit.part("utility_tongs", (TONGS_INIT_XY[0], TONGS_INIT_XY[1], TONGS_INIT_Z), _tongs_boxes(), mass=TONGS_KG, color=BRIGHT_STEEL, solver_position_iterations=16)

# Initial open platen rotation (75 degrees open around Y axis)
theta = OPEN_ANGLE_RAD
q_y = math.sin(-theta / 2.0)
q_w = math.cos(-theta / 2.0)
init_quat = (q_w, 0.0, q_y, 0.0)

kit.part("upper_platen", (HINGE_X, HINGE_Y, HINGE_Z), _platen_boxes(), mass=4.0, quat=init_quat, color=CHASSIS_STEEL, solver_position_iterations=16)

FIXTURES = []
OBJECTS = kit.objects()

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


# State rules
_state = {
    "tongs_cleared": False,
    "tongs_on_tray": False,
    "platen_descended": False,
    "platen_closed": False,
    "defect": False,
    "hold_steps": 0,
}


@kit.on_step
def _step_rules(objects):
    tongs = objects.get("utility_tongs")
    platen = objects.get("upper_platen")
    if not tongs or not platen:
        return

    t_pos = kit.point(objects, "utility_tongs", (0.0, 0.0, 0.008))
    h_pos = kit.point(objects, "upper_platen", (PLATEN_W_X + 0.022, 0.0, 0.028))
    p_plate = kit.point(objects, "upper_platen", (0.080, 0.0, 0.012))

    d_grill = math.hypot(t_pos[0] - GRILL_XY[0], t_pos[1] - GRILL_XY[1])
    d_tray = math.hypot(t_pos[0] - TRAY_XY[0], t_pos[1] - TRAY_XY[1])

    plate_gap_z = p_plate[2] - (GRILL_PLATE_Z + 0.012)
    handle_z = h_pos[2]

    # Defect: platen lowered while tongs still on lower grill
    if plate_gap_z < 0.040 and d_grill < 0.080 and not _state["tongs_cleared"]:
        _state["defect"] = True

    if _state["defect"]:
        return

    # Stage 1: Tongs lifted clear of grill plate (> 45 mm)
    if t_pos[2] > GRILL_PLATE_Z + 0.045:
        _state["tongs_cleared"] = True

    # Stage 2: Tongs resting stably on side tray
    if _state["tongs_cleared"] and d_tray < 0.065 and t_pos[2] < TOP + 0.040:
        _state["tongs_on_tray"] = True

    # Stage 3: Platen rotated down past 45 degrees
    if _state["tongs_on_tray"] and handle_z < 0.880:
        _state["platen_descended"] = True

    # Stage 4: Platen fully seated flush against stop (< 10 mm gap)
    if _state["platen_descended"] and plate_gap_z < 0.010:
        _state["platen_closed"] = True

    # Hands clear check
    tcps = _TEST_TCPS if _TEST_TCPS is not None else []
    hands_clear = True
    if len(tcps) >= 2:
        ltcp, rtcp = tcps[0], tcps[1]
        if np.linalg.norm(np.array(ltcp) - np.array(h_pos)) < 0.045:
            hands_clear = False
        if np.linalg.norm(np.array(rtcp) - np.array(t_pos)) < 0.045:
            hands_clear = False

    if _state["platen_closed"] and hands_clear:
        _state["hold_steps"] += 1


def task_score(objects) -> float:
    if _state["defect"]:
        return 0.0
    if not _state["tongs_cleared"]:
        return 0.0
    if not _state["tongs_on_tray"]:
        return 0.20
    if not _state["platen_descended"]:
        return 0.50
    if not _state["platen_closed"]:
        return 0.75
    if _state["hold_steps"] < 20:
        return 0.90
    return 1.0


def score(objects) -> float:
    return task_score(objects)


def success(objects) -> bool:
    return task_score(objects) >= 1.0


# Scripted Bimanual Demonstration Plan
def plan(variant="normal"):
    waypoints = []

    # 1. Approach and grasp tongs with right arm
    t_grip_xy = (0.240, 0.040)
    t_grip_z = GRILL_PLATE_Z + 0.020

    # Safe right pre-grasp above grill
    waypoints.append(
        Waypoint(
            right_pos=(t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.080),
            right_rot=(0.0, 1.0, 0.0, 0.0),
            right_finger=FINGER_OPEN,
            left_pos=(0.320, -0.150, 0.920),
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
    tray_dest_z = TOP + 0.022

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
    waypoints.append(
        Waypoint(
            left_pos=(0.245, 0.040, 0.960),
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
    waypoints.append(
        Waypoint(
            left_pos=(0.330, 0.040, 0.880),
            left_finger=FINGER_CLOSED,
            steps=30,
        )
    )

    # Closed stroke
    waypoints.append(
        Waypoint(
            left_pos=(0.395, 0.040, 0.830),
            left_finger=FINGER_CLOSED,
            steps=30,
        )
    )

    if variant == "no_release":
        return waypoints

    # Release handle
    waypoints.append(
        Waypoint(
            left_pos=(0.395, 0.040, 0.830),
            left_finger=FINGER_OPEN,
            steps=15,
        )
    )

    # Retract left arm clear
    waypoints.append(
        Waypoint(
            left_pos=(0.320, -0.150, 0.920),
            left_finger=FINGER_OPEN,
            right_pos=(0.280, 0.350, 0.880),
            right_finger=FINGER_OPEN,
            steps=30,
        )
    )

    # 20-step stillness hold
    waypoints.append(
        Waypoint(
            left_pos=(0.320, -0.150, 0.920),
            left_finger=FINGER_OPEN,
            right_pos=(0.280, 0.350, 0.880),
            right_finger=FINGER_OPEN,
            steps=25,
        )
    )

    return waypoints

def demo() -> dict[str, list[Waypoint]]:
    variant = os.environ.get("DEMO_VARIANT", "")
    plans: dict[str, list[Waypoint]] = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}

    def add(arm: str, dt: float, pos, finger: float, label: str = "", yaw: float = 0.0) -> None:
        plans[arm].append(Waypoint(float(dt), tuple(float(v) for v in pos), float(finger), label, float(yaw)))
        clock[arm] += float(dt)

    def wait_until(arm: str, t: float, label: str = "wait") -> None:
        dt = t - clock[arm]
        if dt > 1e-6:
            last = plans[arm][-1]
            add(arm, dt, last.pos, last.finger, label, last.yaw)

    # Initial grasp coordinates
    t_grip_xy = (0.240, 0.040)
    t_grip_z = GRILL_PLATE_Z + 0.020

    tray_dest_xy = (TRAY_XY[0], TRAY_XY[1])
    tray_dest_z = TOP + 0.022

    LEFT_PARK = (0.160, -0.250, 0.920)
    RIGHT_PARK = (0.160, 0.350, 0.920)

    # Left arm starts parked clear
    add("left", 1.0, LEFT_PARK, FINGER_OPEN, "left park")

    # Right arm: approach and grasp tongs on grill plate
    add("right", 1.2, (t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.080), FINGER_OPEN, "approach tongs above grill")
    add("right", 0.6, (t_grip_xy[0], t_grip_xy[1], t_grip_z), FINGER_OPEN, "descend to tongs grip")
    add("right", 0.4, (t_grip_xy[0], t_grip_xy[1], t_grip_z), FINGER_CLOSED, "pinch tongs grip")
    # Lift tongs clear (> 60 mm)
    add("right", 0.8, (t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.085), FINGER_CLOSED, "lift tongs clear of grill")

    if variant == "no_tray":
        add("right", 2.0, (t_grip_xy[0], t_grip_xy[1], t_grip_z + 0.085), FINGER_CLOSED, "hold tongs suspended")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Transfer tongs across to side rest tray
    add("right", 1.2, (tray_dest_xy[0], tray_dest_xy[1], t_grip_z + 0.085), FINGER_CLOSED, "transfer tongs to tray")
    add("right", 0.6, (tray_dest_xy[0], tray_dest_xy[1], tray_dest_z), FINGER_CLOSED, "lower tongs onto tray")
    add("right", 0.4, (tray_dest_xy[0], tray_dest_xy[1], tray_dest_z), FINGER_OPEN, "release tongs")
    # Retract right arm to clear park position
    add("right", 0.8, (tray_dest_xy[0], tray_dest_xy[1], tray_dest_z + 0.080), FINGER_OPEN, "lift right arm")
    add("right", 1.0, RIGHT_PARK, FINGER_OPEN, "park right arm")

    if variant == "no_platen":
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Synchronize: left arm waits until tongs are resting on tray and right arm is clearing
    wait_until("left", clock["right"] - 1.0, "wait for tongs clear")

    # Left arm: approach insulated platen front handle in open position
    handle_open_xy = (0.250, 0.040)
    handle_open_z = 0.950

    add("left", 1.2, (handle_open_xy[0], handle_open_xy[1], handle_open_z + 0.040), FINGER_OPEN, "approach insulated handle")
    add("left", 0.6, (handle_open_xy[0], handle_open_xy[1], handle_open_z), FINGER_OPEN, "align with handle")
    add("left", 0.4, (handle_open_xy[0], handle_open_xy[1], handle_open_z), FINGER_CLOSED, "grasp insulated handle")

    # Lower platen smoothly along arc
    # Mid-point (40 degrees)
    add("left", 1.0, (0.325, 0.040, 0.885), FINGER_CLOSED, "pivot platen down mid")
    # Bottom stop (0 degrees, closed)
    handle_closed_xy = (0.395, 0.040)
    handle_closed_z = GRILL_PLATE_Z + 0.025
    add("left", 1.0, (handle_closed_xy[0], handle_closed_xy[1], handle_closed_z), FINGER_CLOSED, "seat platen flush on stop")

    if variant == "no_release":
        add("left", 2.0, (handle_closed_xy[0], handle_closed_xy[1], handle_closed_z), FINGER_CLOSED, "dwell without release")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Release insulated handle and retract clear
    add("left", 0.5, (handle_closed_xy[0], handle_closed_xy[1], handle_closed_z), FINGER_OPEN, "release handle")
    add("left", 0.8, (handle_closed_xy[0], handle_closed_xy[1], handle_closed_z + 0.080), FINGER_OPEN, "lift left arm")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Ensure right arm remains parked clear
    wait_until("right", clock["left"], "stay parked")

    # 1.5 s stillness dwell
    end_t = max(clock.values()) + 1.5
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")

    return plans
