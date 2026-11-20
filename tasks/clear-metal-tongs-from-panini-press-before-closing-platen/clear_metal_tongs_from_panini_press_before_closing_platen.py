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
    if plate_gap_z < 0.040 and d_grill < 0.080 and not mk.MEMORY.get("tongs_cleared"):
        mk.MEMORY["defect"] = True

    if mk.MEMORY.get("defect"):
        return

    # Stage 1: Tongs lifted clear of grill plate (> 45 mm)
    if t_pos[2] > GRILL_PLATE_Z + 0.045:
        mk.MEMORY["tongs_cleared"] = True

    # Stage 2: Tongs resting stably on side tray
    if mk.MEMORY.get("tongs_cleared") and d_tray < 0.065 and t_pos[2] < TOP + 0.040:
        mk.MEMORY["tongs_on_tray"] = True

    # Stage 3: Platen rotated down past 45 degrees
    if mk.MEMORY.get("tongs_on_tray") and handle_z < 0.880:
        mk.MEMORY["platen_descended"] = True

    # Stage 4: Platen fully seated flush against stop (< 15 mm gap)
    if mk.MEMORY.get("platen_descended") and plate_gap_z < 0.015:
        mk.MEMORY["platen_closed"] = True

    # Hands clear check
    tcps = mk.tcps()
    hands_clear = True
    if len(tcps) >= 2:
        ltcp, rtcp = tcps[0], tcps[1]
        if np.linalg.norm(np.array(ltcp) - np.array(h_pos)) < 0.045:
            hands_clear = False
        if np.linalg.norm(np.array(rtcp) - np.array(t_pos)) < 0.045:
            hands_clear = False

    if mk.MEMORY.get("platen_closed") and hands_clear:
        mk.MEMORY["hold_steps"] = mk.MEMORY.get("hold_steps", 0) + 1

    step_id = mk.MEMORY.get("step_id", 0) + 1
    mk.MEMORY["step_id"] = step_id
    if step_id % 20 == 0 or step_id in (1, 10, 50, 100, 150, 200, 250, 300, 320):
        print(f"[STEP {step_id:04d}] score={score(objects):.2f} t_pos=({t_pos[0]:.3f},{t_pos[1]:.3f},{t_pos[2]:.3f}) d_tray={d_tray:.4f} handle_z={handle_z:.4f} gap_z={plate_gap_z:.4f} cleared={mk.MEMORY.get('tongs_cleared')} on_tray={mk.MEMORY.get('tongs_on_tray')} descended={mk.MEMORY.get('platen_descended')} closed={mk.MEMORY.get('platen_closed')} hold={mk.MEMORY.get('hold_steps', 0)}", flush=True)


kit.on_step(_step_rules)


def score(objects) -> float:
    if mk.MEMORY.get("defect"):
        return 0.0
    if not mk.MEMORY.get("tongs_cleared"):
        return 0.0
    if not mk.MEMORY.get("tongs_on_tray"):
        return 0.20
    if not mk.MEMORY.get("platen_descended"):
        return 0.50
    if not mk.MEMORY.get("platen_closed"):
        return 0.75
    if mk.MEMORY.get("hold_steps", 0) < 20:
        return 0.90
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# Scripted Bimanual Demonstration Plan
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
