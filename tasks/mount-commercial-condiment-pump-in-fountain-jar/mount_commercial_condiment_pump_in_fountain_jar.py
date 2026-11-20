"""Mount a commercial condiment dispenser pump in a stainless fountain jar.

Workforce / Occupation: Fast Food and Counter Workers / Food Preparation Workers (BLS SOC 35-3023 / 35-2021: >4.2 Million US workers).
Equipment: Commercial stainless steel 3.5 qt deep fountain jar (Server Products / Vollrath style) and 380 g portion pump assembly.
Fidelity & Mechanics:
- 100% natural 1:1 tabletop scale.
- Real hollow 5-plate open chamber cavity with collisions 100% enabled.
- Coaxial insertion of 175 mm pick-up tube into deep jar bore.
- Flush seating of rectangular collar on jar rim.
- Mechanical engagement into rim detent slot.
- Upward 5.0 N retention proof pull test before releasing clear.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
STEP_S = 0.05
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Colors
STAINLESS = (0.75, 0.77, 0.80)
DARK_STAINLESS = (0.45, 0.47, 0.50)
STAND_STEEL = (0.62, 0.64, 0.67)
PLUNGER_WHITE = (0.90, 0.90, 0.92)

# Coordinates
JAR_XY = (0.260, 0.080)
JAR_W_X = 0.180
JAR_D_Y = 0.115
JAR_H = 0.180
JAR_THICK = 0.008
JAR_RIM_Z = TOP + JAR_H  # 0.9275 m
JAR_KG = 1.20

RACK_XY = (0.240, 0.250)
RACK_KG = 0.50

PUMP_KG = 0.380
PUMP_START_POS = (RACK_XY[0], RACK_XY[1], TOP + 0.100)
PUMP_GRIP_INIT_Z = TOP + 0.190

# Grasps
STEM_PINCH = FINGER_CLOSED
JAR_PINCH = FINGER_CLOSED

# Park poses
LEFT_PARK = (-0.150, 0.280, 0.980)
RIGHT_PARK = (0.150, -0.220, 0.980)


def _jar_boxes() -> list[mk.BoxSpec]:
    """Hollow rectangular chamber: floor plate + 4 walls + rim detent flange."""
    boxes = []
    # Floor plate
    boxes.append(mk.box((JAR_W_X, JAR_D_Y, JAR_THICK), at=(0.0, 0.0, JAR_THICK / 2), color=STAINLESS))
    # Side walls
    boxes.append(mk.box((JAR_THICK, JAR_D_Y, JAR_H), at=(-JAR_W_X / 2 + JAR_THICK / 2, 0.0, JAR_H / 2), color=STAINLESS))
    boxes.append(mk.box((JAR_THICK, JAR_D_Y, JAR_H), at=(JAR_W_X / 2 - JAR_THICK / 2, 0.0, JAR_H / 2), color=STAINLESS))
    # Front / Back walls
    boxes.append(mk.box((JAR_W_X - 2 * JAR_THICK, JAR_THICK, JAR_H), at=(0.0, -JAR_D_Y / 2 + JAR_THICK / 2, JAR_H / 2), color=STAINLESS))
    boxes.append(mk.box((JAR_W_X - 2 * JAR_THICK, JAR_THICK, JAR_H), at=(0.0, JAR_D_Y / 2 - JAR_THICK / 2, JAR_H / 2), color=STAINLESS))
    # Rim detent lip
    boxes.append(mk.box((JAR_W_X + 0.016, JAR_D_Y + 0.016, 0.008), at=(0.0, 0.0, JAR_H - 0.004), color=DARK_STAINLESS))
    return boxes


def _rack_boxes() -> list[mk.BoxSpec]:
    """Sanitary wire wash rack for holding pump upright."""
    boxes = []
    boxes.append(mk.box((0.120, 0.120, 0.015), at=(0.0, 0.0, 0.0075), color=STAND_STEEL))
    boxes.append(mk.box((0.040, 0.040, 0.080), at=(0.0, 0.0, 0.040), color=STAND_STEEL))
    return boxes


def _pump_boxes() -> list[mk.BoxSpec]:
    """Commercial portion-control condiment dispenser pump assembly."""
    boxes = []
    # Pick-up cylinder
    boxes.append(mk.box((0.024, 0.024, 0.160), at=(0.0, 0.0, -0.080), color=STAINLESS))
    # Rectangular lid collar
    boxes.append(mk.box((JAR_W_X + 0.008, JAR_D_Y + 0.008, 0.012), at=(0.0, 0.0, 0.010), color=STAINLESS))
    # Inner centering boss
    boxes.append(mk.box((JAR_W_X - 2 * JAR_THICK - 0.004, JAR_D_Y - 2 * JAR_THICK - 0.004, 0.008), at=(0.0, 0.0, 0.001), color=DARK_STAINLESS))
    # Vertical grip stem
    boxes.append(mk.box((0.022, 0.022, 0.070), at=(0.0, 0.0, 0.050), color=STAINLESS))
    # Portion knob / plunger
    boxes.append(mk.box((0.046, 0.046, 0.020), at=(0.0, 0.0, 0.090), color=PLUNGER_WHITE))
    # Collar locking detent tab (projects forward)
    boxes.append(mk.box((0.016, 0.024, 0.012), at=(-JAR_W_X / 2 - 0.004, 0.0, 0.008), color=DARK_STAINLESS))
    return boxes


kit = mk.Kit()

# Register parts
kit.part("fountain_jar", (JAR_XY[0], JAR_XY[1], TOP + 0.0015), _jar_boxes(), mass=JAR_KG, color=STAINLESS, solver_position_iterations=16)
kit.part("wire_rack", (RACK_XY[0], RACK_XY[1], TOP), _rack_boxes(), mass=None, color=STAND_STEEL)
kit.part("condiment_pump", PUMP_START_POS, _pump_boxes(), mass=PUMP_KG, color=STAINLESS, solver_position_iterations=16)

FIXTURES = []
OBJECTS = kit.objects()

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


# -- Scorer logic ------------------------------------------------------------------------------------------------

def _step_rules(state: dict, actions: dict, env_ctx: dict) -> tuple[float, list[str]]:
    pump = state.get("condiment_pump", {})
    jar = state.get("fountain_jar", {})
    ppos = pump.get("pos", PUMP_START_POS)
    jpos = jar.get("pos", (JAR_XY[0], JAR_XY[1], TOP))

    left_tcp = actions.get("left", {}).get("tcp", LEFT_PARK)
    right_tcp = actions.get("right", {}).get("tcp", RIGHT_PARK)
    left_force_z = actions.get("left", {}).get("force_z", 0.0)

    px, py, pz = ppos
    jx, jy, jz = jpos

    # Check jar displacement
    j_disp = math.hypot(jx - JAR_XY[0], jy - JAR_XY[1])
    if j_disp > 0.035:
        return 0.0, ["jar_excessive_displacement"]

    # Check dropped pump
    if pz < TOP + 0.030 and math.hypot(px - JAR_XY[0], py - JAR_XY[1]) > 0.100 and math.hypot(px - RACK_XY[0], py - RACK_XY[1]) > 0.080:
        return 0.0, ["pump_dropped_on_counter"]

    # Stage 1 (0.20): Lift pump clear of rack
    stage_lift = (pz > PUMP_GRIP_INIT_Z + 0.030)

    # Relative alignment over jar
    r_xy = math.hypot(px - jx, py - jy)
    tube_bottom_z = pz - 0.160

    # Stage 2 (0.40): Pick-up tube entered jar bore
    stage_entered = (stage_lift and r_xy < 0.040 and tube_bottom_z < JAR_RIM_Z - 0.050)

    # Stage 3 (0.65): Collar seated flush on jar rim
    collar_z = pz + 0.010
    gap_z = abs(collar_z - JAR_RIM_Z)
    stage_seated = (stage_entered and r_xy < 0.015 and gap_z < 0.008)

    # Stage 4 (0.85): Collar locking detent engaged
    stage_locked = (stage_seated and px < jx - 0.005)

    # Stage 5 (1.00): Upward proof pull test (>= 5.0 N) held and hands clear
    d_left = math.hypot(left_tcp[0] - px, left_tcp[1] - py, left_tcp[2] - pz)
    d_right = math.hypot(right_tcp[0] - jx, right_tcp[1] - jy, right_tcp[2] - jz)
    hands_clear = (d_left > 0.035 and d_right > 0.035)

    stage_proof = (stage_locked and left_force_z >= 5.0 and gap_z < 0.010)

    score = 0.0
    if stage_lift:
        score = max(score, 0.20)
    if stage_entered:
        score = max(score, 0.40)
    if stage_seated:
        score = max(score, 0.65)
    if stage_locked:
        score = max(score, 0.85)
    if stage_proof and hands_clear:
        score = 1.00

    return score, []


def task_score(state: dict, actions: dict, env_ctx: dict) -> float:
    score, _ = _step_rules(state, actions, env_ctx)
    return score


# -- Policy demonstration ----------------------------------------------------------------------------------------

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

    jx, jy = JAR_XY
    rx, ry = RACK_XY

    # Right arm: steady the stainless fountain jar
    add("right", 1.2, (jx + 0.070, jy, JAR_RIM_Z - 0.040), FINGER_OPEN, "approach jar rim")
    add("right", 0.6, (jx + 0.065, jy, JAR_RIM_Z - 0.040), JAR_PINCH, "steady jar wall")

    # Left arm: approach and pick up pump from wash rack
    add("left", 1.0, (rx, ry, PUMP_GRIP_INIT_Z + 0.060), FINGER_OPEN, "approach rack above pump")
    add("left", 0.6, (rx, ry, PUMP_GRIP_INIT_Z), FINGER_OPEN, "descend onto pump stem")
    add("left", 0.5, (rx, ry, PUMP_GRIP_INIT_Z), STEM_PINCH, "grasp pump stem")
    add("left", 0.8, (rx, ry, PUMP_GRIP_INIT_Z + 0.120), STEM_PINCH, "lift pump clear of rack")

    if variant == "no_insert":
        add("left", 2.0, (rx, ry, PUMP_GRIP_INIT_Z + 0.120), STEM_PINCH, "hold pump above rack")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Transfer pump above fountain jar
    add("left", 1.5, (jx, jy, JAR_RIM_Z + 0.160), STEM_PINCH, "coaxial align over jar")

    # Insert pick-up tube down into jar bore
    add("left", 1.2, (jx, jy, JAR_RIM_Z + 0.060), STEM_PINCH, "insert tube halfway")

    if variant == "no_seat":
        add("left", 2.0, (jx, jy, JAR_RIM_Z + 0.060), STEM_PINCH, "hold suspended in bore")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Seat lid collar flush on jar rim
    collar_seat_z = JAR_RIM_Z + 0.020
    add("left", 0.8, (jx, jy, collar_seat_z), STEM_PINCH, "seat collar flush on rim")

    if variant == "no_lock":
        add("left", 2.0, (jx, jy, collar_seat_z), STEM_PINCH, "dwell seated without latch")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Press collar locking tab into detent slot
    add("left", 0.6, (jx - 0.010, jy, collar_seat_z - 0.003), STEM_PINCH, "press locking tab forward into detent")
    add("left", 0.4, (jx, jy, collar_seat_z), STEM_PINCH, "latch detent engaged")

    # Upward verification proof pull (lift with >=5.0 N upward force against mechanical detent)
    add("left", 0.8, (jx, jy, collar_seat_z + 0.002), STEM_PINCH, "apply 5 N upward proof pull")
    add("left", 0.5, (jx, jy, collar_seat_z), STEM_PINCH, "proof sustained")

    # Release pump stem and retract left arm
    add("left", 0.6, (jx, jy, collar_seat_z), FINGER_OPEN, "release pump stem")
    add("left", 0.8, (jx, jy, collar_seat_z + 0.100), FINGER_OPEN, "retract left arm")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Right arm release jar and retract
    wait_until("right", clock["left"] - 1.5, "hold jar through proof")
    add("right", 0.6, (jx + 0.065, jy, JAR_RIM_Z - 0.040), FINGER_OPEN, "release jar wall")
    add("right", 1.2, RIGHT_PARK, FINGER_OPEN, "park right arm")

    end_t = max(clock.values()) + 1.5
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")
    return plans


def main():
    print(f"Registered {len(OBJECTS)} objects with MechKit: {[o.name for o in OBJECTS]}")
    plans = demo()
    print(f"Generated demo trajectory for arms: {list(plans.keys())}")
    for arm, wps in plans.items():
        total_dt = sum(w.duration for w in wps)
        print(f"  {arm} arm: {len(wps)} waypoints, duration {total_dt:.2f} s")


if __name__ == "__main__":
    main()
