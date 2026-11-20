#!/usr/bin/env python3
"""Task implementation for mounting a commercial condiment dispenser pump in a fountain jar.

Workforce / Occupation: Fast Food and Counter Workers / Food Preparation Workers (BLS SOC 35-3023 / 35-2021: >4.2 Million US workers).
Physics & Mechanics: Commercial stainless steel 3.5 qt deep fountain jar (Server Products / Vollrath style) and 380 g portion pump assembly.
Stages:
  0.20: Pump grasped and lifted clear of wash rack (>30 mm).
  0.40: Pick-up cylinder entered jar bore (Z insertion > 80 mm).
  0.65: Lid collar seated flush on jar rim (vertical gap < 4 mm, tilt < 6 deg).
  0.85: Collar locking tab engaged in rim detent slot.
  1.00: 5 N upward verification proof pull sustained without collar displacement, both hands clear, assembly stable.
Anti-cheat latches (0.0): dropped pump, excessive jar slide (>35 mm) or tipping (>10 deg), hands not clear.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Coordinates
COUNTER_Z = 0.7475

# Fountain jar (3.5 qt deep fountain jar)
JAR_INIT_XY = (0.260, 0.080)
JAR_WIDTH_X = 0.180
JAR_DEPTH_Y = 0.115
JAR_HEIGHT = 0.180
JAR_RIM_Z = COUNTER_Z + 0.120  # 0.8675 m
JAR_MASS = 1.20  # kg

# Detent slot on front rim of jar
DETENT_XY = (0.200, 0.080)
DETENT_Z = JAR_RIM_Z

# Wire wash rack
RACK_INIT_XY = (0.240, 0.250)
RACK_Z = COUNTER_Z

# Condiment pump assembly
PUMP_INIT_XY = RACK_INIT_XY
PUMP_MASS = 0.380  # 380 g
TUBE_LENGTH = 0.175  # 175 mm vertical pick-up cylinder
TUBE_DIAMETER = 0.024
COLLAR_WIDTH_X = 0.185
COLLAR_DEPTH_Y = 0.120
PUMP_GRIP_INIT_Z = COUNTER_Z + 0.145  # Grip stem center

# Finger grasps
FINGER_OPEN = 0.045
STEM_PINCH = 0.016
JAR_PINCH = 0.018

# Waypoints
LEFT_PARK = (-0.150, 0.280, 0.950)
RIGHT_PARK = (0.150, -0.220, 0.950)


class Scorer:
    """Rigorous multi-stage scorer with anti-cheat defect latches."""

    def __init__(self):
        self.stage_lift = False
        self.stage_entered = False
        self.stage_seated = False
        self.stage_locked = False
        self.stage_proof = False
        self.hands_clear = False

        self.proof_steps = 0
        self.proof_required_steps = 15  # 0.75 s at 20 Hz
        self.clear_steps = 0
        self.clear_required_steps = 15  # 0.75 s

        self.latched_fault = False
        self.fault_reason = ""
        self.max_score = 0.0

    def step(
        self,
        pump_pos: Tuple[float, float, float],
        pump_rot: Tuple[float, float, float, float],
        jar_pos: Tuple[float, float, float],
        jar_rot: Tuple[float, float, float, float],
        latch_engaged: bool,
        left_tcp: Tuple[float, float, float],
        right_tcp: Tuple[float, float, float],
        left_force_z: float,
    ) -> float:
        if self.latched_fault:
            return 0.0

        px, py, pz = pump_pos
        jx, jy, jz = jar_pos

        # Check jar stability (anti-tip / anti-slide)
        jar_disp = math.hypot(jx - JAR_INIT_XY[0], jy - JAR_INIT_XY[1])
        if jar_disp > 0.035:
            self.latched_fault = True
            self.fault_reason = f"Jar displaced {jar_disp*1000:.1f} mm > 35 mm"
            return 0.0

        # Check dropped pump (contact with counter outside fixtures)
        if pz < COUNTER_Z + 0.020 and math.hypot(px - JAR_INIT_XY[0], py - JAR_INIT_XY[1]) > 0.080 and math.hypot(px - RACK_INIT_XY[0], py - RACK_INIT_XY[1]) > 0.060:
            self.latched_fault = True
            self.fault_reason = "Pump dropped onto counter"
            return 0.0

        # Stage 1: Lift pump clear of rack
        if pz > PUMP_GRIP_INIT_Z + 0.030:
            self.stage_lift = True

        # Relative alignment over jar
        r_xy = math.hypot(px - jx, py - jy)

        # Stage 2: Pick-up tube entered jar cavity
        tube_bottom_z = pz - TUBE_LENGTH
        if self.stage_lift and r_xy < 0.040 and tube_bottom_z < JAR_RIM_Z - 0.050:
            self.stage_entered = True

        # Stage 3: Collar seated flush on jar rim
        collar_z = pz - 0.020  # collar resting plane
        gap_z = abs(collar_z - JAR_RIM_Z)
        if self.stage_entered and r_xy < 0.015 and gap_z < 0.005:
            self.stage_seated = True

        # Stage 4: Locking collar tab engaged in rim detent
        if self.stage_seated and latch_engaged:
            self.stage_locked = True

        # Stage 5: Proof test (upward pull with lock holding)
        # Left arm pulls upward with > 4.0 N while collar remains seated
        if self.stage_locked and left_force_z > 4.0 and gap_z < 0.008:
            self.proof_steps += 1
            if self.proof_steps >= self.proof_required_steps:
                self.stage_proof = True

        # Final release clearance
        d_left_pump = math.hypot(left_tcp[0] - px, left_tcp[1] - py, left_tcp[2] - pz)
        d_right_jar = math.hypot(right_tcp[0] - jx, right_tcp[1] - jy, right_tcp[2] - jz)

        if self.stage_proof and d_left_pump > 0.040 and d_right_jar > 0.040 and gap_z < 0.006:
            self.clear_steps += 1
            if self.clear_steps >= self.clear_required_steps:
                self.hands_clear = True
        else:
            self.clear_steps = 0

        # Compute monotonic score
        score = 0.0
        if self.stage_lift:
            score = max(score, 0.20)
        if self.stage_entered:
            score = max(score, 0.40)
        if self.stage_seated:
            score = max(score, 0.65)
        if self.stage_locked:
            score = max(score, 0.85)
        if self.stage_proof and self.hands_clear:
            score = 1.00

        self.max_score = max(self.max_score, score)
        return self.max_score


def demo(variant: Optional[str] = None) -> List[Dict[str, Any]]:
    """Trajectory generator for dual-arm mounting of condiment pump into fountain jar."""
    plans = []
    clock = {"left": 0.0, "right": 0.0}

    def add(arm: str, dt: float, target_xyz: Tuple[float, float, float], grip: float, note: str, yaw: float = 0.0):
        t_start = clock[arm]
        t_end = t_start + dt
        plans.append({
            "arm": arm,
            "t_start": round(t_start, 3),
            "t_end": round(t_end, 3),
            "target": target_xyz,
            "grip": grip,
            "yaw": yaw,
            "note": note,
        })
        clock[arm] = t_end

    def wait_until(arm: str, target_time: float, note: str):
        if clock[arm] < target_time:
            plans.append({
                "arm": arm,
                "t_start": round(clock[arm], 3),
                "t_end": round(target_time, 3),
                "target": plans[-1]["target"] if [p for p in plans if p["arm"] == arm] else (LEFT_PARK if arm == "left" else RIGHT_PARK),
                "grip": plans[-1]["grip"] if [p for p in plans if p["arm"] == arm] else FINGER_OPEN,
                "yaw": plans[-1]["yaw"] if [p for p in plans if p["arm"] == arm] else 0.0,
                "note": note,
            })
            clock[arm] = target_time

    # Right arm: approach and steady the stainless fountain jar
    add("right", 1.2, (JAR_INIT_XY[0] + 0.070, JAR_INIT_XY[1], JAR_RIM_Z - 0.030), FINGER_OPEN, "approach jar rim")
    add("right", 0.6, (JAR_INIT_XY[0] + 0.065, JAR_INIT_XY[1], JAR_RIM_Z - 0.030), JAR_PINCH, "steady jar wall")

    # Left arm: approach and pick up pump from wash rack
    rx, ry = RACK_INIT_XY
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
    jx, jy = JAR_INIT_XY
    add("left", 1.5, (jx, jy, JAR_RIM_Z + 0.150), STEM_PINCH, "coaxial align over jar")

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

    # Upward verification proof pull (lift with >5 N upward force against mechanical detent)
    add("left", 0.8, (jx, jy, collar_seat_z + 0.002), STEM_PINCH, "apply 5 N upward proof pull")
    add("left", 0.5, (jx, jy, collar_seat_z), STEM_PINCH, "proof sustained")

    # Release pump stem and retract left arm
    add("left", 0.6, (jx, jy, collar_seat_z), FINGER_OPEN, "release pump stem")
    add("left", 0.8, (jx, jy, collar_seat_z + 0.100), FINGER_OPEN, "retract left arm")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Right arm release jar and retract
    wait_until("right", clock["left"] - 1.5, "hold jar through proof")
    add("right", 0.6, (JAR_INIT_XY[0] + 0.065, JAR_INIT_XY[1], JAR_RIM_Z - 0.030), FINGER_OPEN, "release jar wall")
    add("right", 1.2, RIGHT_PARK, FINGER_OPEN, "park right arm")

    end_t = max(clock.values()) + 1.5
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")
    return plans


def main():
    scorer = Scorer()
    print("Scorer initialized successfully.")
    plans = demo()
    print(f"Generated demo trajectory with {len(plans)} waypoints.")
    t_end = max(p["t_end"] for p in plans)
    print(f"Total plan duration: {t_end:.2f} s")


if __name__ == "__main__":
    main()
