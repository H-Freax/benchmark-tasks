"""Dock a cordless handheld barcode scanner into its desktop presentation cradle.

Point of sale, retail, pharmacy, and healthcare operations under BLS SOC 41-2011,
41-2031, and 53-7065 (>7,500,000 workers) rely constantly on docking and undocking
cordless 2D handheld imagers (Zebra DS2278) into weighted presentation cradles
(Zebra CR2278) for continuous charging, hands-free scanning, and presentation-mode reading.

To prevent pushing or sliding the cradle across the counter, the left arm
steadies the weighted cradle base flange, while the right arm grasps the
contoured scanner handle, lifts it clear of the counter, aligns the imager nose
over the molded throat guide channel, and lowers it vertically into the cradle socket
past the spring-loaded detent retention latch until fully seated flush against the charging pins.
Both arms must then clear and release, and the scanner must remain stably held in equilibrium
without dislodging or tipping for at least 1.0 s (20 control steps).
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict

import numpy as np

# Task directory in sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mechkit as mk
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import ObjectState

SCENE = "station"
TOP = station.TABLE_TOP
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20
SCORE_OVER = "best"

# Colors (sRGB)
DARK_GREY = (0.15, 0.15, 0.16)
SCANNER_BLACK = (0.08, 0.08, 0.09)
CRADLE_GREY = (0.22, 0.22, 0.24)
WHITE = (0.9, 0.9, 0.9)
BRASS = (0.8, 0.65, 0.25)
BLUE_LED = (0.1, 0.4, 0.9)

# Spatial setup (relative to robot table midpoint at z = 0.0 on bench)
CRADLE_X = 0.36
CRADLE_Y = -0.05
CRADLE_Z = TOP + 0.0425  # half-height of 85 mm base

# Scanner start pose on counter: resting upright at x = 0.30, y = -0.10, z = 0.085
SCANNER_START_X = 0.30
SCANNER_START_Y = -0.10
SCANNER_START_Z = TOP + 0.085

# Seated target position of scanner in cradle (socket bottom stop)
DOCKED_POS = np.array([CRADLE_X, CRADLE_Y, TOP + 0.080])

# Thresholds
POS_TOL_M = 0.025       # 25 mm spatial tolerance to seated stop
TILT_TOL_DEG = 15.0     # 15 degrees tilt tolerance (must sit upright in socket)
STEADY_DIST_M = 0.15    # Left hand within 150 mm of cradle base
CLEAR_DIST_M = 0.040    # Both grippers clear > 40 mm
HOLD_STEPS_REQUIRED = 20

kit = mk.Kit()

# 1. Desktop Presentation Cradle (weighted cast base with molded cup)
# Zebra CR2278-PC: 150 mm long (x), 100 mm wide (y), 85 mm tall (z), 380 g
cradle_boxes = [
    # Weighted cast-iron base plate with rubber feet (150 x 100 x 25 mm)
    mk.box((0.150, 0.100, 0.025), (0.0, 0.0, -0.030), color=CRADLE_GREY),
    # Left and right precision guide cheeks forming hollow insertion throat
    mk.box((0.080, 0.015, 0.060), (0.0, 0.035, 0.0125), color=CRADLE_GREY),
    mk.box((0.080, 0.015, 0.060), (0.0, -0.035, 0.0125), color=CRADLE_GREY),
    # Rear contoured support cup wall
    mk.box((0.015, 0.055, 0.060), (-0.0325, 0.0, 0.0125), color=DARK_GREY),
    # Front retaining entry collar lip
    mk.box((0.015, 0.055, 0.030), (0.0325, 0.0, -0.0025), color=DARK_GREY),
    # Bottom socket charging contact rest stop
    mk.box((0.050, 0.055, 0.010), (0.0, 0.0, -0.0125), color=SCANNER_BLACK),
    # Front status LED indicator lens
    mk.box((0.010, 0.025, 0.008), (0.055, 0.0, -0.020), color=BLUE_LED),
]

# Cradle is an ordinary free rigid body with realistic 0.380 kg mass
kit.part("cradle", (CRADLE_X, CRADLE_Y, CRADLE_Z), cradle_boxes, mass=0.380, color=CRADLE_GREY)

# 2. Cordless Handheld Barcode Scanner
# Zebra DS2278: 175 mm tall, 75 mm head depth, 55 mm head width, 214 g
scanner_boxes = [
    # Contoured ergonomic handle
    mk.box((0.036, 0.032, 0.110), (0.0, 0.0, -0.015), color=SCANNER_BLACK),
    # Scan head housing
    mk.box((0.075, 0.055, 0.045), (0.0, 0.0, 0.0625), color=DARK_GREY),
    # Front exit window bezel
    mk.box((0.008, 0.048, 0.038), (0.035, 0.0, 0.0625), color=SCANNER_BLACK),
    # Base latch notch / charging contact pads
    mk.box((0.028, 0.026, 0.015), (0.0, 0.0, -0.0775), color=BRASS),
    # Trigger guard
    mk.box((0.022, 0.008, 0.025), (0.018, 0.0, 0.005), color=DARK_GREY),
]

# Scanner is a free rigid body (214 g)
kit.part("scanner", (SCANNER_START_X, SCANNER_START_Y, SCANNER_START_Z), scanner_boxes, mass=0.214, color=SCANNER_BLACK)

OBJECTS = kit.objects()


def _get_q(obj):
    return getattr(obj, "quat_xyzw", getattr(obj, "quat", np.array([0.0, 0.0, 0.0, 1.0])))


def _check_retention_hold(objects: Dict[str, ObjectState]) -> None:
    cradle = objects["cradle"]
    scanner = objects["scanner"]

    cq = _get_q(cradle)
    sq = _get_q(scanner)

    cradle_disp = np.linalg.norm(cradle.pos[:2] - np.array([CRADLE_X, CRADLE_Y]))
    cradle_up = 1.0 - 2.0 * (cq[0]**2 + cq[1]**2)
    cradle_stable = (cradle_disp < 0.040) and (cradle_up > 0.90)

    dock_dist = np.linalg.norm(scanner.pos - DOCKED_POS)
    scanner_up = 1.0 - 2.0 * (sq[0]**2 + sq[1]**2)
    tilt_deg = math.degrees(math.acos(np.clip(scanner_up, -1.0, 1.0)))

    tcps = mk.tcps()
    if tcps and len(tcps) == 2:
        dist_l_scanner = np.linalg.norm(tcps[0] - scanner.pos)
        dist_r_scanner = np.linalg.norm(tcps[1] - scanner.pos)
        dist_l_cradle = np.linalg.norm(tcps[0] - cradle.pos)
        hands_clear = (dist_l_scanner > CLEAR_DIST_M) and (dist_r_scanner > CLEAR_DIST_M) and (dist_l_cradle > CLEAR_DIST_M)
    else:
        hands_clear = True

    seated = (dock_dist < POS_TOL_M) and (tilt_deg < TILT_TOL_DEG)
    no_tip = not mk.MEMORY.get("cradle_tipped", False) and cradle_stable

    if seated and hands_clear and no_tip:
        held = mk.MEMORY.get("hold_steps", 0) + 1
        mk.MEMORY["hold_steps"] = held
        if held >= HOLD_STEPS_REQUIRED:
            mk.MEMORY["retained_equilibrium"] = True
    else:
        mk.MEMORY["hold_steps"] = 0


kit.on_step(_check_retention_hold)


def score(objects: Dict[str, ObjectState]) -> float:
    """Multi-stage scoring predicate:
      0.25: Left arm steadies cradle base AND right arm lifts scanner.
      0.50: Scanner aligned and guided above cradle throat (< 60 mm from target).
      0.75: Scanner seated into cradle socket against bottom stop (< 25 mm, < 15 deg tilt).
      1.00: Both grippers clear (> 40 mm) and scanner maintained seated in equilibrium >= 1.0 s.
    """
    if mk.MEMORY.get("cradle_tipped", False):
        return 0.0

    cradle = objects["cradle"]
    scanner = objects["scanner"]

    cq = _get_q(cradle)
    sq = _get_q(scanner)

    cradle_disp = np.linalg.norm(cradle.pos[:2] - np.array([CRADLE_X, CRADLE_Y]))
    cradle_up = 1.0 - 2.0 * (cq[0]**2 + cq[1]**2)
    cradle_stable = (cradle_disp < 0.040) and (cradle_up > 0.90)

    if not cradle_stable:
        mk.MEMORY["cradle_tipped"] = True
        return 0.0

    dock_dist = np.linalg.norm(scanner.pos - DOCKED_POS)
    scanner_up = 1.0 - 2.0 * (sq[0]**2 + sq[1]**2)
    tilt_deg = math.degrees(math.acos(np.clip(scanner_up, -1.0, 1.0)))

    tcps = mk.tcps()
    if tcps and len(tcps) == 2:
        dist_l_cradle = np.linalg.norm(tcps[0] - cradle.pos)
        steadiers = (dist_l_cradle < STEADY_DIST_M) and (scanner.pos[2] > SCANNER_START_Z + 0.015)
    else:
        steadiers = scanner.pos[2] > SCANNER_START_Z + 0.015

    mk.MEMORY.setdefault("steady_seen", False)
    mk.MEMORY.setdefault("inserted_seen", False)
    mk.MEMORY.setdefault("latched_seen", False)

    mk.MEMORY["steady_seen"] |= steadiers

    guided = mk.MEMORY["steady_seen"] and (dock_dist < 0.060)
    mk.MEMORY["inserted_seen"] |= guided

    seated = (dock_dist < POS_TOL_M) and (tilt_deg < TILT_TOL_DEG)
    mk.MEMORY["latched_seen"] |= seated

    if mk.MEMORY.get("retained_equilibrium", False):
        return 1.00
    if seated:
        return 0.75
    if mk.MEMORY["inserted_seen"]:
        return 0.50
    if mk.MEMORY["steady_seen"]:
        return 0.25
    return 0.00


def success(objects: Dict[str, ObjectState]) -> bool:
    """Return True if docked scanner is held in equilibrium with hands clear."""
    return score(objects) >= 1.00


def demo():
    """Bimanual synchronized demonstration for yam_scripted policy."""
    variant = os.environ.get("DEMO_VARIANT", "normal")

    if variant == "unsteadied":
        # Left arm does not steady cradle; cradle slips/tips
        left_wps = [
            Waypoint(8.0, (0.28, 0.15, TOP + 0.16), FINGER_OPEN, "stay clear"),
        ]
    else:
        left_wps = [
            Waypoint(1.5, (0.36, 0.05, TOP + 0.12), FINGER_OPEN, "above cradle"),
            Waypoint(1.0, (0.36, 0.05, TOP + 0.06), FINGER_CLOSED, "press cradle"),
            Waypoint(7.0, (0.36, 0.05, TOP + 0.06), FINGER_CLOSED, "steady cradle base"),
            Waypoint(1.5, (0.28, 0.15, TOP + 0.16), FINGER_OPEN, "clear left"),
            Waypoint(2.0, (0.28, 0.15, TOP + 0.16), FINGER_OPEN, "maintain clear left"),
        ]

    if variant == "unreleased":
        # Right arm never releases scanner handle
        right_wps = [
            Waypoint(1.5, (0.30, -0.10, TOP + 0.15), FINGER_OPEN, "above scanner"),
            Waypoint(1.0, (0.30, -0.10, TOP + 0.085), FINGER_OPEN, "reach scanner handle"),
            Waypoint(0.8, (0.30, -0.10, TOP + 0.085), FINGER_CLOSED, "grasp scanner handle"),
            Waypoint(1.5, (0.30, -0.10, TOP + 0.15), FINGER_CLOSED, "lift scanner"),
            Waypoint(1.5, (0.36, -0.05, TOP + 0.15), FINGER_CLOSED, "align over cradle throat"),
            Waypoint(1.5, (0.36, -0.05, TOP + 0.080), FINGER_CLOSED, "lower into cradle"),
            Waypoint(3.0, (0.36, -0.05, TOP + 0.080), FINGER_CLOSED, "hold gripped"),
        ]
    else:
        right_wps = [
            Waypoint(1.5, (0.30, -0.10, TOP + 0.15), FINGER_OPEN, "above scanner"),
            Waypoint(1.0, (0.30, -0.10, TOP + 0.085), FINGER_OPEN, "reach scanner handle"),
            Waypoint(0.8, (0.30, -0.10, TOP + 0.085), FINGER_CLOSED, "grasp scanner handle"),
            Waypoint(1.5, (0.30, -0.10, TOP + 0.15), FINGER_CLOSED, "lift scanner"),
            Waypoint(1.5, (0.36, -0.05, TOP + 0.15), FINGER_CLOSED, "align over cradle throat"),
            Waypoint(1.5, (0.36, -0.05, TOP + 0.080), FINGER_CLOSED, "lower into cradle"),
            Waypoint(0.8, (0.36, -0.05, TOP + 0.080), FINGER_OPEN, "release handle"),
            Waypoint(1.5, (0.28, -0.18, TOP + 0.16), FINGER_OPEN, "clear right"),
            Waypoint(2.0, (0.28, -0.18, TOP + 0.16), FINGER_OPEN, "maintain clear right"),
        ]

    return {"left": left_wps, "right": right_wps}
