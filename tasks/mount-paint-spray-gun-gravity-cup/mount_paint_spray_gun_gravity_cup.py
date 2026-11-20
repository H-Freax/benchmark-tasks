"""Yam arena task: mount-paint-spray-gun-gravity-cup.

Occupational Context:
Industrial and automotive painters (SOC 51-9122 / 49-3021 / 47-2141; >780,000 US
workers) prepare HVLP gravity-feed spray guns at specialized bench workstations.
Before loading coating fluid, the technician mounts a rigid 650 mL gravity cup
onto the top fluid inlet boss of the spray gun, seats the tapered spout firmly
into the internal sealing seat, and engages the knurled locking cam collar ring
past its 45-degree detent until locked. A positive retention proof tug confirms
the bayonet cam is securely latched before both arms clear the workspace.

Physics & Scorer:
- Stand and spray gun body mounted on benchtop at (GUN_X, GUN_Y, TOP).
- Knurled locking collar ring rotates around vertical fluid inlet axis with 0.08 N*m detent friction.
- Gravity cup starts upright in staging cup rest on bench.
- Left arm lifts cup and seats spout coaxially into fluid inlet.
- Right arm counter-holds gun handle and rotates locking cam ring clockwise into locked detent.
- Scorer verifies 4 physical stages:
    0.25: Cup lifted and aligned above inlet within 25 mm XY and tilt <= 15 deg.
    0.50: Cup spout seated coaxially into inlet boss (radial error <= 3.5 mm, tilt <= 6 deg).
    0.75: Locking collar rotated clockwise >= 40 deg into positive locked detent.
    1.00: Active upward retention proof pull (20 N) held 1.0 s with slip <= 0.8 mm,
          both arms clear >= 25 mm, and held stationary for 1.0 s.
- Cheat variants: nothing (0.0), align_only (0.25), seat_only (0.50), no_release (0.75).
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
from dataclasses import dataclass

import numpy as np

# Enable local imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mechkit as mk
from yam_arena.task_file import GREY, TABLE_TOP, Box, ObjectState

# -- Environment & Scene Setup ------------------------------------------------
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
TAG = "mount_paint_spray_gun_gravity_cup"

INSTRUCTION = (
    "Hold the spray gun upright, seat the gravity cup firmly into the top fluid"
    " inlet, rotate the locking cam ring clockwise until it clicks into locked"
    " position, verify retention with an upward tug, and clear both arms."
)

# Colors
GUN_SILVER = (0.72, 0.74, 0.76)
GUN_BLUE = (0.12, 0.35, 0.65)
STAND_STEEL = (0.35, 0.37, 0.40)
KNURLED_RING = (0.28, 0.30, 0.32)
DARK_METAL = (0.20, 0.22, 0.25)
CUP_TRANSLUCENT = (0.92, 0.93, 0.95)
LID_BLACK = (0.15, 0.15, 0.16)
REST_BLUE = (0.20, 0.45, 0.70)

# -- Physical Coordinates & Dimensions (1:1 True Scale) -----------------------
# Spray gun mounted in low-profile bench clamp stand
GUN_X = 0.240
GUN_Y = 0.050
GUN_STAND_Z = TOP

# Fluid inlet boss on top of gun body
INLET_X = GUN_X
INLET_Y = GUN_Y
INLET_RIM_Z = TOP + 0.080  # 80 mm above table
INLET_SEAT_Z = INLET_RIM_Z

# Gravity paint cup (650 mL rigid nylon cup resting in cup holder tray)
CUP_START_X = 0.240
CUP_START_Y = 0.170
CUP_START_Z = TOP + 0.040  # Seated in cup rest tray

# When fully seated in inlet:
CUP_SEATED_Z = TOP + 0.090  # Flush seated in inlet socket

# Locking collar cam ring:
RING_Z = TOP + 0.075
LOCK_ANGLE_DEG = 45.0
LOCK_ANGLE_RAD = math.radians(LOCK_ANGLE_DEG)  # 0.7854 rad

# Physical thresholds
SEAT_MAX_XY_ERR = 0.0035  # 3.5 mm max radial offset from inlet axis
SEAT_MAX_TILT_DEG = 6.0  # 6.0 deg max angular tilt from vertical
PROOF_PULL_FORCE_N = 20.0  # 20 N upward retention proof pull
PROOF_MAX_SLIP_M = 0.0008  # 0.8 mm max vertical displacement during proof
PROOF_HOLD_STEPS = 20  # 1.0 s at 20 Hz
CLEAR_MARGIN_M = 0.025  # 25 mm clearance margin for hands

kit = mk.Kit()

# -- 1. Spray Gun & Stand Assembly (Base Rigid Body) --------------------------
stand_gun_boxes = [
    # Low-profile heavy bench clamp base plate (120 x 80 x 10 mm)
    mk.box((0.120, 0.080, 0.010), (0.0, 0.0, 0.005), color=STAND_STEEL),
    # Upright clamp post (18 x 18 x 45 mm)
    mk.box((0.018, 0.018, 0.045), (-0.025, 0.0, 0.028), color=STAND_STEEL),
    # Spray gun handle: ergonomic grip (26 x 32 x 55 mm)
    mk.box((0.026, 0.032, 0.055), (0.0, 0.0, 0.035), color=GUN_BLUE),
    # Gun receiver frame & valve housing (32 x 36 x 22 mm)
    mk.box((0.032, 0.036, 0.022), (0.0, 0.0, 0.068), color=GUN_SILVER),
    # Gun barrel extending forward (-X) (65 x 24 x 22 mm)
    mk.box((0.065, 0.024, 0.022), (-0.035, 0.0, 0.068), color=GUN_SILVER),
    # Forward air cap nozzle (20 x 22 x 22 mm)
    mk.box((0.020, 0.022, 0.022), (-0.075, 0.0, 0.068), color=DARK_METAL),
    # Rear fluid micrometer knob (18 x 18 x 18 mm)
    mk.box((0.018, 0.018, 0.018), (0.025, 0.0, 0.068), color=DARK_METAL),
    # Fluid inlet boss extending vertically to receive gravity cup spout
    mk.box((0.022, 0.022, 0.016), (0.0, 0.0, 0.080), color=GUN_SILVER),
]

spray_gun = kit.part(
    "spray_gun",
    (GUN_X, GUN_Y, GUN_STAND_Z),
    stand_gun_boxes,
    mass=0.750,  # 750g stable clamped assembly
    color=GUN_SILVER,
)

# -- 2. Knurled Locking Cam Collar Ring (Revolute Mechanism) -------------------
ring_boxes = [
    # Knurled outer collar ring (OD 32 mm, ID 22 mm, height 12 mm)
    mk.box((0.032, 0.005, 0.012), (0.0, 0.0135, 0.0), color=KNURLED_RING),
    mk.box((0.032, 0.005, 0.012), (0.0, -0.0135, 0.0), color=KNURLED_RING),
    mk.box((0.005, 0.022, 0.012), (0.0135, 0.0, 0.0), color=KNURLED_RING),
    mk.box((0.005, 0.022, 0.012), (-0.0135, 0.0, 0.0), color=KNURLED_RING),
    # Ergonomic thumb grip tab on collar (+X side, 8 x 12 x 8 mm)
    mk.box((0.008, 0.012, 0.008), (0.020, 0.0, 0.0), color=DARK_METAL),
]

locking_ring = kit.part(
    "locking_ring",
    (INLET_X, INLET_Y, RING_Z),
    ring_boxes,
    mass=0.040,  # 40g knurled collar
    color=KNURLED_RING,
)

kit.hinge(
    "ring_latch",
    parent="spray_gun",
    child="locking_ring",
    anchor=(INLET_X, INLET_Y, RING_Z),
    axis=(0.0, 0.0, 1.0),
    lower_deg=-55.0,  # Clockwise rotation in world Z
    upper_deg=5.0,
    start_deg=0.0,
    stiffness=0.0,
    friction=0.08,
)

# Staging cup rest tray on bench
cup_tray_boxes = [
    mk.box((0.080, 0.080, 0.020), (0.0, 0.0, 0.010), color=REST_BLUE),
    mk.box((0.090, 0.006, 0.035), (0.0, 0.042, 0.018), color=STAND_STEEL),
    mk.box((0.090, 0.006, 0.035), (0.0, -0.042, 0.018), color=STAND_STEEL),
    mk.box((0.006, 0.080, 0.035), (0.042, 0.0, 0.018), color=STAND_STEEL),
    mk.box((0.006, 0.080, 0.035), (-0.042, 0.0, 0.018), color=STAND_STEEL),
]

cup_tray = kit.part(
    "cup_tray",
    (CUP_START_X, CUP_START_Y, TOP),
    cup_tray_boxes,
    mass=None,  # Fixed staging tray
    color=REST_BLUE,
)

# -- 3. Gravity Paint Cup (Free Rigid Body) -----------------------------------
cup_boxes = [
    # Main container body (OD 76 mm, height 70 mm)
    mk.box((0.076, 0.076, 0.070), (0.0, 0.0, 0.030), color=CUP_TRANSLUCENT),
    # Top vented screw lid (OD 78 mm, height 10 mm)
    mk.box((0.078, 0.078, 0.010), (0.0, 0.0, 0.068), color=LID_BLACK),
    # Bottom tapered funnel neck (OD 32 mm, height 12 mm)
    mk.box((0.032, 0.032, 0.012), (0.0, 0.0, -0.010), color=CUP_TRANSLUCENT),
    # Bottom cylindrical spout nozzle (OD 15 mm, length 16 mm)
    mk.box((0.015, 0.015, 0.016), (0.0, 0.0, -0.022), color=DARK_METAL),
    # Dual bayonet locking lugs on spout (width 4 mm)
    mk.box((0.020, 0.004, 0.004), (0.0, 0.0, -0.018), color=DARK_METAL),
]

paint_cup = kit.part(
    "paint_cup",
    (CUP_START_X, CUP_START_Y, CUP_START_Z),
    cup_boxes,
    mass=0.110,  # 110g rigid cup
    color=CUP_TRANSLUCENT,
)

OBJECTS = [
    *kit.objects(),
]

# Shared memory state
mk.MEMORY.clear()
mk.MEMORY.update({
    "cup_aligned": False,
    "cup_seated": False,
    "ring_locked": False,
    "proof_state": "idle",
    "proof_timer": 0,
    "proof_start_z": 0.0,
    "proof_slip_m": 0.0,
    "step_count": 0,
})


def _update_mechanics(objects: dict):
    """Step callback executing physics contact and proof checks."""
    M = mk.MEMORY
    step = M.get("step_count", 0)
    M["step_count"] = step + 1

    cup_p, _ = kit.pose(objects, "paint_cup")
    gun_p, _ = kit.pose(objects, "spray_gun")

    inlet_cur_x = gun_p[0]
    inlet_cur_y = gun_p[1]

    # Radial XY offset from inlet axis
    xy_offset = math.hypot(cup_p[0] - inlet_cur_x, cup_p[1] - inlet_cur_y)
    # Vertical distance from flush seated height
    z_diff = cup_p[2] - (gun_p[2] + (CUP_SEATED_Z - GUN_STAND_Z))

    # Tilt angle from vertical
    cup_top_pt = kit.point(objects, "paint_cup", (0.0, 0.0, 1.0))
    cup_base_pt = kit.point(objects, "paint_cup", (0.0, 0.0, 0.0))
    up_vec = cup_top_pt - cup_base_pt
    up_norm = np.linalg.norm(up_vec)
    up_z = float(up_vec[2] / up_norm) if up_norm > 1e-6 else 1.0
    up_z = max(-1.0, min(1.0, up_z))
    tilt_deg = math.degrees(math.acos(up_z))

    # 1. Alignment check (hovering within 25 mm XY and within 45 mm above inlet)
    aligned = bool(
        xy_offset <= 0.025
        and 0.000 <= z_diff <= 0.045
        and tilt_deg <= 15.0
    )
    M["cup_aligned"] = aligned

    # 2. Seated check (coaxial within 3.5 mm XY, seated depth within 3.5 mm Z, tilt <= 6 deg)
    seated = bool(
        xy_offset <= SEAT_MAX_XY_ERR
        and abs(z_diff) <= 0.0035
        and tilt_deg <= SEAT_MAX_TILT_DEG
    )
    M["cup_seated"] = seated

    # 3. Locking collar rotation angle (read from hinge angle)
    try:
        ring_angle = kit.q_deg(objects, "ring_latch")
    except Exception:
        ring_angle = 0.0

    # Locking condition: cup is seated and ring is rotated clockwise >= 40 deg
    ring_locked = bool(seated and abs(ring_angle) >= 40.0)
    M["ring_locked"] = ring_locked

    # 4. Proof pull evaluation:
    hands_clear = bool(
        _released(objects["paint_cup"], CLEAR_MARGIN_M)
        and _released(objects["spray_gun"], CLEAR_MARGIN_M)
    )
    proof_state = M.get("proof_state", "idle")

    if ring_locked and hands_clear:
        if proof_state == "idle":
            M["proof_state"] = "pulling"
            M["proof_timer"] = 0
            M["proof_start_z"] = cup_p[2]
            M["proof_slip_m"] = 0.0
        elif proof_state == "pulling":
            kit.pull("paint_cup", (0.0, 0.0, PROOF_PULL_FORCE_N))
            M["proof_timer"] = M.get("proof_timer", 0) + 1
            slip = abs(cup_p[2] - M.get("proof_start_z", cup_p[2]))
            M["proof_slip_m"] = max(M.get("proof_slip_m", 0.0), slip)

            if slip > PROOF_MAX_SLIP_M:
                M["proof_state"] = "failed"
                kit.pull("paint_cup")
            elif M["proof_timer"] >= PROOF_HOLD_STEPS:
                M["proof_state"] = "held"
                kit.pull("paint_cup")
    elif not ring_locked and hands_clear and proof_state == "pulling":
        M["proof_state"] = "failed"
        kit.pull("paint_cup")

    if step % 20 == 0:
        cur_score = score(objects)
        print(
            f"[{TAG}] step={step:3d} cup_z={cup_p[2]:.4f} xy_off={xy_offset*1000:.1f}mm "
            f"z_diff={z_diff*1000:.1f}mm tilt={tilt_deg:.1f}deg ring={ring_angle:.1f}deg "
            f"seated={seated} proof={M.get('proof_state')} score={cur_score}",
            flush=True,
        )


kit.on_step(_update_mechanics)


# -- Scorer Helpers ------------------------------------------------------------


def _released(obj_state: ObjectState, margin: float = CLEAR_MARGIN_M) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    p = np.asarray(obj_state.pos, dtype=float)
    for tcp in tcps:
        if np.linalg.norm(np.asarray(tcp, dtype=float) - p) < margin:
            return False
    return True


def cup_aligned(objects: dict) -> bool:
    """Stage 1: True if paint cup is aligned above fluid inlet boss."""
    return bool(mk.MEMORY.get("cup_aligned", False))


def cup_seated(objects: dict) -> bool:
    """Stage 2: True if cup spout is seated coaxially in fluid inlet boss."""
    return bool(mk.MEMORY.get("cup_seated", False))


def ring_locked(objects: dict) -> bool:
    """Stage 3: True if cup is seated and locking collar ring is rotated to lock."""
    return bool(cup_seated(objects) and mk.MEMORY.get("ring_locked", False))


def proof_held(objects: dict) -> bool:
    """Stage 4: True if collar locked, 20 N proof held with slip <= 0.8 mm, and arms clear."""
    proof_pass = bool(mk.MEMORY.get("proof_state") == "held")
    hands_clear = bool(
        _released(objects["paint_cup"], CLEAR_MARGIN_M)
        and _released(objects["spray_gun"], CLEAR_MARGIN_M)
    )
    return bool(ring_locked(objects) and proof_pass and hands_clear)


def failed(objects: dict) -> bool:
    """Score 0 if cup dropped, overturned > 45 deg, or proof retention failed."""
    cup_p, _ = kit.pose(objects, "paint_cup")
    if cup_p[2] < TOP - 0.020:
        return True

    cup_top_pt = kit.point(objects, "paint_cup", (0.0, 0.0, 1.0))
    cup_base_pt = kit.point(objects, "paint_cup", (0.0, 0.0, 0.0))
    up_vec = cup_top_pt - cup_base_pt
    up_norm = np.linalg.norm(up_vec)
    up_z = float(up_vec[2] / up_norm) if up_norm > 1e-6 else 1.0
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    if tilt_deg > 45.0:
        return True

    if mk.MEMORY.get("proof_state") == "failed":
        return True

    return False


def score(objects: dict) -> float:
    """Staged scoring:

    0.25: Paint cup lifted and aligned within 25 mm XY of fluid inlet axis.
    0.50: Paint cup spout seated coaxially into fluid inlet boss.
    0.75: Locking cam collar ring rotated clockwise >= 40 deg into positive locked detent.
    1.00: Proof retention held 1.0 s under 20 N tug (slip <= 0.8 mm) and both arms cleared.
    Score 0.0: Cup dropped, overturned > 45 deg, or proof retention failed.
    """
    if failed(objects):
        return 0.0

    s = 0.0
    if cup_aligned(objects) or cup_seated(objects):
        s += 0.25
        if cup_seated(objects):
            s += 0.25
            if ring_locked(objects):
                s += 0.25
                if proof_held(objects):
                    s += 0.25
    return round(s, 4)


def success(objects: dict) -> bool:
    """True once full 1.0 score is achieved and held."""
    return score(objects) >= 1.0 - 1e-9


# -- Scripted Demo & Cheat Variants --------------------------------------------

try:
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
except ImportError:
    class Waypoint:
        def __init__(self, duration: float, pos: tuple[float, float, float] | None = None, finger: float | None = None, label: str = "", yaw: float = 0.0, *, xyz: tuple[float, float, float] | None = None, gripper: float | None = None):
            self.duration = float(duration)
            self.pos = pos if pos is not None else xyz
            self.finger = finger if finger is not None else gripper
            self.label = label
            self.yaw = float(yaw)
        @property
        def xyz(self) -> tuple[float, float, float]:
            return self.pos
        @xyz.setter
        def xyz(self, val: tuple[float, float, float]):
            self.pos = val
        @property
        def gripper(self) -> float:
            return self.finger
        @gripper.setter
        def gripper(self, val: float):
            self.finger = val

    FINGER_CLOSED = 0.0
    FINGER_OPEN = -0.04695


def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("SPRAY_CHEAT", "")

    if cheat == "nothing":
        left = [
            Waypoint(10.0, (0.20, 0.22, TOP + 0.150), FINGER_OPEN, "park_left")
        ]
        right = [
            Waypoint(10.0, (0.20, -0.22, TOP + 0.150), FINGER_OPEN, "park_right")
        ]
        return {"left": left, "right": right}

    CUP_GRIP_Z = 0.015
    SEAT_TCP_Z = CUP_SEATED_Z + CUP_GRIP_Z  # TOP + 0.105
    START_TCP_Z = CUP_START_Z + CUP_GRIP_Z  # TOP + 0.055

    # LEFT ARM: Manipulates paint cup
    left = [
        # Hover over cup in tray
        Waypoint(1.2, (CUP_START_X, CUP_START_Y, TOP + 0.140), FINGER_OPEN, "hover_cup"),
        # Descend to grip cup body
        Waypoint(1.0, (CUP_START_X, CUP_START_Y, START_TCP_Z), FINGER_OPEN, "descend_cup"),
        Waypoint(0.8, (CUP_START_X, CUP_START_Y, START_TCP_Z), FINGER_CLOSED, "grip_cup"),
        # Lift cup clear of tray
        Waypoint(1.2, (CUP_START_X, CUP_START_Y, TOP + 0.150), FINGER_CLOSED, "lift_cup"),
        # Transfer horizontally above fluid inlet boss
        Waypoint(1.5, (INLET_X, INLET_Y, TOP + 0.150), FINGER_CLOSED, "align_above_inlet"),
        # Lower cup into alignment zone (Stage 1: 0.25)
        Waypoint(1.0, (INLET_X, INLET_Y, SEAT_TCP_Z + 0.025), FINGER_CLOSED, "hover_inlet_aligned"),
    ]

    if cheat == "align_only":
        left.extend([
            Waypoint(2.0, (INLET_X, INLET_Y, SEAT_TCP_Z + 0.025), FINGER_CLOSED, "hold_aligned"),
            Waypoint(4.0, (0.20, 0.22, TOP + 0.150), FINGER_OPEN, "retreat_left"),
        ])
        right = [
            Waypoint(8.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "brace_gun_handle")
        ]
        return {"left": left, "right": right}

    # Seat cup axially into fluid inlet (Stage 2: 0.50)
    left.extend([
        Waypoint(1.2, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_CLOSED, "seat_spout_in_boss"),
        # Hold seated firmly while collar is rotated
        Waypoint(5.0, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_CLOSED, "hold_cup_seated"),
    ])

    if cheat == "seat_only":
        left.extend([
            Waypoint(0.8, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_OPEN, "release_cup_seat"),
            Waypoint(4.0, (0.20, 0.22, TOP + 0.150), FINGER_OPEN, "retreat_left"),
        ])
        right = [
            Waypoint(8.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "brace_gun_handle"),
            Waypoint(4.0, (0.20, -0.22, TOP + 0.150), FINGER_OPEN, "park_right"),
        ]
        return {"left": left, "right": right}

    # Stage 3: Collar ring engagement & Stage 4 proof
    left.extend([
        # Release cup grip and clear up
        Waypoint(0.8, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_OPEN, "release_cup_seat"),
        Waypoint(0.8, (INLET_X, INLET_Y, TOP + 0.160), FINGER_OPEN, "clear_above_cup"),
    ])

    if cheat == "no_release":
        left.extend([
            Waypoint(1.0, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_CLOSED, "hold_unreleased"),
            Waypoint(6.0, (INLET_X, INLET_Y, SEAT_TCP_Z), FINGER_CLOSED, "hold_unreleased_dwell"),
        ])
        right = [
            Waypoint(1.2, (GUN_X, GUN_Y - 0.035, TOP + 0.120), FINGER_OPEN, "hover_gun_handle"),
            Waypoint(1.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_OPEN, "approach_handle"),
            Waypoint(0.8, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "brace_gun_handle"),
            Waypoint(4.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "counter_hold_brace"),
            Waypoint(0.6, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_OPEN, "release_handle"),
            Waypoint(1.0, (INLET_X + 0.020, INLET_Y, RING_Z + 0.020), FINGER_OPEN, "approach_ring_tab"),
            Waypoint(0.6, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "pinch_ring_tab"),
            Waypoint(1.5, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "rotate_collar_lock", yaw=-0.785),
            Waypoint(6.0, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "hold_collar_unreleased"),
        ]
        return {"left": left, "right": right}

    # Full qualification demo:
    # Left arm: Performs manual proof tug on cup rim and retreats clear
    left.extend([
        Waypoint(1.0, (INLET_X, INLET_Y, TOP + 0.155), FINGER_OPEN, "approach_cup_rim"),
        Waypoint(0.6, (INLET_X, INLET_Y, TOP + 0.155), FINGER_CLOSED, "grip_cup_rim"),
        Waypoint(1.0, (INLET_X, INLET_Y, TOP + 0.168), FINGER_CLOSED, "manual_proof_tug"),
        Waypoint(0.6, (INLET_X, INLET_Y, TOP + 0.168), FINGER_OPEN, "release_proof_grip"),
        Waypoint(1.2, (0.20, 0.22, TOP + 0.150), FINGER_OPEN, "retreat_left_clear"),
        Waypoint(4.0, (0.20, 0.22, TOP + 0.150), FINGER_OPEN, "park_left_clear"),
    ])

    # RIGHT ARM: Counter-holds gun handle, rotates locking collar, retreats clear
    right = [
        # Hover near spray gun handle
        Waypoint(1.2, (GUN_X, GUN_Y - 0.035, TOP + 0.120), FINGER_OPEN, "hover_gun_handle"),
        Waypoint(1.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_OPEN, "approach_handle"),
        Waypoint(0.8, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "brace_gun_handle"),
        # Maintain rigid counter-hold during cup insertion
        Waypoint(4.0, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_CLOSED, "counter_hold_brace"),
        # Release handle to engage collar ring
        Waypoint(0.6, (GUN_X, GUN_Y - 0.035, TOP + 0.050), FINGER_OPEN, "release_handle"),
        # Approach collar ring tab
        Waypoint(1.0, (INLET_X + 0.020, INLET_Y, RING_Z + 0.020), FINGER_OPEN, "approach_ring_tab"),
        Waypoint(0.6, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "pinch_ring_tab"),
        # Rotate collar clockwise (yaw: 0.0 -> -0.785 rad) to engage cam detent
        Waypoint(1.5, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "rotate_collar_lock", yaw=-0.785),
        # Disengage from collar
        Waypoint(0.6, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_OPEN, "release_ring_tab", yaw=-0.785),
        Waypoint(0.8, (INLET_X, INLET_Y - 0.030, TOP + 0.140), FINGER_OPEN, "clear_above_collar"),
        # Retreat clear >= 25 mm
        Waypoint(1.2, (0.20, -0.22, TOP + 0.150), FINGER_OPEN, "retreat_right_clear"),
        Waypoint(7.0, (0.20, -0.22, TOP + 0.150), FINGER_OPEN, "park_right_clear"),
    ]

    return {"left": left, "right": right}
