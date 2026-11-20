"""Mount a paint spray gun gravity cup onto the fluid inlet and engage the locking cam ring.

Task specification:
- Embodiment: yam_arena bimanual tabletop workcell
- Domain: Automotive refinishing & industrial coatings (BLS SOC 51-9122 / 49-3021 / 47-2141; >780,000 workers)
- Hardware: Standard 1:1 professional HVLP spray gun (SATAjet / 3M PPS 2.0 style) on bench stand,
  650 mL rigid nylon gravity cup with dual bayonet lugs, and quarter-turn knurled locking cam collar.
- Mechanics:
  1. Left arm counter-holds spray gun handle upright on bench rest.
  2. Right arm grasps 650 mL gravity cup, transfers coaxially above top fluid inlet boss.
  3. Right arm inserts cup spout 15 mm axially into inlet chamber against Buna-N seal.
  4. Right arm rotates outer knurled locking cam ring 45 deg clockwise past detent click into locked seat.
  5. Right arm applies active 20 N upward retention proof tug for 1.0 s; locked cup withstands pull (slip <= 0.5 mm).
  6. Both robot arms release and retract clear (>= 25 mm).
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import mechkit as mk
from yam_arena.task_file import GREY, TABLE_TOP, Box, Cylinder, ObjectState

TAG = "mount_paint_spray_gun_gravity_cup"
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Hold the spray gun upright, seat the gravity cup firmly into the top fluid"
    " inlet, rotate the locking cam ring clockwise until it clicks into locked"
    " position, verify retention with an upward tug, and clear both arms."
)

# Colors
GUN_SILVER = (0.78, 0.80, 0.82)
GUN_BLUE = (0.15, 0.35, 0.65)
DARK_METAL = (0.25, 0.26, 0.28)
KNURLED_RING = (0.85, 0.55, 0.10)  # Anodized orange-gold collar ring
CUP_TRANSLUCENT = (0.92, 0.93, 0.95)
LID_BLACK = (0.12, 0.12, 0.14)
STAND_STEEL = (0.45, 0.46, 0.48)

# -- Physical Coordinates & Dimensions (1:1 True Scale) -----------------------
# Spray gun mounted in bench stand
GUN_X = 0.240
GUN_Y = -0.060
GUN_STAND_Z = TOP

# Fluid inlet boss on top of gun body
# Gun body top surface is at TOP + 0.120 m
# Inlet boss extends from z = TOP + 0.120 to TOP + 0.145 (height 25 mm, outer diam 22 mm, bore 16 mm)
INLET_X = GUN_X
INLET_Y = GUN_Y
INLET_SEAT_Z = TOP + 0.145  # Top rim of fluid inlet boss
INLET_BORE_BOTTOM_Z = TOP + 0.130  # Internal sealing face (15 mm insertion depth)

# Gravity paint cup (650 mL rigid nylon cup)
# Start position on table
CUP_START_X = 0.240
CUP_START_Y = 0.160
CUP_BODY_DIAM = 0.090  # 90 mm diameter
CUP_BODY_HEIGHT = 0.125  # 125 mm body height
CUP_SPOUT_DIAM = 0.015  # 15 mm outer diameter
CUP_SPOUT_LEN = 0.018  # 18 mm spout length
CUP_TOTAL_HEIGHT = CUP_BODY_HEIGHT + CUP_SPOUT_LEN  # 143 mm total height
CUP_START_Z = TOP + CUP_TOTAL_HEIGHT / 2.0

# When fully seated in inlet:
# Spout tip is at INLET_BORE_BOTTOM_Z (TOP + 0.130)
# Cup bottom flange is at INLET_SEAT_Z (TOP + 0.145)
# Cup center Z when seated:
CUP_SEATED_Z = INLET_SEAT_Z + CUP_BODY_HEIGHT / 2.0  # TOP + 0.2075 m

# Locking collar cam ring:
# Concentric with inlet boss, height 16 mm, outer diameter 32 mm
RING_Z = TOP + 0.138
LOCK_ANGLE_DEG = 45.0  # 45 degrees clockwise rotation to full lock
LOCK_ANGLE_RAD = math.radians(LOCK_ANGLE_DEG)  # 0.7854 rad

# Physical thresholds
SEAT_MAX_XY_ERR = 0.0035  # 3.5 mm max radial offset from inlet axis
SEAT_MIN_DEPTH = 0.0120  # 12.0 mm minimum axial insertion depth
SEAT_MAX_TILT_DEG = 6.0  # 6.0 deg max angular tilt from vertical
PROOF_PULL_FORCE_N = 20.0  # 20 N upward retention proof pull
PROOF_MAX_SLIP_M = 0.0008  # 0.8 mm max vertical displacement during proof
PROOF_HOLD_STEPS = 20  # 1.0 s at 20 Hz
CLEAR_MARGIN_M = 0.025  # 25 mm clearance margin for hands

kit = mk.Kit()

# -- 1. Spray Gun & Stand Assembly (Base Rigid Body) --------------------------
# Combined heavy benchtop stand and professional HVLP spray gun body
stand_gun_boxes = [
    # Heavy rectangular bench stand base plate (140 x 100 x 12 mm)
    mk.box((0.140, 0.100, 0.012), (0.0, 0.0, 0.006), color=STAND_STEEL),
    # Upright support post (20 x 20 x 90 mm)
    mk.box((0.020, 0.020, 0.090), (-0.030, 0.0, 0.057), color=STAND_STEEL),
    # Spray gun handle: ergonomic grip angled slightly (28 x 34 x 110 mm)
    mk.box((0.028, 0.034, 0.110), (0.0, 0.0, 0.065), color=GUN_BLUE),
    # Gun receiver frame & valve housing (35 x 40 x 30 mm)
    mk.box((0.035, 0.040, 0.030), (0.0, 0.0, 0.125), color=GUN_SILVER),
    # Gun barrel extending forward (-X) (85 x 28 x 26 mm)
    mk.box((0.085, 0.028, 0.026), (-0.042, 0.0, 0.125), color=GUN_SILVER),
    # Forward air cap & atomizing nozzle (24 x 26 x 26 mm)
    mk.box((0.024, 0.026, 0.026), (-0.095, 0.0, 0.125), color=DARK_METAL),
    # Rear fluid micrometer adjustment knob (20 x 22 x 22 mm)
    mk.box((0.020, 0.022, 0.022), (0.028, 0.0, 0.125), color=DARK_METAL),
    # Trigger paddle in front of grip (12 x 18 x 55 mm)
    mk.box((0.012, 0.018, 0.055), (-0.025, 0.0, 0.085), color=DARK_METAL),
    # Fluid inlet boss on top of gun (outer collar walls, OD 22 mm, ID 16 mm)
    # Modeled as 4 collar wall boxes enclosing the cylindrical socket
    mk.box((0.022, 0.003, 0.025), (0.0, 0.0095, 0.1375), color=GUN_SILVER),
    mk.box((0.022, 0.003, 0.025), (0.0, -0.0095, 0.1375), color=GUN_SILVER),
    mk.box((0.003, 0.016, 0.025), (0.0095, 0.0, 0.1375), color=GUN_SILVER),
    mk.box((0.003, 0.016, 0.025), (-0.0095, 0.0, 0.1375), color=GUN_SILVER),
]

spray_gun = kit.part(
    "spray_gun",
    (GUN_X, GUN_Y, GUN_STAND_Z),
    stand_gun_boxes,
    mass=0.650,  # 650g combined stand and gun
    color=GUN_SILVER,
)

# -- 2. Knurled Locking Cam Collar Ring (Revolute Mechanism) -------------------
# Concentric with fluid inlet boss, rotates around vertical Z axis
ring_boxes = [
    # Knurled outer collar ring (OD 34 mm, ID 24 mm, height 14 mm)
    mk.box((0.034, 0.005, 0.014), (0.0, 0.0145, 0.0), color=KNURLED_RING),
    mk.box((0.034, 0.005, 0.014), (0.0, -0.0145, 0.0), color=KNURLED_RING),
    mk.box((0.005, 0.024, 0.014), (0.0145, 0.0, 0.0), color=KNURLED_RING),
    mk.box((0.005, 0.024, 0.014), (-0.0145, 0.0, 0.0), color=KNURLED_RING),
    # Ergonomic thumb grip tab on collar (+X side, 8 x 14 x 10 mm)
    mk.box((0.008, 0.014, 0.010), (0.020, 0.0, 0.0), color=DARK_METAL),
]

locking_ring = kit.part(
    "locking_ring",
    (INLET_X, INLET_Y, RING_Z),
    ring_boxes,
    mass=0.045,  # 45g knurled aluminum ring
    color=KNURLED_RING,
)

# Hinge joint: locking ring revolves about Z-axis relative to spray gun body
# Lower limit: 0.0 deg (UNLOCKED); Upper limit: 55.0 deg (LOCKED, past 45 deg detent)
# Friction: 0.08 N*m to hold position stably against slight vibrations
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

# Bayonet cam lock weld: disarmed when authored, engaged when ring is locked past 40 deg
cup_seated_T = np.eye(4)
cup_seated_T[:3, 3] = [INLET_X, INLET_Y, CUP_SEATED_Z]
gun_stand_T = np.eye(4)
gun_stand_T[:3, 3] = [GUN_X, GUN_Y, GUN_STAND_Z]
kit.catch("cup_bayonet_lock", "paint_cup", cup_seated_T, parent="spray_gun", parent_T=gun_stand_T)

# -- 3. Gravity Paint Cup (Free Rigid Body) -----------------------------------
# 650 mL cup with bottom cylindrical spout nozzle and dual bayonet lugs
cup_boxes = [
    # Main cylindrical container body (OD 90 mm, height 120 mm)
    mk.box((0.090, 0.090, 0.120), (0.0, 0.0, 0.010), color=CUP_TRANSLUCENT),
    # Top vented screw lid (OD 92 mm, height 12 mm)
    mk.box((0.092, 0.092, 0.012), (0.0, 0.0, 0.070), color=LID_BLACK),
    # Bottom tapered neck funnel (OD 40 mm, height 14 mm)
    mk.box((0.040, 0.040, 0.014), (0.0, 0.0, -0.054), color=CUP_TRANSLUCENT),
    # Bottom cylindrical spout nozzle (OD 15 mm, length 18 mm)
    mk.box((0.015, 0.015, 0.018), (0.0, 0.0, -0.068), color=DARK_METAL),
    # Dual bayonet locking lugs on spout (width 4 mm, protrusion 3 mm each side)
    mk.box((0.021, 0.004, 0.004), (0.0, 0.0, -0.064), color=DARK_METAL),
]

paint_cup = kit.part(
    "paint_cup",
    (CUP_START_X, CUP_START_Y, CUP_START_Z),
    cup_boxes,
    mass=0.120,  # 120g rigid cup
    color=CUP_TRANSLUCENT,
)

OBJECTS = [
    *kit.objects(),
]

# -- Internal Simulation State Memory -----------------------------------------
mk.MEMORY.update({
    "cup_aligned": False,
    "cup_seated": False,
    "ring_locked": False,
    "proof_state": "idle",
    "proof_timer": 0,
    "proof_start_z": 0.0,
    "proof_slip_m": 0.0,
    "locked_joint_engaged": False,
})


def _update_mechanics(objects: dict):
    """Step callback executing physics contact and proof checks."""
    M = mk.MEMORY

    cup_p, cup_q = kit.pose(objects, "paint_cup")
    gun_p, _ = kit.pose(objects, "spray_gun")
    ring_p, _ = kit.pose(objects, "locking_ring")

    # Current inlet coordinates relative to gun pose
    inlet_cur_x = gun_p[0]
    inlet_cur_y = gun_p[1]
    inlet_cur_z = gun_p[2] + (INLET_SEAT_Z - GUN_STAND_Z)

    # Radial XY offset of cup from fluid inlet axis
    xy_offset = math.hypot(cup_p[0] - inlet_cur_x, cup_p[1] - inlet_cur_y)
    # Cup vertical distance: seated when cup_p[2] is near CUP_SEATED_Z
    z_diff = cup_p[2] - (gun_p[2] + (CUP_SEATED_Z - GUN_STAND_Z))

    # Compute cup tilt angle from vertical [0, 0, 1]
    # Quaternion: [x, y, z, w]
    qx, qy, qz, qw = cup_q
    # Up vector z-component: 1 - 2*(qx^2 + qy^2)
    up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    up_z = max(-1.0, min(1.0, up_z))
    tilt_rad = math.acos(up_z)
    tilt_deg = math.degrees(tilt_rad)

    # 1. Alignment check (hovering within 25 mm XY and within 50 mm above inlet)
    aligned = bool(
        xy_offset <= 0.025
        and 0.000 <= z_diff <= 0.055
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
    # Ring hinge angle: 0 deg = unlocked, negative for clockwise rotation (down to -45 deg)
    try:
        ring_angle = kit.q_deg(objects, "ring_latch")
    except Exception:
        # Fallback using ring pose
        ring_angle = 0.0

    # Locking condition: cup is seated and ring is rotated clockwise >= 40 deg
    ring_locked = bool(seated and abs(ring_angle) >= 40.0)
    M["ring_locked"] = ring_locked

    # Engage rigid locking catch once ring is turned past 40 deg
    if ring_locked and not M.get("locked_joint_engaged"):
        M["locked_joint_engaged"] = True
        kit.engage("cup_bayonet_lock")

    # 4. Proof pull evaluation:
    # Triggered once ring is locked, hands are clear, and test bench applies 20 N upward pull
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
            # Apply active 20 N upward retention proof pull
            kit.pull("paint_cup", (0.0, 0.0, PROOF_PULL_FORCE_N))
            M["proof_timer"] = M.get("proof_timer", 0) + 1
            slip = abs(cup_p[2] - M.get("proof_start_z", cup_p[2]))
            M["proof_slip_m"] = max(M.get("proof_slip_m", 0.0), slip)

            if slip > PROOF_MAX_SLIP_M:
                M["proof_state"] = "failed"
                kit.pull("paint_cup")  # stop pull
            elif M["proof_timer"] >= PROOF_HOLD_STEPS:
                M["proof_state"] = "held"
                kit.pull("paint_cup")  # stop pull
    elif not ring_locked and hands_clear and proof_state == "pulling":
        # Loose cup pulled loose immediately
        M["proof_state"] = "failed"
        kit.pull("paint_cup")


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
    """Score 0 if cup dropped, overturned, knocked off bench, or proof failed."""
    p, q = kit.pose(objects, "paint_cup")

    # Dropped off bench
    if p[2] < TOP - 0.020:
        return True

    # Extreme tilt: overturned > 45 deg
    qx, qy, qz, qw = q
    up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    if up_z < math.cos(math.radians(45.0)):
        return True

    # Proof failed (cup popped out or slipped > 0.8 mm under proof pull)
    if mk.MEMORY.get("proof_state") == "failed":
        return True

    return False


# -- Public Scorer API ---------------------------------------------------------


def score(objects: dict) -> float:
    """Staged scoring:

    0.25: Paint cup lifted and aligned within 25 mm XY of fluid inlet axis.
    0.50: Paint cup spout seated coaxially into fluid inlet boss (axial depth >= 12 mm).
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
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
except ImportError:
    FINGER_CLOSED = 0.0
    FINGER_OPEN = -0.04695


class Waypoint:

    def __init__(
        self,
        duration: float,
        pos: tuple[float, float, float] | None = None,
        finger: float | None = None,
        label: str = "",
        yaw: float = 0.0,
        *,
        xyz: tuple[float, float, float] | None = None,
        gripper: float | None = None,
    ):
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


def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("SPRAY_CHEAT", "")

    if cheat == "nothing":
        left = [
            Waypoint(10.0, (0.20, -0.25, TOP + 0.180), FINGER_OPEN, "park_left")
        ]
        right = [
            Waypoint(10.0, (0.20, 0.25, TOP + 0.180), FINGER_OPEN, "park_right")
        ]
        return {"left": left, "right": right}

    # Left arm counter-holds spray gun handle upright throughout operation
    left = [
        # Hover near spray gun handle
        Waypoint(1.2, (GUN_X - 0.040, GUN_Y, TOP + 0.100), FINGER_OPEN, "hover_gun_handle"),
        # Move forward to grip handle securely
        Waypoint(1.0, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_OPEN, "approach_handle"),
        Waypoint(0.8, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_CLOSED, "brace_gun_handle"),
        # Maintain rigid counter-hold
        Waypoint(12.0, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_CLOSED, "counter_hold_brace"),
    ]

    # Right arm manipulates cup and collar
    right = [
        # Hover over paint cup
        Waypoint(1.2, (CUP_START_X, CUP_START_Y, TOP + 0.220), FINGER_OPEN, "hover_cup"),
        # Descend to grip cup body
        Waypoint(1.0, (CUP_START_X, CUP_START_Y, TOP + 0.140), FINGER_OPEN, "descend_cup"),
        Waypoint(0.8, (CUP_START_X, CUP_START_Y, TOP + 0.140), FINGER_CLOSED, "grip_cup"),
        # Lift cup vertically clear of table
        Waypoint(1.2, (CUP_START_X, CUP_START_Y, TOP + 0.240), FINGER_CLOSED, "lift_cup"),
        # Transfer cup horizontally above fluid inlet boss
        Waypoint(1.5, (INLET_X, INLET_Y, TOP + 0.240), FINGER_CLOSED, "align_above_inlet"),
        # Lower cup into alignment zone (Stage 1 credit: 0.25)
        Waypoint(1.0, (INLET_X, INLET_Y, TOP + 0.220), FINGER_CLOSED, "hover_inlet_aligned"),
    ]

    if cheat == "align_only":
        right.extend([
            Waypoint(2.0, (INLET_X, INLET_Y, TOP + 0.220), FINGER_CLOSED, "hold_aligned"),
            Waypoint(4.0, (0.20, 0.25, TOP + 0.200), FINGER_OPEN, "retreat_right"),
        ])
        left.append(
            Waypoint(6.0, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_CLOSED, "brace_hold")
        )
        return {"left": left, "right": right}

    # Seat cup axially into fluid inlet (Stage 2 credit: 0.50)
    right.extend([
        Waypoint(1.2, (INLET_X, INLET_Y, CUP_SEATED_Z), FINGER_CLOSED, "seat_spout_in_boss"),
        # Release cup grip and clear up 30 mm
        Waypoint(0.8, (INLET_X, INLET_Y, CUP_SEATED_Z), FINGER_OPEN, "release_cup_seat"),
        Waypoint(0.8, (INLET_X, INLET_Y, TOP + 0.250), FINGER_OPEN, "clear_above_cup"),
    ])

    if cheat == "seat_only":
        # Cup seated but ring not turned
        right.extend([
            Waypoint(1.5, (0.20, 0.25, TOP + 0.200), FINGER_OPEN, "retreat_right"),
            Waypoint(5.0, (0.20, 0.25, TOP + 0.200), FINGER_OPEN, "park_right"),
        ])
        left.append(
            Waypoint(6.5, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_CLOSED, "brace_hold")
        )
        return {"left": left, "right": right}

    # Engage locking cam collar ring:
    # Right arm descends to outer collar tab at (INLET_X + 0.020, INLET_Y, RING_Z)
    right.extend([
        Waypoint(1.2, (INLET_X + 0.020, INLET_Y, RING_Z + 0.020), FINGER_OPEN, "approach_ring_tab", yaw=0.0),
        Waypoint(0.8, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "pinch_ring_tab", yaw=0.0),
        # Rotate collar ring 45 deg clockwise (yaw: 0.0 -> -0.785 rad) to engage cam detent (Stage 3 credit: 0.75)
        Waypoint(1.5, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_CLOSED, "rotate_collar_lock", yaw=-0.785),
        # Disengage from collar
        Waypoint(0.6, (INLET_X + 0.020, INLET_Y, RING_Z), FINGER_OPEN, "release_ring_tab", yaw=-0.785),
        Waypoint(0.8, (INLET_X, INLET_Y, TOP + 0.250), FINGER_OPEN, "clear_above_collar", yaw=0.0),
    ])

    if cheat == "no_release":
        # Grip cup and hold in place without completing proof and release
        right.extend([
            Waypoint(1.0, (INLET_X, INLET_Y, CUP_SEATED_Z), FINGER_CLOSED, "hold_unreleased"),
            Waypoint(6.0, (INLET_X, INLET_Y, CUP_SEATED_Z), FINGER_CLOSED, "hold_unreleased_dwell"),
        ])
        left.append(
            Waypoint(7.0, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_CLOSED, "brace_hold")
        )
        return {"left": left, "right": right}

    # Stage 4: Retention proof tug & clear arms
    right.extend([
        # Grip upper rim of mounted cup for proof tug
        Waypoint(1.0, (INLET_X, INLET_Y, CUP_SEATED_Z + 0.040), FINGER_OPEN, "approach_cup_rim"),
        Waypoint(0.6, (INLET_X, INLET_Y, CUP_SEATED_Z + 0.040), FINGER_CLOSED, "grip_cup_rim"),
        # Tug upward against mechanical bayonet lock to verify positive engagement
        Waypoint(1.2, (INLET_X, INLET_Y, CUP_SEATED_Z + 0.055), FINGER_CLOSED, "manual_proof_tug"),
        # Release cup rim
        Waypoint(0.6, (INLET_X, INLET_Y, CUP_SEATED_Z + 0.040), FINGER_OPEN, "release_proof_grip"),
        # Retreat clear >= 25 mm
        Waypoint(1.2, (0.20, 0.25, TOP + 0.200), FINGER_OPEN, "retreat_right_clear"),
        Waypoint(4.0, (0.20, 0.25, TOP + 0.200), FINGER_OPEN, "park_right_clear"),
    ])

    left.extend([
        # Release spray gun handle and retreat clear >= 25 mm
        Waypoint(0.8, (GUN_X, GUN_Y - 0.030, TOP + 0.080), FINGER_OPEN, "release_gun_handle"),
        Waypoint(1.2, (0.20, -0.25, TOP + 0.200), FINGER_OPEN, "retreat_left_clear"),
        Waypoint(4.0, (0.20, -0.25, TOP + 0.200), FINGER_OPEN, "park_left_clear"),
    ])

    return {"left": left, "right": right}
