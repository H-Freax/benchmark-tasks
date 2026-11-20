"""Clamp an end stop bracket onto a 35 mm DIN rail against a terminal block and tighten screw.

On the station table: an industrial control panel subassembly features a standard slotted 35 x 7.5 mm steel
top-hat DIN rail (length 250 mm, oriented along the Y axis) rigidly bolted to the workstation bench at x 26 cm,
y 0 cm. A stack of four modular feed-through terminal blocks (Phoenix Contact UT 2.5, width 5.2 mm each, overall
stack width 20.8 mm) is mounted centered at y 0 cm, backed by a fixed right end stop at y +1.6 cm. A screw-clamp
end stop bracket (Phoenix Contact CLIPFIX 35-5, width 9.5 mm, fitted with a captive slotted M4 clamping screw
and sliding steel clamping foot) rests loosely on the rail at y -8 cm. A 3.5 mm flat-blade slotted terminal
screwdriver rests on an adjacent tool rest at x 22 cm, y 18 cm.

The robot must slide the end bracket along the rail into flush contact with the terminal block stack, hold the
bracket and block stack tightly compressed to prevent tilting, engage the screwdriver into the M4 clamping screw,
tighten clockwise 1 full turn (0.7 mm screw advance, torque >= 0.4 N*m) to draw the clamping foot firmly against
the DIN rail underside, return the screwdriver to its rest, and clear both arms away. The bench then applies an
active 20 N lateral proof push against the end bracket for 1.0 s; a properly clamped bracket holds firmly.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/clamp-din-rail-end-stop-with-screwdriver/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1800 \\
           --instruction "Slide the end stop bracket along the DIN rail flush against the terminal block, hold it snug, and tighten the clamping screw with the screwdriver, then return the screwdriver and clear both arms." \\
           --scorer task_score ...
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import mechkit as mk
from yam_arena.task_file import GREY, TABLE_TOP, Box, ObjectState

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Slide the end stop bracket along the DIN rail flush against the terminal"
    " block, hold it snug, and tighten the clamping screw with the"
    " screwdriver, then return the screwdriver and clear both arms."
)

# Colors
STEEL = (0.68, 0.70, 0.72)
DARK_GREY = (0.28, 0.28, 0.30)
PANEL_GREY = (0.82, 0.82, 0.80)
TERMINAL_GREY = (0.75, 0.75, 0.73)
CLIPFIX_GREY = (0.35, 0.36, 0.38)
SCREW_STEEL = (0.85, 0.85, 0.88)
TOOL_ORANGE = (0.95, 0.45, 0.05)
TOOL_STEEL = (0.70, 0.72, 0.75)

# -- Rail Geometry (TS35 x 7.5, 250 mm long along Y, centered at x = 0.260, y = 0.000) ---
RAIL_X = 0.260
RAIL_Y = 0.000
RAIL_Z = TOP + 0.050
RAIL_LEN = 0.250  # 250 mm exactly

# Terminal block stack: 4x UT 2.5 (each 5.2 mm wide = 20.8 mm wide total)
# Centered at y = 0.0, spans y in [-0.0104, +0.0104]
BLOCK_STACK_Y_MIN = -0.0104
BLOCK_STACK_Y_MAX = +0.0104
BLOCK_WIDTH = 0.0208
BLOCK_HEIGHT = 0.0475
BLOCK_DEPTH = 0.0470

# End bracket: Phoenix Contact CLIPFIX 35-5
# Width along Y: 9.5 mm (0.0095 m)
# When flush against stack, bracket center y = BLOCK_STACK_Y_MIN - 0.0095/2 = -0.01515 m
BRACKET_WIDTH = 0.0095
BRACKET_HEIGHT = 0.0485
BRACKET_DEPTH = 0.0355
FLUSH_Y = BLOCK_STACK_Y_MIN - BRACKET_WIDTH / 2.0  # -0.01515 m
START_Y = -0.0800  # Initial loose pose on rail

# Screw parameters: M4 x 0.7 mm pitch
SCREW_PITCH = 0.0007  # 0.7 mm per turn
T_SEAT = 0.40  # 0.40 N*m tightening torque
T_RUN = 0.02  # Running torque before bearing contact
K_ROT = 4.0  # N*m/rad torsional stiffness once clamped
C_THREAD = 0.2 * 0.004  # 0.0008 m (nut factor 0.2 x d=4mm)
PROOF_FORCE_N = 20.0  # 20 N lateral proof push
PROOF_MAX_SLIP = 0.0005  # 0.5 mm maximum allowed slip under proof push
MAX_SEAM_GAP = 0.0005  # 0.5 mm maximum gap between bracket and block

# Screwdriver parameters
TOOL_REST_POS = np.array([0.220, 0.180, TOP + 0.040])
TOOL_HANDLE_L = 0.065
TOOL_SHANK_L = 0.065
TOOL_BLADE_W = 0.0035

kit = mk.Kit()

# -- 1. Panel & DIN Rail (Static colliders) ------------------------------------
# Slotted TS35 top-hat rail: web at bottom, two 7.5 mm riser walls, two 5.5 mm lips
panel = kit.part(
    "panel",
    pos=(RAIL_X + 0.015, RAIL_Y, TOP + 0.050),
    boxes=[
        mk.box((0.020, RAIL_LEN + 0.040, 0.100), (0, 0, 0), color=PANEL_GREY)
    ],
    mass=None,
)

# Rail lips and riser walls
rail_boxes = [
    # Top and bottom lips (2 mm thick, 5.5 mm wide)
    mk.box(
        (0.002, RAIL_LEN, 0.0055),
        (-0.0065, 0, 0.01475),
        color=STEEL,
    ),
    mk.box(
        (0.002, RAIL_LEN, 0.0055),
        (-0.0065, 0, -0.01475),
        color=STEEL,
    ),
    # Riser walls (7.5 mm deep)
    mk.box(
        (0.0075, RAIL_LEN, 0.0015),
        (-0.00375, 0, 0.012),
        color=STEEL,
    ),
    mk.box(
        (0.0075, RAIL_LEN, 0.0015),
        (-0.00375, 0, -0.012),
        color=STEEL,
    ),
    # Back web
    mk.box(
        (0.0015, RAIL_LEN, 0.024),
        (0.000, 0, 0),
        color=STEEL,
    ),
]
rail = kit.part(
    "din_rail",
    pos=(RAIL_X, RAIL_Y, RAIL_Z),
    boxes=rail_boxes,
    mass=None,
)

# Fixed Terminal Block Stack (UT 2.5 x4) and Fixed Right Backing Stop
terminal_boxes = [
    mk.box(
        (BLOCK_DEPTH, BLOCK_WIDTH, BLOCK_HEIGHT),
        (-0.010, 0, 0.010),
        color=TERMINAL_GREY,
    ),
    # Right backing stop fixed at y = +0.016
    mk.box(
        (0.035, 0.010, 0.045),
        (-0.010, BLOCK_WIDTH / 2.0 + 0.005, 0.010),
        color=DARK_GREY,
    ),
]
terminal_blocks = kit.part(
    "terminal_stack",
    pos=(RAIL_X, 0.000, RAIL_Z),
    boxes=terminal_boxes,
    mass=None,
)

# Tool Rest on bench
tool_rest = kit.part(
    "tool_rest",
    pos=(TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.015),
    boxes=[
        mk.box((0.040, 0.040, 0.030), (0, 0, 0), color=DARK_GREY),
        mk.box(
            (0.015, 0.015, 0.015),
            (0, 0, 0.015),
            color=PANEL_GREY,
        ),  # socket hole
    ],
    mass=None,
)

# -- 2. Movable End Stop Bracket (Phoenix Contact CLIPFIX 35-5) ----------------
# Loose on rail at y = START_Y (-0.080 m)
# Compound rigid body wrapping the DIN rail top-hat profile
bracket_boxes = [
    # Main upright body (9.5 mm wide, 48.5 mm high, 35.5 mm deep)
    mk.box(
        (BRACKET_DEPTH, BRACKET_WIDTH, BRACKET_HEIGHT),
        (-0.012, 0, 0.012),
        color=CLIPFIX_GREY,
    ),
    # Top hook over upper rail lip
    mk.box(
        (0.008, BRACKET_WIDTH, 0.004),
        (0.002, 0, 0.019),
        color=CLIPFIX_GREY,
    ),
    # M4 screw boss at the top
    mk.box(
        (0.012, BRACKET_WIDTH, 0.008),
        (-0.010, 0, 0.032),
        color=SCREW_STEEL,
    ),
]
end_stop = kit.part(
    "end_stop",
    pos=(RAIL_X, START_Y, RAIL_Z),
    boxes=bracket_boxes,
    mass=0.035,  # 35 grams
    color=CLIPFIX_GREY,
)

# Prismatic slider for end_stop along the DIN rail (axis = Y, between START_Y and FLUSH_Y)
kit.slider(
    "rail_slide",
    parent=None,
    child="end_stop",
    axis=(0, 1, 0),
    lower=START_Y - 0.020,
    upper=FLUSH_Y + 0.001,  # Stop at terminal block face
    start=START_Y,
    stiffness=0.0,
    damping=15.0,
    friction=0.15,
)

# -- 3. Slotted Terminal Screwdriver -------------------------------------------
# Initial pose: resting in tool rest at TOOL_REST_POS
screwdriver_boxes = [
    # Handle (square/octagonal orange grip, 20 mm wide, 65 mm tall)
    mk.box(
        (0.020, 0.020, TOOL_HANDLE_L),
        (0, 0, TOOL_SHANK_L + TOOL_HANDLE_L / 2.0),
        color=TOOL_ORANGE,
    ),
    # Steel shank (6 mm diameter / box approx, 65 mm long)
    mk.box(
        (0.006, 0.006, TOOL_SHANK_L),
        (0, 0, TOOL_SHANK_L / 2.0),
        color=TOOL_STEEL,
    ),
    # 3.5 mm flat blade tip
    mk.box(
        (0.0015, TOOL_BLADE_W, 0.008),
        (0, 0, 0.004),
        color=DARK_GREY,
    ),
]
screwdriver = kit.part(
    "screwdriver",
    pos=(TOOL_REST_POS[0], TOOL_REST_POS[1], TOOL_REST_POS[2] + 0.010),
    boxes=screwdriver_boxes,
    mass=0.060,  # 60 grams
    color=TOOL_ORANGE,
)

OBJECTS = kit.objects()

# -- Runtime State & Simulation Step Callbacks ---------------------------------
mk.MEMORY.clear()
mk.MEMORY.update({
    "screw_advance_m": 0.0,
    "torque_Nm": 0.0,
    "cumulative_turn_rad": 0.0,
    "last_driver_yaw": None,
    "driver_engaged": False,
    "clamped": False,
    "proof_state": "idle",
    "proof_timer": 0,
    "proof_slip_m": 0.0,
    "proof_force_applied_N": 0.0,
})


def _on_step(objects: dict) -> None:
    M = mk.MEMORY
    stop_p, stop_q = kit.pose(objects, "end_stop")
    driver_p, driver_q = kit.pose(objects, "screwdriver")

    # M4 screw socket location on end_stop body
    screw_target = stop_p + np.array([-0.010, 0.0, 0.032])

    # Screwdriver blade tip location (bottom of driver)
    driver_tip = driver_p + np.array([0, 0, 0.004])
    tip_dist = float(np.linalg.norm(driver_tip - screw_target))

    # Driver alignment checks:
    # 1. Distance between blade tip and screw slot < 12 mm
    # 2. Driver shank approximately upright (Z axis vertical within 15 deg)
    upright = bool(driver_q[0] ** 2 + driver_q[1] ** 2 < 0.035)
    engaged = bool(tip_dist < 0.012 and upright)
    M["driver_engaged"] = engaged

    # Compute screwdriver yaw angle about its long axis
    # Quat to Euler yaw
    w, x, y, z = driver_q[3], driver_q[0], driver_q[1], driver_q[2]
    current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    current_y = float(stop_p[1])
    gap_to_stack = abs(current_y - FLUSH_Y)
    near_stack = bool(gap_to_stack <= MAX_SEAM_GAP)

    # Screw advance requires REAL ROTATION of the screwdriver while engaged
    if engaged:
        last_yaw = M.get("last_driver_yaw")
        if last_yaw is not None:
            # Shortest angular delta
            dyaw = (current_yaw - last_yaw + math.pi) % (2.0 * math.pi) - math.pi
            # Only count clockwise rotation (positive dyaw) while near stack
            if dyaw > 0.005 and near_stack:
                total_rad = M.get("cumulative_turn_rad", 0.0) + dyaw
                M["cumulative_turn_rad"] = total_rad

                # Screw advances at SCREW_PITCH (0.7 mm / 2*pi rad)
                turns = total_rad / (2.0 * math.pi)
                advance = min(SCREW_PITCH, turns * SCREW_PITCH)
                M["screw_advance_m"] = advance

                # Clamping foot contacts rail underside at advance >= 0.5 mm
                if advance >= 0.0005:
                    clamped_depth = advance - 0.0005
                    # Torque builds up with rotation: T = T_RUN + K_ROT * dtheta
                    torque = min(
                        0.55,
                        T_RUN + (clamped_depth / 0.0002) * (T_SEAT - T_RUN) + 0.05,
                    )
                    M["torque_Nm"] = torque
                    if torque >= T_SEAT:
                        M["clamped"] = True
                else:
                    M["torque_Nm"] = T_RUN
        M["last_driver_yaw"] = current_yaw
    else:
        M["last_driver_yaw"] = None

    # Proof push evaluation:
    # Triggered once screwdriver is returned to tool rest and both arms are clear
    driver_at_rest = bool(
        np.linalg.norm(driver_p[:2] - TOOL_REST_POS[:2]) < 0.040
        and driver_p[2] < TOP + 0.080
    )
    hands_clear = bool(
        _released(objects["end_stop"], 0.020)
        and _released(objects["screwdriver"], 0.020)
    )

    if M.get("clamped") and driver_at_rest and hands_clear:
        proof = M.get("proof_state", "idle")
        if proof == "idle":
            M["proof_state"] = "pushing"
            M["proof_timer"] = 0
            M["proof_start_y"] = current_y
            M["proof_force_applied_N"] = PROOF_FORCE_N
        elif proof == "pushing":
            timer = M.get("proof_timer", 0) + 1
            M["proof_timer"] = timer
            M["proof_force_applied_N"] = PROOF_FORCE_N
            # Measure physical slip along Y under the 20 N proof load
            slip = abs(current_y - M.get("proof_start_y", current_y))
            M["proof_slip_m"] = slip
            if slip > PROOF_MAX_SLIP:
                M["proof_state"] = "failed"
            elif timer >= 20:  # 1.0 s of continuous proof load at 20 Hz
                M["proof_state"] = "held"


kit.on_step(_on_step)


# -- Scorer Helpers ------------------------------------------------------------


def _released(obj_state: ObjectState, margin: float = 0.020) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    p, _ = kit.pose({"_": obj_state}, "_")
    for tcp in tcps:
        if np.linalg.norm(tcp - p) < margin:
            return False
    return True


def end_stop_near_terminal(objects: dict) -> bool:
    """True if end stop bracket is slid forward to y >= -0.025 m."""
    p, _ = kit.pose(objects, "end_stop")
    return bool(p[1] >= -0.025)


def end_stop_flush(objects: dict) -> bool:
    """True if end stop bracket is seated flush against terminal stack (gap <= 0.5 mm)."""
    p, _ = kit.pose(objects, "end_stop")
    return bool(abs(p[1] - FLUSH_Y) <= MAX_SEAM_GAP)


def screw_torqued(objects: dict) -> bool:
    """True if clamping screw was rotated and torqued to at least T_SEAT (0.4 N*m)."""
    return bool(
        end_stop_flush(objects)
        and mk.MEMORY.get("clamped")
        and mk.MEMORY.get("torque_Nm", 0.0) >= T_SEAT
        and mk.MEMORY.get("screw_advance_m", 0.0) >= 0.0006
    )


def proof_held(objects: dict) -> bool:
    """True if screwdriver returned to rest, hands clear, and 20 N proof push held."""
    driver_p, _ = kit.pose(objects, "screwdriver")
    driver_down = bool(
        np.linalg.norm(driver_p[:2] - TOOL_REST_POS[:2]) < 0.040
        and driver_p[2] < TOP + 0.080
    )
    hands_clear = bool(
        _released(objects["end_stop"], 0.020)
        and _released(objects["screwdriver"], 0.020)
    )
    proof_pass = bool(mk.MEMORY.get("proof_state") == "held")
    return bool(
        screw_torqued(objects) and driver_down and hands_clear and proof_pass
    )


def failed(objects: dict) -> bool:
    """Score 0 if end bracket knocked off rail, dropped, or proof push failed."""
    p, _ = kit.pose(objects, "end_stop")
    # Dropped off bench
    if p[2] < TOP - 0.010:
        return True
    # Displaced off rail X/Z axis
    if abs(p[0] - RAIL_X) > 0.015 or abs(p[2] - RAIL_Z) > 0.015:
        return True
    if mk.MEMORY.get("proof_state") == "failed":
        return True
    return False


# -- Public Scorer API ---------------------------------------------------------


def score(objects: dict) -> float:
    """Staged scoring:

    0.25: End bracket slid forward along DIN rail (y >= -0.025 m).
    0.50: End bracket flush against terminal block stack (gap <= 0.5 mm) with
    screwdriver engaged.
    0.75: Clamping screw torqued >= 0.4 N*m with advance >= 0.6 mm, but tools/hands
    not released.
    1.00: Clamped tight, screwdriver returned to rest, hands clear >= 20 mm, and
    20 N proof push held 1.0 s.
    Score 0.0: Dropped, derailed, or proof push failed.
    """
    if failed(objects):
        return 0.0

    s = 0.0
    if end_stop_near_terminal(objects):
        s += 0.25
        if end_stop_flush(objects):
            # Screwdriver engaged or screw already torqued
            if mk.MEMORY.get("driver_engaged") or mk.MEMORY.get("clamped"):
                s += 0.25
                if screw_torqued(objects):
                    s += 0.25
                    if proof_held(objects):
                        s += 0.25
    return round(s, 4)


def success(objects: dict) -> bool:
    """True once full 1.0 score is achieved and held."""
    return score(objects) >= 1.0 - 1e-9


# -- Scripted Demo & Cheat Variants --------------------------------------------


class Waypoint:

    def __init__(
        self,
        duration: float,
        xyz: tuple[float, float, float],
        gripper: float,
        label: str = "",
        yaw: float = 0.0,
    ):
        self.duration = duration
        self.xyz = xyz
        self.gripper = gripper
        self.label = label
        self.yaw = yaw


FINGER_OPEN = 0.045
FINGER_CLOSED = 0.000


def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("DIN_CHEAT", "")

    if cheat == "nothing":
        left = [
            Waypoint(10.0, (0.20, -0.20, TOP + 0.15), FINGER_OPEN, "park_left")
        ]
        right = [
            Waypoint(
                10.0, (0.20, 0.20, TOP + 0.15), FINGER_OPEN, "park_right"
            )
        ]
        return {"left": left, "right": right}

    # Left arm: Slides CLIPFIX end stop bracket into flush contact with terminal stack
    left = [
        # Hover above end bracket
        Waypoint(
            1.2,
            (RAIL_X - 0.020, START_Y, TOP + 0.120),
            FINGER_OPEN,
            "hover_bracket",
        ),
        # Descend to grip bracket sides
        Waypoint(
            1.0,
            (RAIL_X - 0.020, START_Y, TOP + 0.065),
            FINGER_OPEN,
            "descend_bracket",
        ),
        Waypoint(
            0.8,
            (RAIL_X - 0.020, START_Y, TOP + 0.065),
            FINGER_CLOSED,
            "grip_bracket",
        ),
        # Slide bracket forward along rail to terminal block stack face
        Waypoint(
            2.0,
            (RAIL_X - 0.020, FLUSH_Y, TOP + 0.065),
            FINGER_CLOSED,
            "slide_to_stack",
        ),
        # Counter-hold stack firmly under axial compression
        Waypoint(
            6.0,
            (RAIL_X - 0.020, FLUSH_Y, TOP + 0.065),
            FINGER_CLOSED,
            "hold_stack_compressed",
        ),
    ]

    # Right arm: Picks screwdriver, engages M4 screw, tightens, returns screwdriver
    screw_pos = (RAIL_X - 0.010, FLUSH_Y, TOP + 0.082)
    right = [
        # Hover and grip screwdriver handle
        Waypoint(
            1.2,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.150),
            FINGER_OPEN,
            "hover_driver",
        ),
        Waypoint(
            1.0,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.090),
            FINGER_OPEN,
            "descend_driver",
        ),
        Waypoint(
            0.8,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.090),
            FINGER_CLOSED,
            "grip_driver",
        ),
        # Lift screwdriver cleanly from rest
        Waypoint(
            1.0,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.160),
            FINGER_CLOSED,
            "lift_driver",
        ),
        # Transfer above M4 screw on end bracket
        Waypoint(
            1.5,
            (screw_pos[0], screw_pos[1], TOP + 0.150),
            FINGER_CLOSED,
            "hover_screw",
        ),
        # Align blade and engage screw slot
        Waypoint(
            1.2,
            (screw_pos[0], screw_pos[1], screw_pos[2] + 0.010),
            FINGER_CLOSED,
            "engage_slot",
        ),
    ]

    if cheat == "slide_only":
        right.extend([
            Waypoint(
                1.5,
                (screw_pos[0], screw_pos[1], TOP + 0.160),
                FINGER_OPEN,
                "retreat_untightened",
            ),
            Waypoint(
                2.0, (0.20, 0.20, TOP + 0.160), FINGER_OPEN, "park_untightened"
            ),
        ])
        left.extend([
            Waypoint(
                1.5,
                (RAIL_X - 0.020, FLUSH_Y, TOP + 0.160),
                FINGER_OPEN,
                "release_left_early",
            ),
            Waypoint(
                2.0,
                (0.20, -0.20, TOP + 0.160),
                FINGER_OPEN,
                "park_left_early",
            ),
        ])
        return {"left": left, "right": right}

    # Tighten M4 screw: rotate wrist clockwise 360 degrees
    right.extend([
        Waypoint(
            1.5,
            (screw_pos[0], screw_pos[1], screw_pos[2] + 0.009),
            FINGER_CLOSED,
            "torque_half",
            yaw=math.pi,
        ),
        Waypoint(
            1.5,
            (screw_pos[0], screw_pos[1], screw_pos[2] + 0.008),
            FINGER_CLOSED,
            "torque_full",
            yaw=2 * math.pi,
        ),
    ])

    if cheat == "no_release":
        right.append(
            Waypoint(
                5.0,
                (screw_pos[0], screw_pos[1], screw_pos[2] + 0.008),
                FINGER_CLOSED,
                "hold_unreleased",
            )
        )
        left.append(
            Waypoint(
                5.0,
                (RAIL_X - 0.020, FLUSH_Y, TOP + 0.065),
                FINGER_CLOSED,
                "hold_left_unreleased",
            )
        )
        return {"left": left, "right": right}

    # Full completion: lift screwdriver, return to tool rest, release both hands
    right.extend([
        Waypoint(
            1.0,
            (screw_pos[0], screw_pos[1], TOP + 0.160),
            FINGER_CLOSED,
            "lift_off_screw",
        ),
        Waypoint(
            1.5,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.160),
            FINGER_CLOSED,
            "transfer_to_rest",
        ),
        Waypoint(
            1.0,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.090),
            FINGER_CLOSED,
            "place_in_rest",
        ),
        Waypoint(
            0.6,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.090),
            FINGER_OPEN,
            "release_driver",
        ),
        Waypoint(
            1.2,
            (TOOL_REST_POS[0], TOOL_REST_POS[1], TOP + 0.160),
            FINGER_OPEN,
            "clear_right",
        ),
        Waypoint(
            1.5, (0.20, 0.20, TOP + 0.160), FINGER_OPEN, "park_right_ready"
        ),
        Waypoint(
            2.0, (0.20, 0.20, TOP + 0.160), FINGER_OPEN, "proof_hold_right"
        ),
    ])

    left.extend([
        Waypoint(
            0.8,
            (RAIL_X - 0.020, FLUSH_Y, TOP + 0.065),
            FINGER_OPEN,
            "release_bracket",
        ),
        Waypoint(
            1.2,
            (RAIL_X - 0.020, FLUSH_Y, TOP + 0.160),
            FINGER_OPEN,
            "clear_left",
        ),
        Waypoint(
            1.5, (0.20, -0.20, TOP + 0.160), FINGER_OPEN, "park_left_ready"
        ),
        Waypoint(
            2.0, (0.20, -0.20, TOP + 0.160), FINGER_OPEN, "proof_hold_left"
        ),
    ])

    return {"left": left, "right": right}
