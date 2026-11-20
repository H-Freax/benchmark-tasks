"""Reassemble a commercial manual citrus juicer: seat strainer cone and latch collector bowl.

On the station table: a heavy commercial countertop citrus press (Hamilton Beach Commercial
932 style, heavy cast iron frame with rubber suction feet) sits centered at x = 0.280, y = 0.000.
Following commercial sanitation and dishwashing teardown, the juicer's removable food-contact
components are staged disassembled on the prep table:
  1. A food-grade 304 stainless steel juice collector funnel bowl rests on the table to the
     right of the juicer at x = 0.220, y = -0.160.
  2. A matching perforated stainless steel strainer cone insert rests on the table to the
     left of the juicer at x = 0.220, y = +0.160.
  3. The juicer's sprung rack-and-pinion pressing lever arm rests in its raised rest position
     at 0 deg, holding the upper stainless compressor cone elevated above the collar receiver.

The bimanual robot must reassemble the unit for service:
  - Left arm braces the cast upright column to provide counter-torque.
  - Right arm picks up the collector bowl by its side handle, aligns it into the circular
    receiver collar of the base, lowers it into the collar, and rotates it 30 degrees clockwise
    to lock its bayonet rim tabs into the internal detents.
  - Right arm picks up the perforated strainer cone insert by its central reamer knob, carries
    it over, and lowers it coaxially into the collector bowl until seated flush on the internal shelf.
  - Left arm reaches up to the press lever arm and depresses it through a 15-degree mechanical
    test stroke, verifying that the rack-and-pinion plunger strokes smoothly into the seated
    strainer cone without collision, and allows the sprung lever to return to its rest stop.
  - Both arms withdraw clear of all components (clearance >= 25 mm) and hold stable for 1.0 s.

Run it:
  inspect-robots run --embodiment yam_arena -E task=tasks/reassemble-commercial-citrus-juicer/task.py \\
    -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1800 \\
    --instruction "Reassemble the commercial citrus juicer: seat the juice collector bowl into the base collar and rotate it to lock, drop the strainer cone flush inside, swing the press lever through a test stroke, and clear both arms." \\
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

TAG = "reassemble_commercial_citrus_juicer"
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Reassemble the commercial citrus juicer: seat the juice collector bowl"
    " into the base collar and rotate it to lock, drop the strainer cone"
    " flush inside, swing the press lever through a test stroke, and clear"
    " both arms."
)

# Colors
CAST_BLACK = (0.16, 0.17, 0.18)
CAST_IRON = (0.24, 0.25, 0.27)
STAINLESS_BRIGHT = (0.86, 0.88, 0.90)
STAINLESS_CONE = (0.78, 0.80, 0.82)
LEVER_CHROME = (0.92, 0.92, 0.94)
CHROME_KNOB = (0.12, 0.12, 0.14)

# Base Coordinates
BASE_X = 0.280
BASE_Y = 0.000
BASE_Z = TOP + 0.015

# Collar Center & Aperture (inside juicer base)
COLLAR_X = 0.280
COLLAR_Y = 0.000
COLLAR_Z = TOP + 0.110

# Collector Bowl initial & target poses
BOWL_INIT_POS = np.array([0.220, -0.160, TOP + 0.035])
BOWL_HANDLE_OFFSET_Y = 0.050
BOWL_SEAT_Z = TOP + 0.100

# Strainer Cone initial & target poses
CONE_INIT_POS = np.array([0.220, 0.160, TOP + 0.030])
CONE_SEAT_Z = TOP + 0.115

# Lever Hinge Anchor & Parameters
LEVER_ANCHOR = np.array([BASE_X - 0.030, 0.000, TOP + 0.350])
LEVER_AXIS = np.array([0.0, 1.0, 0.0])  # pitch rotation in X-Z
LEVER_REST_DEG = 0.0
LEVER_STROKE_THRESHOLD_DEG = -12.0  # At least 12 deg downward stroke to verify clearance

kit = mk.Kit()

# -- 1. Juicer Base Frame (Heavy static cast iron structure) -------------------
kit.part(
    name="juicer_base",
    pos=(BASE_X, BASE_Y, BASE_Z),
    boxes=[
        # Heavy horseshoe base plate (240 x 200 x 30 mm)
        mk.box((0.240, 0.200, 0.030), at=(0.000, 0.000, 0.000), color=CAST_BLACK),
        # Vertical upright spine / column (60 x 60 x 320 mm)
        mk.box((0.060, 0.060, 0.320), at=(-0.090, 0.000, 0.175), color=CAST_IRON),
        # Upper rack pinion housing at column head (100 x 70 x 60 mm)
        mk.box((0.100, 0.070, 0.060), at=(-0.030, 0.000, 0.335), color=CAST_IRON),
        # Receiver collar ring support arm extending forward from column
        mk.box((0.110, 0.040, 0.025), at=(-0.030, 0.000, 0.095), color=CAST_IRON),
        # Receiver collar ring perimeter walls (115 mm opening in X and Y)
        mk.box((0.015, 0.150, 0.025), at=(0.065, 0.000, 0.095), color=CAST_IRON),
        mk.box((0.015, 0.150, 0.025), at=(-0.065, 0.000, 0.095), color=CAST_IRON),
        mk.box((0.150, 0.015, 0.025), at=(0.000, 0.065, 0.095), color=CAST_IRON),
        mk.box((0.150, 0.015, 0.025), at=(0.000, -0.065, 0.095), color=CAST_IRON),
        # Internal bayonet stops inside collar
        mk.box((0.010, 0.015, 0.010), at=(0.055, 0.020, 0.082), color=CAST_BLACK),
        mk.box((0.010, 0.015, 0.010), at=(-0.055, 0.020, 0.082), color=CAST_BLACK),
        mk.box((0.010, 0.015, 0.010), at=(0.000, -0.055, 0.082), color=CAST_BLACK),
    ],
    mass=None,  # Solid bench-mounted unit (does not tip)
)

# -- 2. Sprung Press Lever & Plunger Assembly ----------------------------------
kit.part(
    name="press_lever",
    pos=(LEVER_ANCHOR[0], LEVER_ANCHOR[1], LEVER_ANCHOR[2]),
    boxes=[
        # Main chrome lever bar (200 mm long, 20 x 20 mm cross section)
        mk.box((0.200, 0.022, 0.022), at=(0.090, 0.000, 0.050), color=LEVER_CHROME),
        # Ergonomic top knob handle
        mk.box((0.040, 0.040, 0.040), at=(0.190, 0.000, 0.075), color=CHROME_KNOB),
        # Vertical plunger rack link
        mk.box((0.025, 0.025, 0.120), at=(0.030, 0.000, -0.050), color=LEVER_CHROME),
        # Upper stainless compressor pressing cup (cone shape)
        mk.box((0.070, 0.070, 0.030), at=(0.030, 0.000, -0.105), color=STAINLESS_BRIGHT),
    ],
    mass=0.850,
)

kit.hinge(
    name="lever_pivot",
    parent="juicer_base",
    child="press_lever",
    anchor=LEVER_ANCHOR,
    axis=LEVER_AXIS,
    lower_deg=-35.0,
    upper_deg=0.0,
    start_deg=0.0,
    stiffness=1.5,
    rest_deg=0.0,
    damping=0.4,
    friction=0.05,
)

# -- 3. Stainless Juice Collector Funnel Bowl ----------------------------------
kit.part(
    name="collector_bowl",
    pos=(BOWL_INIT_POS[0], BOWL_INIT_POS[1], BOWL_INIT_POS[2]),
    boxes=[
        # Main funnel cup walls (90 mm outer diameter, 55 mm high)
        mk.box((0.090, 0.015, 0.055), at=(0.000, 0.037, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.090, 0.015, 0.055), at=(0.000, -0.037, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.015, 0.090, 0.055), at=(0.037, 0.000, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.015, 0.090, 0.055), at=(-0.037, 0.000, 0.000), color=STAINLESS_BRIGHT),
        # Bottom drainage funnel base
        mk.box((0.050, 0.050, 0.015), at=(0.000, 0.000, -0.025), color=STAINLESS_BRIGHT),
        # Forward pouring spout
        mk.box((0.030, 0.020, 0.015), at=(0.050, 0.000, -0.020), color=STAINLESS_BRIGHT),
        # Grasp handle / rim flange (on +Y rim, 18 mm wide x 25 mm high)
        mk.box((0.018, 0.028, 0.030), at=(0.000, 0.050, 0.015), color=STAINLESS_BRIGHT),
        # Bayonet locking tabs on upper rim
        mk.box((0.012, 0.010, 0.008), at=(0.048, 0.010, 0.020), color=STAINLESS_BRIGHT),
        mk.box((0.012, 0.010, 0.008), at=(-0.048, 0.010, 0.020), color=STAINLESS_BRIGHT),
        # Internal circular support shelf for strainer cone
        mk.box((0.080, 0.080, 0.008), at=(0.000, 0.000, -0.010), color=STAINLESS_BRIGHT),
    ],
    mass=0.320,
)

# -- 4. Stainless Perforated Strainer Cone Insert ------------------------------
kit.part(
    name="strainer_cone",
    pos=(CONE_INIT_POS[0], CONE_INIT_POS[1], CONE_INIT_POS[2]),
    boxes=[
        # Outer rim seating lip (78 mm diameter, seats flush on internal bowl ledge)
        mk.box((0.078, 0.078, 0.006), at=(0.000, 0.000, -0.015), color=STAINLESS_CONE),
        # Conical reamer base (65 x 65 x 18 mm)
        mk.box((0.065, 0.065, 0.018), at=(0.000, 0.000, -0.003), color=STAINLESS_CONE),
        # Upper reamer cone body (45 x 45 x 20 mm)
        mk.box((0.045, 0.045, 0.020), at=(0.000, 0.000, 0.012), color=STAINLESS_CONE),
        # Central reamer apex knob (20 x 20 x 25 mm, grasped by robot fingers)
        mk.box((0.020, 0.020, 0.025), at=(0.000, 0.000, 0.030), color=STAINLESS_CONE),
    ],
    mass=0.180,
)

OBJECTS = kit.objects()


# -- Step Telemetry & State Detection ------------------------------------------

def _on_step(objects: dict):
    M = mk.MEMORY
    step_idx = mk._EP.get("step", 0)

    bp, bq = kit.pose(objects, "collector_bowl")
    cp, cq = kit.pose(objects, "strainer_cone")
    lever_deg = kit.q_deg(objects, "lever_pivot")

    # 1. Collector bowl placed in collar
    dr_bowl = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz_bowl = abs(bp[2] - BOWL_SEAT_Z)
    bowl_placed = dr_bowl <= 0.025 and dz_bowl <= 0.020
    if bowl_placed:
        M["bowl_placed"] = True

    # Check bowl bayonet twist angle
    b_mat = mk.quat_to_mat(bq)
    bowl_yaw_deg = math.degrees(math.atan2(b_mat[1, 0], b_mat[0, 0]))
    bowl_locked = bool(
        M.get("bowl_placed")
        and dr_bowl <= 0.020
        and dz_bowl <= 0.015
        and (bowl_yaw_deg <= -15.0 or bowl_yaw_deg >= 165.0)
    )
    if bowl_locked:
        M["bowl_locked"] = True

    # 2. Strainer cone nested coaxially in collector bowl
    dr_cone = math.sqrt((cp[0] - bp[0]) ** 2 + (cp[1] - bp[1]) ** 2)
    dz_cone = abs(cp[2] - (bp[2] + 0.015))
    cone_seated = bool(
        M.get("bowl_locked")
        and dr_cone <= 0.018
        and dz_cone <= 0.015
    )
    if cone_seated:
        M["cone_seated"] = True

    # 3. Press lever mechanical test stroke
    min_lever = min(M.get("min_lever_deg", 0.0), lever_deg)
    M["min_lever_deg"] = min_lever
    lever_stroked = bool(
        M.get("cone_seated")
        and min_lever <= LEVER_STROKE_THRESHOLD_DEG
        and lever_deg >= -6.0  # Returned to top rest stop
    )
    if lever_stroked:
        M["lever_stroked"] = True

    # 4. Robot hands cleared
    hands_clear = bool(
        _released(objects["collector_bowl"], 0.025)
        and _released(objects["strainer_cone"], 0.025)
        and _released(objects["press_lever"], 0.025)
    )

    # 5. Success hold timer
    if lever_stroked and hands_clear and not failed(objects):
        hold = M.get("hold_timer", 0) + 1
        M["hold_timer"] = hold
    else:
        M["hold_timer"] = 0

    if step_idx % 40 == 0 or (step_idx > 300 and step_idx % 10 == 0):
        print(
            f"[{TAG}] step={step_idx} dr_b={dr_bowl:.3f} dz_b={dz_bowl:.3f} yaw_b={bowl_yaw_deg:.1f} "
            f"dr_c={dr_cone:.3f} dz_c={dz_cone:.3f} lever={lever_deg:.1f} min_l={min_lever:.1f} "
            f"placed={M.get('bowl_placed')} locked={M.get('bowl_locked')} seated={M.get('cone_seated')} "
            f"stroke={M.get('lever_stroked')} clear={hands_clear} hold={M.get('hold_timer',0)} "
            f"score={score(objects)}",
            flush=True,
        )


kit.on_step(_on_step)


# -- Scorer Helpers ------------------------------------------------------------

def _released(obj_state: ObjectState, margin: float = 0.025) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    p = np.asarray(obj_state.pos, dtype=float)
    for tcp in tcps:
        if np.linalg.norm(np.asarray(tcp, dtype=float) - p) < margin:
            return False
    return True


def bowl_placed(objects: dict) -> bool:
    """True if collector bowl is seated in receiver collar."""
    bp, _ = kit.pose(objects, "collector_bowl")
    dr = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz = abs(bp[2] - BOWL_SEAT_Z)
    return bool(mk.MEMORY.get("bowl_placed") or (dr <= 0.025 and dz <= 0.020))


def bowl_locked(objects: dict) -> bool:
    """True if collector bowl is bayonet locked into base detents."""
    if not bowl_placed(objects):
        return False
    bp, bq = kit.pose(objects, "collector_bowl")
    dr = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz = abs(bp[2] - BOWL_SEAT_Z)
    b_mat = mk.quat_to_mat(bq)
    yaw_deg = math.degrees(math.atan2(b_mat[1, 0], b_mat[0, 0]))
    return bool(
        mk.MEMORY.get("bowl_locked")
        or (dr <= 0.020 and dz <= 0.015 and (yaw_deg <= -15.0 or yaw_deg >= 165.0))
    )


def cone_seated(objects: dict) -> bool:
    """True if strainer cone is coaxially nested flush in collector bowl."""
    if not bowl_locked(objects):
        return False
    bp, _ = kit.pose(objects, "collector_bowl")
    cp, _ = kit.pose(objects, "strainer_cone")
    dr = math.sqrt((cp[0] - bp[0]) ** 2 + (cp[1] - bp[1]) ** 2)
    dz = abs(cp[2] - (bp[2] + 0.015))
    return bool(mk.MEMORY.get("cone_seated") or (dr <= 0.018 and dz <= 0.015))


def lever_tested(objects: dict) -> bool:
    """True if press lever completed >= 12 deg test stroke and returned."""
    return bool(cone_seated(objects) and mk.MEMORY.get("lever_stroked"))


def fully_released(objects: dict) -> bool:
    """True if both robot hands are cleared >= 25 mm from all parts."""
    return bool(
        _released(objects["collector_bowl"], 0.025)
        and _released(objects["strainer_cone"], 0.025)
        and _released(objects["press_lever"], 0.025)
    )


def failed(objects: dict) -> bool:
    """Score 0 if base tipped/displaced, or parts dropped off bench."""
    bp, _ = kit.pose(objects, "collector_bowl")
    cp, _ = kit.pose(objects, "strainer_cone")
    base_p, _ = kit.pose(objects, "juicer_base")

    # Dropped off table
    if bp[2] < TOP - 0.020 or cp[2] < TOP - 0.020 or base_p[2] < TOP - 0.020:
        return True
    # Base moved/tipped > 20 mm
    if math.sqrt((base_p[0] - BASE_X) ** 2 + (base_p[1] - BASE_Y) ** 2) > 0.020:
        return True
    return False


# -- Public Scorer API ---------------------------------------------------------

def score(objects: dict) -> float:
    """Staged scoring:

    0.25: Collector bowl placed in collar opening (radial gap <= 25 mm, height seated).
    0.50: Collector bowl rotated >= 15 deg into bayonet lock detents.
    0.75: Strainer cone nested coaxially flush inside collector bowl (gap <= 18 mm).
    1.00: Press lever stroked >= 12 deg to test clearance and returned, hands clear >= 25 mm,
          and stationary for 1.0 s.
    Score 0.0: Displaced, dropped off bench, or base tipped.
    """
    if failed(objects):
        return 0.0

    s = 0.0
    if bowl_placed(objects):
        s += 0.25
        if bowl_locked(objects):
            s += 0.25
            if cone_seated(objects):
                s += 0.25
                if lever_tested(objects) and fully_released(objects):
                    if mk.MEMORY.get("hold_timer", 0) >= 20:
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
    cheat = os.environ.get("JUICER_CHEAT", "")

    if cheat == "nothing":
        left = [Waypoint(10.0, (0.20, 0.20, TOP + 0.150), FINGER_OPEN, "park_left")]
        right = [Waypoint(10.0, (0.20, -0.20, TOP + 0.150), FINGER_OPEN, "park_right")]
        return {"left": left, "right": right}

    # Handle coordinates on the bowl
    bowl_handle_x = BOWL_INIT_POS[0]
    bowl_handle_y = BOWL_INIT_POS[1] + BOWL_HANDLE_OFFSET_Y  # -0.160 + 0.050 = -0.110
    bowl_handle_z = BOWL_INIT_POS[2] + 0.015                # TOP + 0.050

    # Cone knob coordinates
    cone_knob_x = CONE_INIT_POS[0]                          # 0.220
    cone_knob_y = CONE_INIT_POS[1]                          # 0.160
    cone_knob_z = CONE_INIT_POS[2] + 0.030                  # TOP + 0.060

    # Left arm: braces upright spine on +Y side, later tests press lever stroke
    left = [
        # Hover approach towards spine
        Waypoint(1.2, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "brace_approach"),
        # Grip juicer upright column
        Waypoint(1.0, (BASE_X - 0.090, 0.035, TOP + 0.200), FINGER_CLOSED, "brace_grip"),
        # Hold brace while right arm installs bowl and cone (through t = 18.0 s)
        Waypoint(16.0, (BASE_X - 0.090, 0.035, TOP + 0.200), FINGER_CLOSED, "hold_brace"),
    ]

    # Right arm: handles bowl placement, bayonet twist, and cone seating
    right = [
        # Approach above bowl handle
        Waypoint(1.2, (bowl_handle_x, bowl_handle_y, TOP + 0.140), FINGER_OPEN, "bowl_approach"),
        # Descend to clamp bowl handle
        Waypoint(1.0, (bowl_handle_x, bowl_handle_y, bowl_handle_z + 0.010), FINGER_CLOSED, "bowl_grasp"),
        # Lift bowl clear of table
        Waypoint(1.0, (bowl_handle_x, bowl_handle_y, TOP + 0.180), FINGER_CLOSED, "bowl_lift"),
        # Carry bowl above base receiver collar (handle is at y = +0.050)
        Waypoint(1.5, (COLLAR_X, COLLAR_Y + BOWL_HANDLE_OFFSET_Y, TOP + 0.180), FINGER_CLOSED, "bowl_align"),
        # Lower bowl into collar receiver
        Waypoint(1.2, (COLLAR_X, COLLAR_Y + BOWL_HANDLE_OFFSET_Y, BOWL_SEAT_Z + 0.015), FINGER_CLOSED, "bowl_insert"),
    ]

    if cheat == "bowl_only":
        right += [
            Waypoint(1.0, (COLLAR_X, COLLAR_Y + BOWL_HANDLE_OFFSET_Y, BOWL_SEAT_Z + 0.015), FINGER_OPEN, "bowl_release"),
            Waypoint(2.0, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_retract"),
            Waypoint(8.0, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_park"),
        ]
        left += [
            Waypoint(1.0, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "left_retract"),
            Waypoint(8.0, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # Twist bowl 30 deg clockwise to engage bayonet detents
    # Handle rotates around collar center: x_new = 0.280 + 0.050*sin(30°) = 0.305, y_new = 0.050*cos(30°) = 0.043
    right += [
        Waypoint(1.5, (0.305, 0.043, BOWL_SEAT_Z + 0.015), FINGER_CLOSED, "bowl_twist", yaw=math.radians(-30.0)),
        # Release bowl
        Waypoint(0.8, (0.305, 0.043, BOWL_SEAT_Z + 0.025), FINGER_OPEN, "bowl_release", yaw=math.radians(-30.0)),
        # Lift away from bowl
        Waypoint(1.0, (0.305, 0.043, TOP + 0.200), FINGER_OPEN, "bowl_clear"),
    ]

    if cheat == "bowl_locked":
        right += [
            Waypoint(2.0, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_retract"),
            Waypoint(8.0, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_park"),
        ]
        left += [
            Waypoint(1.0, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "left_retract"),
            Waypoint(8.0, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # Right arm: pick up strainer cone
    right += [
        # Move above strainer cone knob
        Waypoint(1.5, (cone_knob_x, cone_knob_y, TOP + 0.140), FINGER_OPEN, "cone_approach"),
        # Descend to clamp cone knob
        Waypoint(1.0, (cone_knob_x, cone_knob_y, cone_knob_z + 0.005), FINGER_CLOSED, "cone_grasp"),
        # Lift cone clear
        Waypoint(1.0, (cone_knob_x, cone_knob_y, TOP + 0.200), FINGER_CLOSED, "cone_lift"),
        # Align cone coaxially above collector bowl
        Waypoint(1.5, (COLLAR_X, COLLAR_Y, TOP + 0.200), FINGER_CLOSED, "cone_align"),
        # Lower cone into collector bowl until flush on internal shelf
        Waypoint(1.2, (COLLAR_X, COLLAR_Y, CONE_SEAT_Z + 0.030), FINGER_CLOSED, "cone_seat"),
    ]

    if cheat == "no_release":
        left += [
            Waypoint(0.8, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "brace_release"),
            Waypoint(1.5, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.440), FINGER_OPEN, "lever_approach"),
            Waypoint(0.8, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.420), FINGER_CLOSED, "lever_grasp"),
            Waypoint(1.5, (LEVER_ANCHOR[0] + 0.170, 0.000, TOP + 0.350), FINGER_CLOSED, "lever_stroke"),
            Waypoint(0.8, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.440), FINGER_OPEN, "lever_release"),
            Waypoint(1.5, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_retract"),
            Waypoint(5.0, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_park"),
        ]
        right += [
            Waypoint(10.0, (COLLAR_X, COLLAR_Y, CONE_SEAT_Z + 0.030), FINGER_CLOSED, "cone_hold"),
        ]
        return {"left": left, "right": right}

    # Normal full demo: release cone and withdraw right arm
    right += [
        Waypoint(0.8, (COLLAR_X, COLLAR_Y, CONE_SEAT_Z + 0.045), FINGER_OPEN, "cone_release"),
        Waypoint(1.2, (COLLAR_X, COLLAR_Y, TOP + 0.220), FINGER_OPEN, "cone_clear"),
        Waypoint(1.5, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_retract"),
        Waypoint(8.0, (0.180, -0.220, TOP + 0.200), FINGER_OPEN, "right_park"),
    ]

    if cheat == "cone_seated":
        left += [
            Waypoint(1.0, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "left_retract"),
            Waypoint(8.0, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # Left arm: release column brace, stroke press lever, and retract clear
    left += [
        Waypoint(0.8, (BASE_X - 0.090, 0.070, TOP + 0.220), FINGER_OPEN, "brace_release"),
        Waypoint(1.5, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.440), FINGER_OPEN, "lever_approach"),
        Waypoint(0.8, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.420), FINGER_CLOSED, "lever_grasp"),
        Waypoint(1.5, (LEVER_ANCHOR[0] + 0.170, 0.000, TOP + 0.350), FINGER_CLOSED, "lever_stroke"),
        Waypoint(0.8, (LEVER_ANCHOR[0] + 0.180, 0.000, TOP + 0.440), FINGER_OPEN, "lever_release"),
        Waypoint(1.5, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_retract"),
        Waypoint(5.0, (0.180, 0.220, TOP + 0.200), FINGER_OPEN, "left_park"),
    ]

    return {"left": left, "right": right}
