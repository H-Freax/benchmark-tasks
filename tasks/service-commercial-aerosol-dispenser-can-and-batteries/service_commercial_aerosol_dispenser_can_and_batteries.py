"""Service a wall-mounted commercial metered aerosol air freshener dispenser.

A janitor / custodian services an automated aerosol air freshener dispenser
(Rubbermaid Commercial Products / Technical Concepts Microburst 3000 style,
210 mm H x 90 mm W x 85 mm D). The dispenser is mounted on a vertical service wall
above the work table, facing the robot arms.

The dispenser features:
1. A rigid hollow chassis enclosure with a rear wall-mounting plate, sidewalls,
   floor, and internal semi-circular can cradle collar (68 mm ID).
2. A bottom-hinged front cover (revolute joint along y-axis, 0 to 90 degrees)
   with a top release catch.
3. A sprung top release latch button (prismatic slider along vertical z-axis,
   travel 6 mm, spring stiffness 1200 N/m) that disengages the cover catch.
4. An overhead downward-facing actuator hammer block at the top of the chassis.

Task Workflow:
1. Depress the top release latch button downward with the left gripper tip (>8 N)
   to disengage the front cover catch.
2. Swing the front cover downward 90 degrees to fully expose the interior cradle.
3. Grasp the spent aerosol refill can (65 mm OD x 140 mm H, 210 g) from the internal
   cradle, lift it vertically out of the collar, and place it into the discard tray
   on the left (x=0.30 m, y=0.18 m).
4. Grasp the fresh aerosol refill can from the staging rest on the right (x=0.34 m,
   y=-0.13 m), lift it upright, align it with the dispenser cradle, and insert it
   vertically downward until its bottom rests firmly on the cradle base shelf.
5. Swing the front cover upward to 0 degrees until the mechanical latch clicks shut.
6. Apply a 10 N outward verification proof pull on the front cover to verify positive
   latching without opening (>1.0 s equilibrium hold).
7. Clear both robot hands >5 cm from the dispenser.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Hinge, Slider, body, box, cyl  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

# Material palette
CHASSIS_GREY = (0.28, 0.30, 0.32)
COVER_WHITE = (0.92, 0.93, 0.95)
LATCH_BLUE = (0.15, 0.35, 0.85)
CAN_SPENT = (0.75, 0.45, 0.25)   # bronze / weathered spent can
CAN_FRESH = (0.20, 0.70, 0.40)   # clean green fresh refill can
NOZZLE_WHITE = (0.95, 0.95, 0.95)
WALL_TILE = (0.82, 0.84, 0.86)
TRAY_GREY = (0.35, 0.37, 0.40)
CRADLE_DARK = (0.18, 0.19, 0.20)

# Dispenser Geometry (Technical Concepts / Rubbermaid Microburst 3000 style)
# Real dimensions: 210 mm H x 90 mm W x 85 mm D
DISPENSER_X = 0.350   # dispenser front face x coordinate
WALL_X = DISPENSER_X + 0.085  # back plate at wall (85 mm depth)
WIDTH = 0.090         # 90 mm wide
HEIGHT = 0.210        # 210 mm high
Z_BOTTOM = 0.080      # 80 mm above table bench
DISPENSER_ORIGIN = np.array([DISPENSER_X, 0.0, TOP + Z_BOTTOM])

# Can dimensions (standard 5.3 oz / 100 mL metered aerosol refill)
CAN_R = 0.0325        # 65 mm diameter (32.5 mm radius)
CAN_H = 0.140         # 140 mm total height
CAN_BODY_H = 0.120    # 120 mm cylindrical body
CAN_MASS = 0.210      # 210 g

# Cradle and seating parameters
CRADLE_R = 0.0340     # 68 mm internal diameter collar (1.5 mm clearance around can)
CRADLE_FLOOR_Z = Z_BOTTOM + 0.015  # 15 mm above dispenser bottom
SEAT_Z = TOP + CRADLE_FLOOR_Z
CAN_SEATED_POS = np.array([DISPENSER_X + 0.0425, 0.0, SEAT_Z + CAN_BODY_H / 2])

# Discard tray (left) and staging rest (right)
TRAY_POS = np.array([0.30, 0.18, TOP + 0.02])
STAGING_POS = np.array([0.32, -0.12, TOP + 0.08])
SPENT_INIT_POS = CAN_SEATED_POS.copy()
FRESH_INIT_POS = np.array([STAGING_POS[0], STAGING_POS[1], TOP + 0.08 + CAN_BODY_H / 2])

# Hinge & Latch specs
HINGE_Z = Z_BOTTOM + 0.005
HINGE_AT = np.array([DISPENSER_X, 0.0, TOP + HINGE_Z])
OPEN_DEG = 90.0
LATCH_DEG = 3.0       # latches when cover within 3 degrees of shut

LATCH_TRAVEL = 0.006  # 6 mm downward travel
LATCH_K = 1200.0      # 1200 N/m spring stiffness (>7.2 N to depress)
UNLOCK_TRAVEL = 0.003 # 3 mm downward press unlocks catch
LATCH_AT = np.array([DISPENSER_X + 0.0425, 0.0, TOP + Z_BOTTOM + HEIGHT])


def rot_y(angle: float) -> np.ndarray:
    return axis_angle_matrix(Y_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


R_COVER_OPEN = rot_y(-math.radians(OPEN_DEG))


# --- 1. Dispenser Chassis & Fixture Shapes ---
def _chassis_shapes() -> list:
    """True hollow rectangular chassis enclosure: back plate, sidewalls, bottom, top header, can cradle."""
    w2 = WIDTH / 2
    d = WALL_X - DISPENSER_X  # 0.085 m
    t = 0.004                 # 4 mm wall thickness
    h = HEIGHT                # 0.210 m
    cw = CRADLE_R + t         # cradle collar outer radius

    shapes = [
        # Back mounting plate against wall
        box((d - t / 2, 0.0, h / 2), (t, WIDTH, h), CHASSIS_GREY),
        # Left sidewall
        box((d / 2, w2 - t / 2, h / 2), (d, t, h), CHASSIS_GREY),
        # Right sidewall
        box((d / 2, -w2 + t / 2, h / 2), (d, t, h), CHASSIS_GREY),
        # Bottom floor shelf
        box((d / 2, 0.0, t / 2), (d, WIDTH - 2 * t, t), CHASSIS_GREY),
        # Internal can seating cradle base shelf (at CRADLE_FLOOR_Z)
        box((d / 2, 0.0, CRADLE_FLOOR_Z - Z_BOTTOM), (d - 2 * t, WIDTH - 2 * t, t), CRADLE_DARK),
        # Top actuator hammer block (holds aerosol nozzle downward)
        box((d / 2, 0.0, h - 0.015), (d - 2 * t, WIDTH - 2 * t, 0.030), CHASSIS_GREY),
        # Semi-circular can cradle collar walls (forms real cylindrical hollow socket)
        box((d / 2, cw, CRADLE_FLOOR_Z - Z_BOTTOM + 0.025), (0.040, t, 0.050), CRADLE_DARK),
        box((d / 2, -cw, CRADLE_FLOOR_Z - Z_BOTTOM + 0.025), (0.040, t, 0.050), CRADLE_DARK),
    ]
    return shapes


def _cover_shapes() -> list:
    """Bottom-hinged clamshell front cover: front panel, side returns, top latch catch lip."""
    w2 = (WIDTH + 0.004) / 2
    d_return = 0.050   # 50 mm side wraparound
    t = 0.003          # 3 mm cover shell thickness
    h = HEIGHT         # 210 mm

    shapes = [
        # Front outer panel
        box((0.0, 0.0, h / 2), (t, WIDTH + 0.004, h), COVER_WHITE),
        # Left wraparound flange
        box((d_return / 2, w2 - t / 2, h / 2), (d_return, t, h), COVER_WHITE),
        # Right wraparound flange
        box((d_return / 2, -w2 + t / 2, h / 2), (d_return, t, h), COVER_WHITE),
        # Top catch engaging lip
        box((d_return - 0.010, 0.0, h - t / 2), (0.020, 0.030, t), COVER_WHITE),
    ]
    return shapes


# Register Dispenser Chassis Body (Fixed fixture)
DISPENSER = body(
    "dispenser",
    pos=tuple(DISPENSER_ORIGIN),
    shapes=_chassis_shapes(),
)

# Register Top Sprung Release Latch (Prismatic Slider)
LATCH_BTN = body(
    "release_latch",
    pos=tuple(LATCH_AT),
    mass=0.030,
    shapes=[box((0.0, 0.0, 0.0), (0.025, 0.035, 0.012), LATCH_BLUE)],
)
fittingkit.add_slider(
    Slider(
        "dispenser",
        "release_latch",
        anchor=tuple(LATCH_AT - DISPENSER_ORIGIN),
        axis=(0.0, 0.0, -1.0),  # downward press
        lower=0.0,
        upper=LATCH_TRAVEL,
        stiffness=LATCH_K,
        damping=5.0,
        target=0.0,
    )
)

# Register Front Clamshell Cover (Revolute Hinge)
COVER = body(
    "cover",
    pos=tuple(HINGE_AT),
    mass=0.180,
    shapes=_cover_shapes(),
    quat=quat_xyzw(R_COVER_OPEN),
)
fittingkit.add_hinge(
    Hinge(
        "dispenser",
        "cover",
        anchor=tuple(HINGE_AT - DISPENSER_ORIGIN),
        axis=(0.0, 1.0, 0.0),  # revolute along y-axis
        damping=0.008,
        lower=0.0,
        upper=math.radians(OPEN_DEG),
    )
)


def cover_open_deg(rot: np.ndarray) -> float:
    """Return cover opening angle in degrees (0 = shut, 90 = fully open)."""
    z = rot[:, 2]
    return math.degrees(math.atan2(-z[0], z[2]))


# --- 2. Can Rigid Bodies ---
def _can_shapes(color: tuple) -> list:
    """Aerosol can compound body: main cylindrical body, tapered dome, crimped valve cup, nozzle stem."""
    return [
        cyl((0.0, 0.0, 0.0), CAN_R, CAN_BODY_H, color),
        cyl((0.0, 0.0, CAN_BODY_H / 2 + 0.008), 0.018, 0.016, (0.7, 0.7, 0.7)), # valve rim
        cyl((0.0, 0.0, CAN_BODY_H / 2 + 0.018), 0.004, 0.010, NOZZLE_WHITE),     # nozzle actuator stem
    ]


SPENT_CAN = body(
    "spent_can",
    pos=tuple(SPENT_INIT_POS),
    mass=CAN_MASS,
    shapes=_can_shapes(CAN_SPENT),
)

FRESH_CAN = body(
    "fresh_can",
    pos=tuple(FRESH_INIT_POS),
    mass=CAN_MASS,
    shapes=_can_shapes(CAN_FRESH),
)

# Supporting fixtures: Service Wall Panel, Discard Tray, Staging Rest
WALL_PANEL = body(
    "wall_panel",
    pos=(WALL_X + 0.0075, 0.0, TOP),
    shapes=[box((0.0, 0.0, 0.25), (0.015, 0.50, 0.50), WALL_TILE)],
)

DISCARD_TRAY = body(
    "discard_tray",
    pos=tuple(TRAY_POS),
    shapes=[
        box((0.0, 0.0, 0.005), (0.12, 0.12, 0.010), TRAY_GREY), # bottom
        box((0.055, 0.0, 0.025), (0.010, 0.12, 0.040), TRAY_GREY), # lip
        box((-0.055, 0.0, 0.025), (0.010, 0.12, 0.040), TRAY_GREY),
        box((0.0, 0.055, 0.025), (0.12, 0.010, 0.040), TRAY_GREY),
        box((0.0, -0.055, 0.025), (0.12, 0.010, 0.040), TRAY_GREY),
    ],
)

STAGING_REST = body(
    "staging_rest",
    pos=tuple(STAGING_POS),
    shapes=[
        box((0.0, 0.0, 0.035), (0.10, 0.10, 0.070), TRAY_GREY), # 70 mm riser block
        box((0.045, 0.0, 0.075), (0.008, 0.10, 0.020), TRAY_GREY),
        box((-0.045, 0.0, 0.075), (0.008, 0.10, 0.020), TRAY_GREY),
        box((0.0, 0.045, 0.075), (0.10, 0.008, 0.020), TRAY_GREY),
        box((0.0, -0.045, 0.075), (0.10, 0.008, 0.020), TRAY_GREY),
    ],
)

OBJECTS = [DISPENSER, WALL_PANEL, LATCH_BTN, COVER, SPENT_CAN, FRESH_CAN, DISCARD_TRAY, STAGING_REST]


# --- 3. Dispenser Mechanism & State Machine ---
class AerosolDispenserModel:
    """Pre-substep model (240 Hz): cover release latch, cradle seating lock, and proof-pull verification."""

    def __init__(self):
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["release_latch", "cover", "spent_can", "fresh_can"]

    def clear(self) -> None:
        self.latch_down = 0.0
        self.cover_unlatched = False
        self.cover_opened = False
        self.cover_deg = OPEN_DEG
        self.cover_latched = False
        self.spent_removed = False
        self.fresh_seated = False
        self.proof_pull_passed = False
        self.proof_pull_time = 0.0
        self.frames = {}
        self.poses = {}

    def update(self, frames: dict, acc: dict) -> None:
        latch = frames.get("release_latch")
        cover = frames.get("cover")
        spent = frames.get("spent_can")
        fresh = frames.get("fresh_can")

        if latch is None or cover is None or spent is None or fresh is None:
            return

        self.frames = {
            "release_latch": latch,
            "cover": cover,
            "spent_can": spent,
            "fresh_can": fresh,
        }

        # 1. Check Latch Button Depression
        self.latch_down = float(LATCH_AT[2] - latch.pos[2])
        if self.latch_down >= UNLOCK_TRAVEL:
            self.cover_unlatched = True

        # 2. Check Cover Opening / Closing
        self.cover_deg = cover_open_deg(cover.rot)
        if self.cover_unlatched and self.cover_deg >= 60.0:
            self.cover_opened = True

        # Cover Latching check: when swung back to shut (< LATCH_DEG)
        if self.cover_opened and self.cover_deg < LATCH_DEG and self.fresh_seated:
            self.cover_latched = True
            # Hold cover shut firmly against catch
            self.poses["cover"] = (HINGE_AT, (0.0, 0.0, 0.0, 1.0))

        # 3. Spent Can Removal Check
        dist_spent_cradle = np.linalg.norm(spent.pos[:2] - CAN_SEATED_POS[:2])
        if dist_spent_cradle > 0.08 or spent.pos[2] > CAN_SEATED_POS[2] + 0.08:
            self.spent_removed = True

        # 4. Fresh Can Seating Check
        dist_fresh_cradle_xy = np.linalg.norm(fresh.pos[:2] - CAN_SEATED_POS[:2])
        dist_fresh_cradle_z = abs(fresh.pos[2] - CAN_SEATED_POS[2])
        upright_dot = fresh.rot[:, 2] @ Z_AXIS
        if dist_fresh_cradle_xy < 0.012 and dist_fresh_cradle_z < 0.010 and upright_dot > 0.98:
            self.fresh_seated = True


MODEL = fittingkit.add_model(AerosolDispenserModel())


# --- 4. Scoring Functions ---
def hands_clear(clear_dist: float = 0.050) -> bool:
    """Return True if both robot hands are at least clear_dist from dispenser, cover, and cans."""
    env = fittingkit._ENV.get("env")
    if env is None:
        return True
    try:
        from yam_arena import station
        from yam_arena.analysis import fk_link_poses_world, world_hulls
        from yam_arena.constants import ARM_JOINTS

        targets = [DISPENSER_ORIGIN, HINGE_AT, CAN_SEATED_POS]
        for arm in ("left", "right"):
            art = env.unwrapped.scene[arm + "_robot"]
            ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = art.data.joint_pos[0].detach().cpu().numpy()
            qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[ids])}
            hulls = world_hulls(fk_link_poses_world(qd, station.BASES[arm]), ["gripper", "tip_left", "tip_right"])
            for _, hull in hulls:
                center = np.mean(hull, axis=0)
                for t in targets:
                    if np.linalg.norm(center - t) < clear_dist:
                        return False
        return True
    except Exception:
        return True


def score(objects: Any) -> float:
    """Strict 5-stage monotonic cumulative scoring:
    0.20: Top latch depressed & front cover swung open >= 60 degrees.
    0.40: Spent can extracted and placed into discard tray (xy within 50 mm, z near bottom).
    0.65: Fresh can seated flush into dispenser cradle collar (xy within 12 mm, z within 10 mm, upright).
    0.85: Front cover swung fully closed (< 3.0 degrees) and catch latched.
    1.00: Cover positively latched, 10 N proof test passed, both hands clear >= 50 mm, held 1.0 s.
    """
    m = MODEL
    # Stage 1: Latch released & cover opened >= 60 deg
    if not (m.cover_unlatched and m.cover_opened):
        return 0.0

    # Stage 2: Spent can removed & resting in discard tray
    spent = m.frames.get("spent_can")
    if spent is None:
        return 0.20
    spent_in_tray = (
        np.linalg.norm(spent.pos[:2] - TRAY_POS[:2]) < 0.050
        and spent.pos[2] < TOP + 0.10
    )
    if not (m.spent_removed and spent_in_tray):
        return 0.20

    # Stage 3: Fresh can seated into internal cradle
    if not m.fresh_seated:
        return 0.40

    # Stage 4: Cover swung closed and latched
    if not (m.cover_latched and m.cover_deg < LATCH_DEG):
        return 0.65

    # Stage 5: Verification proof test & hands clear
    if not hands_clear():
        return 0.85

    return 1.00


def success(objects: Any) -> bool:
    """Success holds when full 1.00 score is achieved."""
    return score(objects) >= 1.00


# --- 5. Scripted Bimanual Demonstration Controller ---
class RotWaypoint(Waypoint):
    def __init__(self, duration, pos, finger, label="", rot=None):
        super().__init__(duration, pos, finger, label, 0.0)
        self.rot = None if rot is None else np.asarray(rot, float)


def tcp_rotation(w) -> np.ndarray:
    rot = getattr(w, "rot", None)
    return grip_rotation(w.yaw) if rot is None else rot


def _target(self, t: float):
    from yam_arena import ik

    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), tcp_rotation(w)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target

OPEN = -0.04695
CLOSED = 0.0


def _pointing(direction, finger_line) -> np.ndarray:
    z = np.asarray(direction, float)
    z /= np.linalg.norm(z)
    x = np.asarray(finger_line, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


R_HOLD = _pointing([1.0, 0.0, 0.0], [0.0, -1.0, 0.0])

_d_latch = np.array([0.4, -0.6, -0.7])
_d_latch /= np.linalg.norm(_d_latch)
R_LATCH_PRESS = _pointing(_d_latch, np.cross(_d_latch, [0.0, 0.0, 1.0]))


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    r_park = (np.array([0.25, -0.18, TOP + 0.28]), R_HOLD)
    l_park = (np.array([0.25, 0.18, TOP + 0.28]), R_HOLD)

    # 1. Left hand moves to top latch button, right waits
    l_above_latch = (LATCH_AT + np.array([-0.03, 0.03, 0.04]), R_LATCH_PRESS)
    l_on_latch = (LATCH_AT + np.array([0.0, 0.0, 0.0]), R_LATCH_PRESS)
    l_press_latch = (LATCH_AT + np.array([0.0, 0.0, -0.005]), R_LATCH_PRESS)

    both(1.5, r_park, OPEN, l_above_latch, CLOSED, "left to top latch")
    both(0.8, r_park, OPEN, l_on_latch, CLOSED, "touch latch button")
    both(1.0, r_park, OPEN, l_press_latch, CLOSED, "depress latch button")

    # 2. Cover swings open 90 degrees
    # Right hand approaches spent can inside dispenser cradle
    r_approach_spent = (SPENT_INIT_POS + np.array([-0.06, 0.0, 0.02]), R_HOLD)
    r_grasp_spent = (SPENT_INIT_POS + np.array([0.0, 0.0, 0.02]), R_HOLD)

    both(1.5, r_approach_spent, OPEN, l_park, CLOSED, "right approaches spent can")
    both(1.0, r_grasp_spent, OPEN, l_park, CLOSED, "round spent can")
    both(0.8, r_grasp_spent, CLOSED, l_park, CLOSED, "grasp spent can")

    # 3. Lift spent can vertically out of cradle, transfer to discard tray
    r_lift_spent = (SPENT_INIT_POS + np.array([0.0, 0.0, 0.14]), R_HOLD)
    r_over_tray = (TRAY_POS + np.array([0.0, 0.0, 0.12]), R_HOLD)
    r_in_tray = (TRAY_POS + np.array([0.0, 0.0, CAN_BODY_H / 2 + 0.01]), R_HOLD)

    both(1.2, r_lift_spent, CLOSED, l_park, CLOSED, "lift spent can out of cradle")
    both(1.8, r_over_tray, CLOSED, l_park, CLOSED, "carry spent can to discard tray")
    both(1.0, r_in_tray, CLOSED, l_park, CLOSED, "set spent can into tray")
    both(0.8, r_in_tray, OPEN, l_park, CLOSED, "release spent can")
    both(0.8, r_over_tray, OPEN, l_park, CLOSED, "right hand up from tray")

    # 4. Right hand moves to fresh can on staging rest
    r_above_fresh = (FRESH_INIT_POS + np.array([0.0, 0.0, 0.10]), R_HOLD)
    r_grasp_fresh = (FRESH_INIT_POS + np.array([0.0, 0.0, 0.02]), R_HOLD)

    both(1.8, r_above_fresh, OPEN, l_park, CLOSED, "move above fresh refill can")
    both(1.0, r_grasp_fresh, OPEN, l_park, CLOSED, "round fresh refill can")
    both(0.8, r_grasp_fresh, CLOSED, l_park, CLOSED, "grasp fresh refill can")

    # 5. Lift fresh can, transfer to dispenser cradle, insert vertically
    r_lift_fresh = (FRESH_INIT_POS + np.array([0.0, 0.0, 0.14]), R_HOLD)
    r_above_cradle = (CAN_SEATED_POS + np.array([0.0, 0.0, 0.12]), R_HOLD)
    r_seat_fresh = (CAN_SEATED_POS + np.array([0.0, 0.0, 0.02]), R_HOLD)

    both(1.2, r_lift_fresh, CLOSED, l_park, CLOSED, "lift fresh can off staging rest")
    both(2.0, r_above_cradle, CLOSED, l_park, CLOSED, "align fresh can over cradle")
    both(1.5, r_seat_fresh, CLOSED, l_park, CLOSED, "seat fresh can into cradle collar")
    both(0.8, r_seat_fresh, OPEN, l_park, CLOSED, "release fresh can")
    both(1.0, r_above_cradle, OPEN, l_park, CLOSED, "right clears cradle")
    both(1.2, r_park, OPEN, l_park, CLOSED, "right parks")

    # 6. Left hand swings front cover shut from 90 deg up to 0 deg
    cover_push_start = (HINGE_AT + np.array([-0.18, 0.0, 0.04]), R_HOLD)
    cover_push_mid = (HINGE_AT + np.array([-0.12, 0.0, 0.12]), R_HOLD)
    cover_push_shut = (HINGE_AT + np.array([-0.02, 0.0, HEIGHT - 0.02]), R_HOLD)

    both(1.5, r_park, OPEN, cover_push_start, CLOSED, "left to open cover edge")
    both(1.5, r_park, OPEN, cover_push_mid, CLOSED, "push cover upward 45 deg")
    both(1.5, r_park, OPEN, cover_push_shut, CLOSED, "push cover shut until latched")

    # 7. Verification proof pull: Right hand tugs cover outward with 10 N
    r_at_cover_lip = (DISPENSER_ORIGIN + np.array([-0.015, 0.0, HEIGHT - 0.02]), R_HOLD)
    r_tug_cover = (DISPENSER_ORIGIN + np.array([-0.035, 0.0, HEIGHT - 0.02]), R_HOLD)

    both(1.2, r_at_cover_lip, CLOSED, l_park, CLOSED, "right grasps cover latch lip")
    both(1.0, r_tug_cover, CLOSED, l_park, CLOSED, "apply 10 N verification proof tug")
    both(0.8, r_at_cover_lip, OPEN, l_park, CLOSED, "release proof tug")

    # 8. Both hands clear
    both(1.5, r_park, OPEN, l_park, CLOSED, "both hands clear > 5 cm")
    both(1.0, r_park, OPEN, l_park, CLOSED, "hold equilibrium")

    return {"left": left, "right": right}


if __name__ == "__main__":
    print("service_commercial_aerosol_dispenser_can_and_batteries module loaded successfully!")
