"""Submerge an ice cooling paddle into a commercial stockpot of hot soup, agitate to chill, and hang the paddle on the rim bracket.

On the commercial kitchen prep counter stands an 8 qt heavy-gauge stainless steel stockpot (Vollrath / Winco style:
220 mm outer diameter, 204 mm inner diameter, 80 mm tall, 2.4 kg) sitting on the counter at (0.260, 0.080).
The stockpot contains hot broth simulant (liquid depth 40 mm, surface at Z = 0.7935 m, 34 mm headspace to the rim).
The front-left rim of the pot features a welded stainless paddle hanging bracket with a horizontal resting saddle
at (0.240, 0.170, Z = 0.8275 m). The right side of the pot features a heavy welded tubular loop handle at (0.260, -0.045, Z = 0.7925 m).
To the robot's left stands a sanitary stainless paddle stand (80 x 80 x 30 mm, 400 g) at (0.200, 0.260) holding an upright
commercial rapid cooling paddle (San Jamar Rapi-Kool style: 120 g hollow HDPE shell, 65 mm finned cooling blade, 18 mm collar with
radial hanging hook extending 25 mm, and 45 mm cylindrical grip handle).

Restaurant cooks (1.44M US jobs, O*NET 35-2014.00) and food preparation workers (891k, O*NET 35-2021.00) rapidly chill
large stockpots of cooked soups, stocks, and chili under FDA Food Code 3-501.14 (cooling from 135 deg F to 70 deg F within
2 hours and to 41 deg F within 4 hours to prevent spore germination of Clostridium perfringens and Bacillus cereus).
Leaving hot stockpots to cool passively in ambient air violates food safety limits. Cooks use hollow polyethylene ice
cooling wands filled with frozen core water. The cook retrieves the frozen paddle from its stand, submerges the
finned blade deep into the hot broth, agitates with circular stirring strokes to circulate liquid around the cold fins
without splashing scalding broth over the rim, raises the paddle to allow broth to drain back into the pot, and hooks
the integrated collar hook onto the pot rim bracket so the paddle stays suspended.

Mechanism parts come from mechkit.py (in this folder), which adds procedural collision structures and tracking in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rapid-cool-soup-ice-paddle/rapid_cool_soup_ice_paddle.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 --max-steps 1600 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
STEP_S = 0.05  # 20 Hz control
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Colors (sRGB in [0, 1])
STAINLESS = (0.75, 0.77, 0.80)
DARK_STAINLESS = (0.45, 0.47, 0.50)
STAND_STEEL = (0.62, 0.64, 0.67)
BROTH_GOLD = (0.76, 0.58, 0.22)
ICE_PADDLE_CYAN = (0.35, 0.75, 0.85)
PADDLE_HANDLE_BLUE = (0.15, 0.35, 0.70)
HOOK_RED = (0.85, 0.22, 0.20)

# Physical layout and dimensions
# Commercial stockpot (Vollrath / Winco style 8 qt heavy stainless pot)
POT_XY = (0.260, 0.080)
POT_OUTER_ACROSS = 0.220          # 220 mm flat-to-flat outer diameter
POT_INNER_ACROSS = 0.204          # 204 mm inner diameter (8 mm wall thickness)
POT_WALL_THICK = 0.008
POT_TOTAL_H = 0.080               # 80 mm exterior pot height
POT_FLOOR_H = 0.006               # 6 mm thick bottom slab
POT_FLOOR_Z = TOP + POT_FLOOR_H   # Z = 0.7535 m
POT_RIM_Z = TOP + POT_TOTAL_H     # Z = 0.8275 m
POT_KG = 2.40                     # 2.4 kg pot + soup mass

# Hot broth simulant inside pot
BROTH_DEPTH = 0.040               # 40 mm broth depth
BROTH_SURFACE_Z = POT_FLOOR_Z + BROTH_DEPTH  # Z = 0.7935 m
BROTH_HEADSPACE = POT_RIM_Z - BROTH_SURFACE_Z  # 34.0 mm clear headspace to rim

# Stockpot rim hanging bracket (welded at front-left rim: X=0.240, Y=0.170)
BRACKET_XY = (0.240, 0.170)
BRACKET_SEAT_Z = POT_RIM_Z        # Z = 0.8275 m
BRACKET_SIZE = (0.028, 0.028, 0.012)

# Stockpot right loop handle (welded at right side: X=0.260, Y=-0.045)
POT_HANDLE_XY = (0.260, -0.045)
POT_HANDLE_Z = TOP + 0.045        # Z = 0.7925 m

# Paddle sanitary stand (weighted stainless cradle with vertical socket)
STAND_XY = (0.200, 0.260)
STAND_SIZE = (0.080, 0.080, 0.030)
STAND_TOP = TOP + STAND_SIZE[2]   # Z = 0.7775 m
STAND_SOCKET_DEPTH = 0.018
STAND_FLOOR_Z = STAND_TOP - STAND_SOCKET_DEPTH  # Z = 0.7595 m
STAND_KG = 0.40

# Ice cooling paddle (San Jamar Rapi-Kool style wand)
# Frame origin at collar/hook center: blade hangs downwards (-z), handle extends upwards (+z)
PADDLE_BLADE_W = 0.055            # 55 mm wide along Y
PADDLE_BLADE_T = 0.026            # 26 mm thick along X
PADDLE_BLADE_L = 0.065            # 65 mm long along -Z (bottom tip at z = -0.065 m)
PADDLE_COLLAR_H = 0.018           # collar from z = 0.0 to +0.018 m
PADDLE_HOOK_PROJ = 0.025          # hook extends 25 mm radially along +Y
PADDLE_HOOK_LIP = 0.010           # downward retaining lip
PADDLE_HANDLE_DIA = 0.024         # 24 mm cylindrical grip handle
PADDLE_HANDLE_L = 0.045           # handle from z = +0.018 to +0.063 m
PADDLE_GRIP_Z_LOCAL = 0.040       # ergonomic grip centre in paddle frame (+40 mm above collar)
PADDLE_KG = 0.120                 # simulated 120 g (hollow HDPE cooling wand shell)

# Initial placement of paddle in stand:
# Paddle rests blade-down in stand socket with 1 mm gravity settling clearance.
# Blade bottom at STAND_FLOOR_Z + 0.001 m.
# Collar origin z = STAND_FLOOR_Z + PADDLE_BLADE_L + 0.001.
PADDLE_START_POS = (STAND_XY[0], STAND_XY[1], STAND_FLOOR_Z + PADDLE_BLADE_L + 0.001)

# Local hook engagement point in paddle frame
HOOK_LOCAL = (0.0, 0.015, 0.004)

# Tolerances and scoring thresholds
MIN_LIFT_CLEAR_Z = 0.025          # lifted >= 25 mm out of cradle
MIN_SUBMERGE_DEPTH = 0.015        # blade submerged >= 15 mm below liquid surface
AGITATION_MIN_REVS = 3.0          # >= 3 circular revolutions for partial credit (0.60)
AGITATION_FULL_REVS = 5.0         # >= 5 circular revolutions for full credit (0.80)
MIN_STIR_R = 0.005                # minimum radius from pot axis
MAX_STIR_R = 0.085                # maximum radius from pot axis (clears 102 mm inner radius)
MAX_SPLASH_SPEED = 0.45           # linear speed > 0.45 m/s while submerged causes splash fault
MAX_SPLASH_ACCEL = 6.0            # acceleration > 6.0 m/s^2 while submerged causes splash fault
DRAIN_HOLD_STEPS = 16             # 16 steps (0.8 s) drain hold above liquid level
HOOK_SEAT_TOL_XY = 0.035          # hook within 35 mm of bracket saddle
HOOK_SEAT_TOL_Z = 0.030           # hook within 30 mm of bracket saddle
HOOK_TILT_MAX_DEG = 25.0          # paddle hanging upright within 25 deg
POT_MAX_SLIP = 0.030              # pot slid > 30 mm faults
POT_MAX_TILT_DEG = 10.0           # pot tilted > 10 deg faults
RELEASE_TCP_CLEAR = 0.030         # TCP >= 30 mm clear
RELEASE_HULL_CLEAR = 0.005        # collision hulls >= 5 mm clear

_T22 = math.tan(math.radians(22.5))


def _oct_solid(across: float, z0: float, h: float, color=None, xy=(0.0, 0.0)) -> list[mk.BoxSpec]:
    """Regular octagon ``across`` flats, from ``z0`` to ``z0 + h``."""
    return [
        mk.box((across, across * _T22, h), at=(xy[0], xy[1], z0 + h / 2), color=color, yaw=k * math.pi / 4)
        for k in range(4)
    ]


def _oct_ring(across_outer: float, wall: float, z0: float, h: float, color=None, xy=(0.0, 0.0)) -> list[mk.BoxSpec]:
    """Hollow 8-sided ring with outer flat-to-flat width ``across_outer`` and wall thickness ``wall``."""
    r_mid = across_outer / 2 - wall / 2
    side = across_outer * _T22 + 0.0010
    out = []
    for k in range(8):
        th = k * math.pi / 4
        out.append(
            mk.box(
                (wall, side, h),
                at=(xy[0] + r_mid * math.cos(th), xy[1] + r_mid * math.sin(th), z0 + h / 2),
                color=color,
                yaw=th,
            )
        )
    return out


def _pot_boxes() -> list[mk.BoxSpec]:
    """Vollrath / Winco style 8 qt heavy stainless steel stockpot with base, walls, rim, bracket, and handle."""
    wall_h = POT_TOTAL_H - POT_FLOOR_H
    rim_bead_h = 0.006
    
    # Bracket relative to pot centre:
    b_dx = BRACKET_XY[0] - POT_XY[0]
    b_dy = BRACKET_XY[1] - POT_XY[1]
    
    # Handle relative to pot centre:
    h_dx = POT_HANDLE_XY[0] - POT_XY[0]
    h_dy = POT_HANDLE_XY[1] - POT_XY[1]

    return [
        # Solid base floor
        *_oct_solid(POT_OUTER_ACROSS, 0.0, POT_FLOOR_H, color=STAINLESS),
        # 8-sided deep pot wall
        *_oct_ring(POT_OUTER_ACROSS, POT_WALL_THICK, POT_FLOOR_H, wall_h, color=STAINLESS),
        # Rolled top rim lip
        *_oct_ring(POT_OUTER_ACROSS + 0.008, 0.008, POT_TOTAL_H - rim_bead_h, rim_bead_h, color=DARK_STAINLESS),
        # Welded front-left paddle hanging bracket
        mk.box(BRACKET_SIZE, at=(b_dx, b_dy, POT_TOTAL_H - BRACKET_SIZE[2] / 2), color=STAINLESS),
        mk.box((0.008, 0.018, 0.005), at=(b_dx - 0.006, b_dy, POT_TOTAL_H + 0.0025), color=DARK_STAINLESS),
        # Welded right tubular loop handle
        mk.box((0.020, 0.012, 0.040), at=(h_dx, h_dy, 0.045), color=DARK_STAINLESS),
    ]


def _stand_boxes() -> list[mk.BoxSpec]:
    """Sanitary stainless paddle stand with square base and vertical hollow receiving socket."""
    base_w, base_d, base_h = STAND_SIZE
    wall_t = 0.008
    return [
        # Solid weighted base bottom
        mk.box((base_w, base_d, base_h - STAND_SOCKET_DEPTH), at=(0.0, 0.0, (base_h - STAND_SOCKET_DEPTH) / 2), color=STAND_STEEL),
        # 4 socket side walls
        mk.box((wall_t, base_d, STAND_SOCKET_DEPTH), at=(-base_w / 2 + wall_t / 2, 0.0, base_h - STAND_SOCKET_DEPTH / 2), color=STAND_STEEL),
        mk.box((wall_t, base_d, STAND_SOCKET_DEPTH), at=(base_w / 2 - wall_t / 2, 0.0, base_h - STAND_SOCKET_DEPTH / 2), color=STAND_STEEL),
        mk.box((base_w - 2 * wall_t, wall_t, STAND_SOCKET_DEPTH), at=(0.0, -base_d / 2 + wall_t / 2, base_h - STAND_SOCKET_DEPTH / 2), color=STAND_STEEL),
        mk.box((base_w - 2 * wall_t, wall_t, STAND_SOCKET_DEPTH), at=(0.0, base_d / 2 - wall_t / 2, base_h - STAND_SOCKET_DEPTH / 2), color=STAND_STEEL),
    ]


def _paddle_boxes() -> list[mk.BoxSpec]:
    """San Jamar Rapi-Kool cooling wand: finned blade (-z), collar + hook (z~0), handle (+z)."""
    return [
        # Main cooling blade (downwards from collar, z in [-0.065, 0.0])
        mk.box((PADDLE_BLADE_T, PADDLE_BLADE_W, PADDLE_BLADE_L), at=(0.0, 0.0, -PADDLE_BLADE_L / 2), color=ICE_PADDLE_CYAN),
        # Cooling ribs/fins along blade faces (increases heat dissipation in commercial use)
        mk.box((PADDLE_BLADE_T + 0.006, PADDLE_BLADE_W - 0.010, 0.008), at=(0.0, 0.0, -0.020), color=ICE_PADDLE_CYAN),
        mk.box((PADDLE_BLADE_T + 0.006, PADDLE_BLADE_W - 0.010, 0.008), at=(0.0, 0.0, -0.045), color=ICE_PADDLE_CYAN),
        # Collar between blade and handle (z in [0.0, 0.018])
        mk.box((0.030, 0.030, PADDLE_COLLAR_H), at=(0.0, 0.0, PADDLE_COLLAR_H / 2), color=PADDLE_HANDLE_BLUE),
        # Radial hanging hook projection (+Y)
        mk.box((0.018, PADDLE_HOOK_PROJ, 0.010), at=(0.0, 0.015 + PADDLE_HOOK_PROJ / 2, 0.012), color=HOOK_RED),
        # Downward retaining lip of hook
        mk.box((0.018, 0.006, PADDLE_HOOK_LIP), at=(0.0, 0.015 + PADDLE_HOOK_PROJ - 0.003, 0.012 - PADDLE_HOOK_LIP / 2), color=HOOK_RED),
        # Cylindrical grip handle (z in [0.018, 0.063])
        mk.box((PADDLE_HANDLE_DIA, PADDLE_HANDLE_DIA, PADDLE_HANDLE_L), at=(0.0, 0.0, PADDLE_COLLAR_H + PADDLE_HANDLE_L / 2), color=PADDLE_HANDLE_BLUE),
    ]


kit = mk.Kit()

# Fixed sanitary stand on counter
kit.part("storage_stand", (STAND_XY[0], STAND_XY[1], TOP), _stand_boxes(), mass=None, color=STAND_STEEL)

# 8 qt stockpot on counter (free rigid body, sits solidly with friction, 1.5 mm initial clearance)
kit.part("stockpot", (POT_XY[0], POT_XY[1], TOP + 0.0015), _pot_boxes(), mass=POT_KG, color=STAINLESS, solver_position_iterations=16)

# Commercial rapid cooling ice paddle (starts upright in storage stand)
kit.part("ice_paddle", PADDLE_START_POS, _paddle_boxes(), mass=PADDLE_KG, color=ICE_PADDLE_CYAN, solver_position_iterations=16)

FIXTURES = []

OBJECTS = kit.objects()


# -- episode physics & scoring ----------------------------------------------------------------------------------

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


def _all_finite(objects) -> bool:
    for name in ("stockpot", "ice_paddle"):
        st = objects.get(name)
        if st is None:
            return False
        if not (
            np.all(np.isfinite(st.pos))
            and np.all(np.isfinite(st.quat_xyzw))
            and np.all(np.isfinite(st.lin_vel))
            and np.all(np.isfinite(st.ang_vel))
        ):
            return False
    return True


def _tilt_deg(st) -> float:
    q = np.asarray(st.quat_xyzw, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-6:
        return 0.0
    x, y, z, w = q / norm
    cos_th = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_th))))


def paddle_hook_pos(objects) -> np.ndarray:
    """World position of the paddle collar hook seat."""
    return kit.point(objects, "ice_paddle", HOOK_LOCAL)


def paddle_bottom_pos(objects) -> np.ndarray:
    """World position of the paddle blade bottom tip."""
    return kit.point(objects, "ice_paddle", (0.0, 0.0, -PADDLE_BLADE_L))


def paddle_hook_on_bracket(objects) -> bool:
    """Check if paddle collar hook is seated securely on the stockpot rim bracket."""
    hook_pt = paddle_hook_pos(objects)
    bracket_pt = kit.point(objects, "stockpot", (BRACKET_XY[0] - POT_XY[0], BRACKET_XY[1] - POT_XY[1], POT_TOTAL_H))
    xy_err = math.hypot(float(hook_pt[0]) - float(bracket_pt[0]), float(hook_pt[1]) - float(bracket_pt[1]))
    dz = abs(float(hook_pt[2]) - float(bracket_pt[2]))
    tilt = _tilt_deg(objects["ice_paddle"])
    return xy_err <= HOOK_SEAT_TOL_XY and dz <= HOOK_SEAT_TOL_Z and tilt <= HOOK_TILT_MAX_DEG


def hands_clear(objects) -> bool:
    """Both TCPs >= 30 mm and all URDF collision hulls >= 5 mm clear of stockpot and paddle."""
    targets = [objects["stockpot"], objects["ice_paddle"]]
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    for st in targets:
        lo, hi = np.asarray(st.low) - RELEASE_TCP_CLEAR, np.asarray(st.high) + RELEASE_TCP_CLEAR
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps):
            return False
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES
    for lo_r, hi_r in boxes:
        for st in targets:
            gap = np.maximum(np.maximum(np.asarray(st.low) - hi_r, lo_r - np.asarray(st.high)), 0.0)
            if float(np.linalg.norm(gap)) < RELEASE_HULL_CLEAR:
                return False
    return True


def _step_rules(objects) -> None:
    """Update fluid immersion, circular agitation phase accounting, splash speed limits, and rim hanging latch."""
    step_id = mk._EP["step"]
    if mk.MEMORY.get("_last_step") == step_id and step_id != 0:
        return
    mk.MEMORY["_last_step"] = step_id

    if not _all_finite(objects):
        mk.MEMORY["invalid_state"] = True
        return

    pot_st = objects["stockpot"]
    pad_st = objects["ice_paddle"]

    # 1. Pot stability check: stockpot must not slide > 30 mm or tilt > 10 deg (after initial settling)
    pot_pos, _ = kit.pose(objects, "stockpot")
    pot_xy = (float(pot_pos[0]), float(pot_pos[1]))
    pot_slip = math.hypot(pot_xy[0] - POT_XY[0], pot_xy[1] - POT_XY[1])
    pot_tilt = _tilt_deg(pot_st)
    if step_id > 5 and (pot_slip > POT_MAX_SLIP or pot_tilt > POT_MAX_TILT_DEG):
        mk.MEMORY["pot_fault"] = True

    # 2. Dropped paddle fault: paddle falls onto counter top outside stand and pot
    pad_tip = paddle_bottom_pos(objects)
    if float(pad_tip[2]) < TOP - 0.010:
        mk.MEMORY["drop_fault"] = True

    # 3. Fluid immersion tracking:
    pad_pos, _ = kit.pose(objects, "ice_paddle")
    immersion = BROTH_SURFACE_Z - float(pad_tip[2])
    dx_pot = float(pad_pos[0]) - pot_xy[0]
    dy_pot = float(pad_pos[1]) - pot_xy[1]
    r_pot = math.hypot(dx_pot, dy_pot)
    inside_pot = r_pot <= (POT_INNER_ACROSS / 2 - 0.005)

    if immersion >= MIN_SUBMERGE_DEPTH and inside_pot:
        mk.MEMORY["submerged_once"] = True
        
        # Check splash speed / acceleration while submerged
        v_lin = float(np.linalg.norm(pad_st.lin_vel))
        a_lin = float(np.linalg.norm(pad_st.lin_vel - mk.MEMORY.get("prev_v", pad_st.lin_vel))) / STEP_S
        mk.MEMORY["prev_v"] = np.copy(pad_st.lin_vel)

        if v_lin > MAX_SPLASH_SPEED or a_lin > MAX_SPLASH_ACCEL:
            mk.MEMORY["splash_fault"] = True

        # Circular agitation phase tracking (unidirectional counter-clockwise integration)
        if MIN_STIR_R <= r_pot <= MAX_STIR_R:
            theta = math.atan2(dy_pot, dx_pot)
            if "prev_theta" in mk.MEMORY:
                prev_th = mk.MEMORY["prev_theta"]
                d_th = (theta - prev_th) % (2 * math.pi)
                if 0.002 < d_th < math.pi:
                    mk.MEMORY["cum_phase"] = mk.MEMORY.get("cum_phase", 0.0) + d_th
            mk.MEMORY["prev_theta"] = theta
        else:
            mk.MEMORY.pop("prev_theta", None)
    else:
        mk.MEMORY.pop("prev_theta", None)
        mk.MEMORY.pop("prev_v", None)

    # 4. Drainage tracking: paddle lifted above liquid level while inside pot after stirring
    revs = mk.MEMORY.get("cum_phase", 0.0) / (2 * math.pi)
    if revs >= AGITATION_FULL_REVS - 1e-4 and inside_pot and float(pad_tip[2]) >= BROTH_SURFACE_Z - 0.005:
        mk.MEMORY["drain_steps"] = mk.MEMORY.get("drain_steps", 0) + 1

    # 5. Rim bracket hang check (only triggers cheat_bypass if actually placed/released without stirring):
    on_bracket = paddle_hook_on_bracket(objects)
    if on_bracket and (mk.released(pad_st) or pad_st.resting()):
        if revs < AGITATION_MIN_REVS - 1e-4:
            mk.MEMORY["cheat_bypass"] = True
        else:
            mk.MEMORY["rim_latched"] = True

    # Periodic execution logging in sim
    if (step_id % 10 == 0 and 100 <= step_id <= 450) or step_id % 50 == 0 or step_id in (1, 10, 20, 30, 40, 50, 530, 533):
        pad_z = float(pad_tip[2])
        active_faults = [k for k in ("pot_fault", "drop_fault", "splash_fault", "cheat_bypass", "invalid_state") if mk.MEMORY.get(k)]
        th_val = math.atan2(dy_pot, dx_pot)
        print(f"[STEP {step_id:04d}] score={score(objects):.2f} pad_tip_z={pad_z:.4f} revs={revs:.2f} r_pot={r_pot:.4f} pot_xy=({pot_xy[0]:.3f},{pot_xy[1]:.3f}) pad_xy=({pad_pos[0]:.3f},{pad_pos[1]:.3f}) th={th_val:.3f} faults={active_faults}", flush=True)


kit.on_step(_step_rules)


def failed(objects) -> bool:
    return bool(
        mk.MEMORY.get("invalid_state")
        or mk.MEMORY.get("pot_fault")
        or mk.MEMORY.get("drop_fault")
        or mk.MEMORY.get("splash_fault")
        or mk.MEMORY.get("cheat_bypass")
    )


def score(objects) -> float:
    """Hardened staged score in [0, 1]:
    - 0.0 (latched) if non-finite state, scalding broth splash (> 0.45 m/s or jerky motion while submerged),
      stockpot displaced or tilted, paddle dropped below tabletop, or premature hook hang without agitation.
    - 0.15: paddle grasped and lifted vertically clear of the stand cradle (> 25 mm).
    - 0.35: paddle blade submerged into the hot broth (depth >= 20 mm).
    - 0.60: fluid agitation achieved (>= 3 circular revolutions around the pot centerline without splashing).
    - 0.80: fluid agitation completed (>= 5 revolutions) and paddle raised to drainage height above the liquid
      inside the pot for >= 16 steps (0.8 s) to drip off broth.
    - 1.00: paddle hanging hook securely seated on the rim bracket, both robot hands clear (TCPs >= 30 mm,
      link hulls >= 5 mm), and paddle resting still for >= 1.0 s.
    """
    if mk._EP["env"] is None:
        pass
    if not _all_finite(objects) or failed(objects):
        return 0.0

    pad_tip = paddle_bottom_pos(objects)
    lifted = float(pad_tip[2]) >= STAND_FLOOR_Z + MIN_LIFT_CLEAR_Z
    if not lifted and not mk.MEMORY.get("submerged_once"):
        return 0.0

    if not mk.MEMORY.get("submerged_once"):
        return 0.15

    revs = mk.MEMORY.get("cum_phase", 0.0) / (2 * math.pi)
    if revs < AGITATION_MIN_REVS - 1e-4:
        return 0.35
    if revs < AGITATION_FULL_REVS - 1e-4:
        return 0.60

    drained = mk.MEMORY.get("drain_steps", 0) >= DRAIN_HOLD_STEPS
    if not drained and not paddle_hook_on_bracket(objects):
        return 0.60

    if not paddle_hook_on_bracket(objects):
        return 0.80

    still = objects["ice_paddle"].resting() and objects["stockpot"].resting()
    if still and hands_clear(objects):
        return 1.0
    return 0.80


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


# Aliases for inspect-robots scorer binding
task_score = score
task_success = success


# -- scripted open-loop demonstration ---------------------------------------------------------------------------

HANDLE_PINCH = FINGER_CLOSED
POT_HANDLE_PINCH = FINGER_CLOSED

LEFT_PARK = (0.160, 0.280, 0.920)
RIGHT_PARK = (0.180, -0.260, 0.920)


def demo():
    """Bimanual open-loop demo (~26.6 s, 533 steps at 20 Hz):
    1. Left arm descends to paddle stand at (0.200, 0.260), grasps paddle handle at Z = 0.8655 m with yaw=0,
       lifts vertically out of the cradle, and carries it above the stockpot at (0.260, 0.080).
    2. Concurrently, right arm moves to the right stockpot loop handle at (0.260, -0.045, 0.8025) and grasps it firmly
       to steady the pot against hydrodynamic agitation forces.
    3. Left arm lowers paddle blade into the hot broth simulant (depth ~28 mm).
    4. Left arm performs 5 smooth circular stirring revolutions (R = 30 mm, ~0.45 rev/s, v ~ 0.085 m/s), smoothly
       circulating broth past the cold fins without splashing.
    5. Left arm raises paddle to drain height (blade tip Z = 0.8095 m, inside pot walls) for a 1.2 s drainage dwell.
    6. Left arm moves paddle collar hook onto the front-left rim bracket at (0.240, 0.170, 0.8425 m), seats the hook,
       opens gripper, and retreats clear to park.
    7. Right arm releases the pot handle and retreats clear to park.
    8. Both arms stay parked clear for the 1.0 s stillness hold.
    """
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

    sx, sy = STAND_XY
    px, py = POT_XY
    hx, hy = POT_HANDLE_XY
    bx, by = BRACKET_XY

    pad_grasp_z = PADDLE_START_POS[2] + PADDLE_GRIP_Z_LOCAL
    pad_carry_z = 0.950
    pad_submerge_z = 0.878
    pad_drain_z = 0.910
    bracket_seat_z = 0.864
    bracket_approach_z = 0.900

    yaw_grip = 0.0

    # Right arm: grasp stockpot loop handle to steady the pot
    add("right", 1.5, (hx, hy, POT_HANDLE_Z + 0.050), FINGER_OPEN, "above pot handle", 0.0)
    add("right", 1.0, (hx, hy, POT_HANDLE_Z), FINGER_OPEN, "descend to pot handle", 0.0)
    add("right", 0.6, (hx, hy, POT_HANDLE_Z), POT_HANDLE_PINCH, "pinch pot handle", 0.0)

    # Left arm: approach paddle stand, grasp handle, lift clear
    add("left", 1.5, (sx, sy, pad_grasp_z + 0.035), FINGER_OPEN, "above paddle handle", yaw_grip)
    add("left", 1.0, (sx, sy, pad_grasp_z), FINGER_OPEN, "descend to handle", yaw_grip)
    add("left", 0.6, (sx, sy, pad_grasp_z), HANDLE_PINCH, "grasp handle", yaw_grip)
    add("left", 1.0, (sx, sy, pad_carry_z), HANDLE_PINCH, "lift paddle vertically", yaw_grip)
    add("left", 1.5, (px, py, pad_carry_z), HANDLE_PINCH, "carry above stockpot", yaw_grip)

    if variant == "no_submerge":
        add("left", 1.5, LEFT_PARK, FINGER_OPEN, "park left", 0.0)
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Lower into hot broth
    add("left", 1.0, (px, py, pad_submerge_z), HANDLE_PINCH, "submerge paddle into broth", yaw_grip)

    # Circular agitation revolutions
    n_revs = 0 if variant == "no_agitation" else 5
    px_stir = px + 0.008
    py_stir = py - 0.006
    R = 0.035
    t_step = 0.05 if variant == "fast_splash" else 0.45
    for rev in range(n_revs):
        for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
            th = math.radians(angle_deg)
            add("left", t_step, (px_stir + R * math.cos(th), py_stir + R * math.sin(th), pad_submerge_z), HANDLE_PINCH, f"stir rev {rev}", yaw_grip)
    add("left", t_step, (px_stir + R, py_stir, pad_submerge_z), HANDLE_PINCH, "stir finish", yaw_grip)

    if variant == "no_drain":
        pass
    else:
        # Lift to drain height above liquid surface
        add("left", 1.0, (px, py, pad_drain_z), HANDLE_PINCH, "lift to drain height", yaw_grip)
        add("left", 1.2, (px, py, pad_drain_z), HANDLE_PINCH, "drain broth dwell", yaw_grip)

    # Move paddle towards rim hanging bracket
    add("left", 1.2, (bx, by - 0.008, bracket_approach_z), HANDLE_PINCH, "approach rim bracket", yaw_grip)
    add("left", 0.8, (bx, by - 0.008, bracket_seat_z), HANDLE_PINCH, "lower hook onto bracket", yaw_grip)
    add("left", 0.6, (bx, by - 0.008, bracket_seat_z), FINGER_OPEN, "release hook", yaw_grip)
    add("left", 0.8, (bx, by - 0.008, 0.940), FINGER_OPEN, "retract above paddle", 0.0)
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm", 0.0)

    t_left_hung = clock["left"]
    wait_until("right", t_left_hung - 1.8, "steady pot through hang")

    # Right arm: release pot handle and park
    add("right", 0.6, (hx, hy, POT_HANDLE_Z), FINGER_OPEN, "release pot handle", 0.0)
    add("right", 1.5, RIGHT_PARK, FINGER_OPEN, "park right arm", 0.0)

    end_t = max(clock.values()) + 2.2
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")
    return plans


# -- offline self-test ------------------------------------------------------------------------------------------

def _build_state(
    paddle_pos=PADDLE_START_POS,
    paddle_quat=(0.0, 0.0, 0.0, 1.0),
    paddle_vel=(0.0, 0.0, 0.0),
    pot_pos=(POT_XY[0], POT_XY[1], TOP),
    pot_quat=(0.0, 0.0, 0.0, 1.0),
    pot_tilt_deg=0.0,
) -> dict:
    if pot_tilt_deg > 0.0:
        rad = math.radians(pot_tilt_deg)
        pot_quat = (math.sin(rad / 2), 0.0, 0.0, math.cos(rad / 2))
    moved = {
        "stockpot": (np.array(pot_pos, dtype=float), pot_quat),
        "ice_paddle": (np.array(paddle_pos, dtype=float), paddle_quat),
    }
    st = kit.fake_states(moved=moved)
    st["ice_paddle"] = dataclasses.replace(
        st["ice_paddle"],
        lin_vel=np.array(paddle_vel, dtype=float),
    )
    return st


def self_test() -> int:
    """Comprehensive offline unit test covering all stages and negative controls."""
    global _TEST_TCPS, _TEST_ROBOT_BOXES

    s_init = _build_state()
    s_lifted = _build_state(paddle_pos=(STAND_XY[0], STAND_XY[1], PADDLE_START_POS[2] + 0.035))
    s_submerged = _build_state(paddle_pos=(POT_XY[0], POT_XY[1], (BROTH_SURFACE_Z - 0.025) + PADDLE_BLADE_L))
    s_drained = _build_state(paddle_pos=(POT_XY[0], POT_XY[1], (BROTH_SURFACE_Z + 0.008) + PADDLE_BLADE_L))
    s_hung = _build_state(paddle_pos=(BRACKET_XY[0], BRACKET_XY[1] - 0.015, BRACKET_SEAT_Z + 0.004))

    stir_steps = []
    R = 0.035
    sub_z = (BROTH_SURFACE_Z - 0.025) + PADDLE_BLADE_L
    for rev in range(5):
        for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
            th = math.radians(angle_deg)
            stir_steps.append(_build_state(paddle_pos=(POT_XY[0] + R * math.cos(th), POT_XY[1] + R * math.sin(th), sub_z)))
    stir_steps.append(_build_state(paddle_pos=(POT_XY[0] + R, POT_XY[1], sub_z)))

    drain_steps = [_build_state(paddle_pos=(POT_XY[0], POT_XY[1], (BROTH_SURFACE_Z + 0.008) + PADDLE_BLADE_L)) for _ in range(18)]

    tcps_clear = [(0.160, 0.280, 0.920), (0.180, -0.260, 0.920)]
    rboxes_clear = [
        ((0.12, 0.24, 0.88), (0.20, 0.32, 0.96)),
        ((0.14, -0.30, 0.88), (0.22, -0.22, 0.96)),
    ]

    cases = {
        "initial state scores 0": ([s_init], tcps_clear, rboxes_clear, 0.0),
        "lifted paddle scores 0.15": ([s_init, s_lifted], tcps_clear, rboxes_clear, 0.15),
        "submerged in broth scores 0.35": ([s_init, s_lifted, s_submerged], tcps_clear, rboxes_clear, 0.35),
        "3 agitation revolutions scores 0.60": ([s_init, s_lifted, *stir_steps[:25]], tcps_clear, rboxes_clear, 0.60),
        "5 agitation revolutions + drained scores 0.80": ([s_init, s_lifted, *stir_steps, *drain_steps], tcps_clear, rboxes_clear, 0.80),
        "full agitation + rim hang + clear hands scores 1.0": ([s_init, s_lifted, *stir_steps, *drain_steps, s_hung], tcps_clear, rboxes_clear, 1.0),
        "hands touching paddle at end caps at 0.80": ([s_init, s_lifted, *stir_steps, *drain_steps, s_hung], [(BRACKET_XY[0], BRACKET_XY[1], 0.84)], rboxes_clear, 0.80),
        "splash speed violation latches 0.0": ([s_init, s_lifted, _build_state(paddle_pos=(POT_XY[0], POT_XY[1], sub_z), paddle_vel=(0.60, 0.0, 0.0)), s_hung], tcps_clear, rboxes_clear, 0.0),
        "stockpot tipped > 10 deg latches 0.0": ([s_init, s_lifted, *stir_steps, _build_state(paddle_pos=s_hung["ice_paddle"].pos, pot_tilt_deg=15.0)], tcps_clear, rboxes_clear, 0.0),
        "premature hang bypass without stirring scores 0.0": ([s_init, s_lifted, s_hung], tcps_clear, rboxes_clear, 0.0),
    }

    try:
        for name, (seq, tcps, rboxes, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            _TEST_ROBOT_BOXES = rboxes
            for idx, st in enumerate(seq, start=1):
                mk._EP["step"] = idx
                _step_rules(st)
            got = score(seq[-1])
            got_ok = success(seq[-1])
            assert abs(got - want) < 1e-6 and got_ok == (want >= 1.0 - 1e-9), (name, got, want)
    finally:
        mk.new_episode()
        mk._EP["step"] = 0
        _TEST_TCPS = None
        _TEST_ROBOT_BOXES = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    n_pass = self_test()
    print(f"scoring self_test: {n_pass}/{n_pass} cases pass")
    hz = 20.0
    demo_dict = demo()
    for arm, waypoints in demo_dict.items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz))])
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.4f} mm / {plan.worst_ik_deg:.4f} deg")
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
    print("ALL TESTS AND IK PASSED SUCCESSFULLY!")
