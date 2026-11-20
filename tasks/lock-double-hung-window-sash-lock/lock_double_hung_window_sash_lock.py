"""Shut a double-hung window against its sill weatherstrip and lock its cam sash lock.

A complete compact wood double-hung window (600 mm wide x 489 mm tall frame on a 10 mm steel mounting plate,
nominal 24 x 20 in utility/basement unit) stands upright across the bench facing the robot. The upper sash is
fixed shut in the upper half of the frame; the lower sash slides vertically in the side jambs on sash-balance
springs (100 N/m) with 3.0 N of jamb-liner Coulomb friction, so it stays wherever it is left. It starts 12 mm
above shut, clear of the black EPDM bulb weatherstrip on the sill (10 mm free height, 600 N/m = 6.0 N at full
compression). A die-cast cam sash lock (2-1/16 in / 52.4 mm checkrail pattern) sits at the middle of the lower
meeting rail, and its bridge keeper sits opposite on the upper meeting rail.

Physical coupling: turning the lever 150 deg swings a 3.0 mm thick flat cam across the parting gap and under the
keeper's bridge (underside 7.5 mm above the flush meeting rails), blocking the lower sash from rising.
  - Only while the lower sash is held within 1.5 mm of the sill stops does the cam pass under the bridge.
  - If the sash is pushed shut and released before locking, the 6.0 N sill bulb seal pushes it back up ~4 mm into
    the 1.5..7.5 mm jamming band, where the cam strikes the keeper bridge's edge and stalls short of locked.
  - If the lever is thrown with the sash left >= 7.5 mm ajar (such as at the 12 mm start), the cam swings clear
    over the top of the keeper bridge (10.5 mm): a false lock that looks thrown from above while holding nothing.

Scoring reads poses, full-arm collision clearance and an active 30 N upward lift test on the lower meeting rail.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box rigid parts, sprung/friction joints and per-episode hooks)

from yam_arena import demo as _harness_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- full compact double-hung window unit (metres; x forward from arm bases, y left, z up) -------------------------
SASH_T = 0.0349       # 1-3/8 in wood sash thickness
RAIL_H = 0.0286       # 1-1/8 in meeting rail height
OUTER_RAIL_H = 0.035  # bottom rail of lower sash and top rail of upper sash
STILE_W = 0.035       # sash stile width
SASH_W = 0.520        # sash width (nominal 24 in window; outer frame width 600 mm)
PANE_T = 0.004        # 4 mm clear polycarbonate glazing sheet in each sash
PARTING = 0.0025      # 2.5 mm parting gap between lower and upper meeting rails

XL = 0.310                        # lower (inside) sash mid-plane
XU = XL + SASH_T + PARTING        # upper (outside) sash mid-plane (0.3474 m)
BASE_T = 0.010                    # 10 mm steel mounting base plate on the bench
BASE_TOP = TOP + BASE_T           # TOP + 10 mm
SILL_TOP = BASE_TOP + 0.018       # TOP + 28 mm: wood sill floor between the sill stops
CUT_Z = SILL_TOP + 0.012          # TOP + 40 mm: top of steel sill stops (lower sash bottom at shut)
LOWER_H = 0.205                   # full lower sash height (bottom rail + glazed daylight opening + meeting rail)
ZR = CUT_Z + LOWER_H              # TOP + 245 mm: top of both meeting rails when the lower sash is shut
UPPER_H = 0.235                   # full upper sash height (meeting rail + glazed daylight opening + top rail)
HEAD_BOT = ZR - RAIL_H + UPPER_H  # TOP + 451.4 mm: underside of head jamb
HEAD_H = 0.038                    # 38 mm painted pine head jamb (frame top at TOP + 489.4 mm)
JAMB_W = 0.038                    # 38 mm painted pine side jambs
JAMB_CLEAR = 0.002                # 2 mm running clearance per side between sash and jamb
JAMB_Y = SASH_W / 2 + JAMB_CLEAR  # 0.262 m: inner jamb face (outer frame spans y = -0.300..+0.300 m)
JAMB_D = 2 * SASH_T + PARTING + 0.0237  # 0.096 m (3-3/4 in jamb depth)

AJAR = 0.012          # lower sash starts 12 mm above shut (meeting rail top at TOP + 257 mm)
SASH_KG = 1.50        # complete glazed lower sash with sash lock base
BALANCE_K = 100.0     # N/m: pair of sash balances carrying the sash weight at the sill
SASH_FRICTION = 3.0   # N: jamb-liner Coulomb friction (holds the ajar sash where left)
SEAL_FREE = 0.010     # m: EPDM sill bulb weatherstrip stands 10 mm above the sill stops when free
SEAL_K = 600.0        # N/m: 6.0 N upward weatherstrip force at full 10 mm compression
SEAL_KG = 0.025

# -- cam sash lock (2-1/16 in / 52.4 mm checkrail pattern) & bridge keeper ----------------------------------------
PIVOT = (XL + 0.008, 0.0)
THROW = 150.0         # deg from unlocked stop (0) to locked stop (150)
LEVER0 = 120.0        # unlocked lever direction (deg from +x, CCW seen from above); locked = 270 deg (-y)
CAM0 = -150.0         # unlocked cam direction (over lower rail); locked = 0 deg (+x, under keeper bridge)
CAM_R0, CAM_R1, CAM_W, CAM_T = 0.006, 0.024, 0.012, 0.0030
CAM_Z = 0.0030        # underside of cam above lower meeting rail (top at 6.0 mm)
HUB = (0.012, 0.012, 0.009)
LEVER_R0, LEVER_R1, LEVER_T, LEVER_H = 0.006, 0.034, 0.005, 0.014
LEVER_Z = 0.012       # underside of upright finger lever tab above meeting rail (12..26 mm)
LOCK_KG = 0.050
LOCK_FRICTION = 0.04  # N*m detent/pivot drag

KEEP_X0, KEEP_X1 = 0.012, 0.030   # keeper bridge x span measured forward from PIVOT[0]
KEEP_Y = 0.012                    # keeper half-width in y (-12..+12 mm)
BRIDGE_Z0, BRIDGE_Z1 = 0.0075, 0.0105  # underside (7.5 mm) and top (10.5 mm) of keeper bridge above ZR
CAM_PLAY = BRIDGE_Z0 - (CAM_Z + CAM_T)  # 1.5 mm vertical clearance when sash rests on sill stops
OVER_M = BRIDGE_Z1 - CAM_Z              # 7.5 mm: at or above this, the cam swings over the keeper top

WOOD = (0.93, 0.92, 0.88)
GLASS = (0.76, 0.85, 0.90)
BRASS = (0.78, 0.62, 0.26)
STEEL = (0.55, 0.57, 0.60)
RUBBER = (0.10, 0.10, 0.11)


def _polar(r, deg, z=0.0):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a), z)


def _lower_sash_boxes():
    """Part frame: origin at (XL, 0, ZR) on the top face of the lower meeting rail when shut."""
    inner_w = SASH_W - 2 * STILE_W
    pane_h = LOWER_H - RAIL_H - OUTER_RAIL_H
    pane_zc = -(RAIL_H + pane_h / 2)
    px = PIVOT[0] - XL
    return [
        mk.box((SASH_T, inner_w, RAIL_H), at=(0.0, 0.0, -RAIL_H / 2)),
        mk.box((SASH_T, inner_w, OUTER_RAIL_H), at=(0.0, 0.0, -LOWER_H + OUTER_RAIL_H / 2)),
        mk.box((SASH_T, STILE_W, LOWER_H), at=(0.0, (SASH_W - STILE_W) / 2, -LOWER_H / 2)),
        mk.box((SASH_T, STILE_W, LOWER_H), at=(0.0, -(SASH_W - STILE_W) / 2, -LOWER_H / 2)),
        mk.box((PANE_T, inner_w, pane_h), at=(0.0, 0.0, pane_zc), color=GLASS),
        mk.box((0.020, 0.057, 0.0025), at=(px, 0.0, 0.00125), color=BRASS),
    ]


def _upper_sash_boxes():
    """Part frame: origin at (XU, 0, ZR), meeting rail top flush with ZR and top rail under HEAD_BOT."""
    inner_w = SASH_W - 2 * STILE_W
    pane_h = UPPER_H - RAIL_H - OUTER_RAIL_H
    pane_zc = -RAIL_H + RAIL_H + pane_h / 2
    return [
        mk.box((SASH_T, inner_w, RAIL_H), at=(0.0, 0.0, -RAIL_H / 2)),
        mk.box((SASH_T, inner_w, OUTER_RAIL_H), at=(0.0, 0.0, -RAIL_H + UPPER_H - OUTER_RAIL_H / 2)),
        mk.box((SASH_T, STILE_W, UPPER_H), at=(0.0, (SASH_W - STILE_W) / 2, -RAIL_H + UPPER_H / 2)),
        mk.box((SASH_T, STILE_W, UPPER_H), at=(0.0, -(SASH_W - STILE_W) / 2, -RAIL_H + UPPER_H / 2)),
        mk.box((PANE_T, inner_w, pane_h), at=(0.0, 0.0, pane_zc), color=GLASS),
    ]


def _lock_boxes():
    """Part frame: origin on the pivot axis at the lower meeting rail's top face; reference pose unlocked."""
    cr, lr = (CAM_R0 + CAM_R1) / 2, (LEVER_R0 + LEVER_R1) / 2
    return [
        mk.box(HUB, at=(0.0, 0.0, CAM_Z + HUB[2] / 2)),
        mk.box((CAM_R1 - CAM_R0, CAM_W, CAM_T), at=_polar(cr, CAM0, CAM_Z + CAM_T / 2), yaw=math.radians(CAM0)),
        mk.box((LEVER_R1 - LEVER_R0, LEVER_T, LEVER_H), at=_polar(lr, LEVER0, LEVER_Z + LEVER_H / 2),
               yaw=math.radians(LEVER0)),
    ]


def _keeper_boxes():
    """Part frame: origin at (PIVOT[0], PIVOT[1], ZR). Bridge, back wall, +y end post and mounting flange."""
    kx, kw = (KEEP_X0 + KEEP_X1) / 2, KEEP_X1 - KEEP_X0
    return [
        mk.box((kw, 2 * KEEP_Y, BRIDGE_Z1 - BRIDGE_Z0), at=(kx, 0.0, (BRIDGE_Z0 + BRIDGE_Z1) / 2)),
        mk.box((0.003, 2 * KEEP_Y, BRIDGE_Z0), at=(KEEP_X1 - 0.0015, 0.0, BRIDGE_Z0 / 2)),
        mk.box((kw, 0.003, BRIDGE_Z0), at=(kx, KEEP_Y - 0.0015, BRIDGE_Z0 / 2)),
        mk.box((kw, 0.056, 0.0020), at=(kx, 0.0, 0.0010)),
    ]


_FRAME_X = (XL + XU) / 2
_OUTER_W = 2 * (JAMB_Y + JAMB_W)

kit = mk.Kit()
kit.part("base_plate", (_FRAME_X, 0.0, TOP + BASE_T / 2), [mk.box((0.140, _OUTER_W + 0.020, BASE_T))], color=STEEL)
kit.part("wood_sill", (_FRAME_X, 0.0, (BASE_TOP + SILL_TOP) / 2),
         [mk.box((JAMB_D, _OUTER_W, SILL_TOP - BASE_TOP))], color=WOOD)
kit.part("head_jamb", (_FRAME_X, 0.0, HEAD_BOT + HEAD_H / 2),
         [mk.box((JAMB_D, _OUTER_W, HEAD_H))], color=WOOD)
for _s, _n in ((1, "left"), (-1, "right")):
    kit.part(f"jamb_{_n}", (_FRAME_X, _s * (JAMB_Y + JAMB_W / 2), (SILL_TOP + HEAD_BOT) / 2),
             [mk.box((JAMB_D, JAMB_W, HEAD_BOT - SILL_TOP))], color=WOOD)
    kit.part(f"sill_stop_{_n}", (XL, _s * (SASH_W - STILE_W) / 2, (SILL_TOP + CUT_Z - 0.0005) / 2),
             [mk.box((SASH_T, STILE_W, CUT_Z - 0.0005 - SILL_TOP))], color=STEEL)
kit.part("upper_sash", (XU, 0.0, ZR), _upper_sash_boxes(), color=WOOD)
kit.part("keeper", (PIVOT[0], PIVOT[1], ZR), _keeper_boxes(), color=BRASS)
kit.part("sill_seal", (XL, 0.0, CUT_Z + SEAL_FREE - 0.004), [mk.box((0.016, 0.320, 0.008))], mass=SEAL_KG,
         color=RUBBER)
kit.part("lower_sash", (XL, 0.0, ZR), _lower_sash_boxes(), mass=SASH_KG, color=WOOD,
         solver_position_iterations=64)
kit.part("sash_lock", (PIVOT[0], PIVOT[1], ZR), _lock_boxes(), mass=LOCK_KG, color=BRASS,
         solver_position_iterations=64)

# Sill bulb weatherstrip spring: free at 0 (10 mm above sill stops), fully compressed at -10 mm (6.0 N).
kit.slider("seal_spring", None, "sill_seal", axis=(0, 0, 1), lower=-SEAL_FREE, upper=0.0, start=0.0,
           stiffness=SEAL_K, rest=0.0, damping=2.0)
# Lower sash in jamb liners: 0 is shut on the sill stops; balances carry sash weight at the sill.
kit.slider("sash_slide", None, "lower_sash", axis=(0, 0, 1), lower=0.0, upper=0.060, start=AJAR,
           stiffness=BALANCE_K, rest=SASH_KG * 9.81 / BALANCE_K, damping=8.0, friction=SASH_FRICTION)
kit.hinge("lock_turn", "lower_sash", "sash_lock", anchor=(PIVOT[0], PIVOT[1], ZR), axis=(0, 0, 1), lower_deg=0.0,
          upper_deg=THROW, start_deg=0.0, friction=LOCK_FRICTION)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring & active 30 N retention test -------------------------------------------------------------------------

SHUT_M = 0.0024        # lower sash counts as shut within 2.4 mm of the sill stops
ENGAGED_DEG = 120.0    # cam tip has entered under the keeper bridge
LOCKED_DEG = 145.0     # lever within 5 deg of its 150 deg locked stop
LIFT_N = 30.0          # N straight up on the middle of the lower meeting rail
LIFT_STEPS = 10        # 0.5 s at 20 Hz
LIFT_MOVE = 0.0015     # m: sash may rise at most 1.5 mm during the 30 N pull (and must remain <= SHUT_M)
CLEAR_M = 0.020        # m: every robot collision hull at least 20 mm from lower sash, lock and keeper
_TEST_ROBOT_BOXES = None

_CAM_TIP_LOCAL = _polar(CAM_R1 - 0.002, CAM0, CAM_Z + CAM_T)  # top-tip of the cam in sash_lock's frame


def sash_up(objects) -> float:
    """Height of the lower sash above its sill stops (m)."""
    return kit.q(objects, "sash_slide")


def lock_deg(objects) -> float:
    """Angle of the sash-lock lever from its unlocked stop (deg, 0 to 150)."""
    return kit.q_deg(objects, "lock_turn")


def shut(objects) -> bool:
    return -0.001 <= sash_up(objects) <= SHUT_M


def lever_thrown(objects) -> bool:
    return lock_deg(objects) >= LOCKED_DEG


def _cam_tip_under_bridge(objects, min_in_x=0.002) -> bool:
    """3D check on the actual cam body: tip inside the keeper pocket and beneath the keeper bridge underside."""
    tip = kit.point(objects, "sash_lock", _CAM_TIP_LOCAL)
    dx = float(tip[0] - PIVOT[0])
    dy = float(tip[1] - PIVOT[1])
    dz = float(tip[2] - ZR)
    return (KEEP_X0 + min_in_x <= dx <= KEEP_X1 and abs(dy) <= KEEP_Y - 0.0015
            and CAM_Z - 0.001 <= dz <= BRIDGE_Z0 + 0.0015)


def cam_engaged(objects) -> bool:
    """Sash shut and cam rotated partway under the keeper bridge (120..145 deg), retaining the sash."""
    return shut(objects) and lock_deg(objects) >= ENGAGED_DEG and _cam_tip_under_bridge(objects, min_in_x=0.001)


def cam_under_keeper(objects) -> bool:
    """Sash shut, lever fully thrown (>= 145 deg), and cam physically under the keeper bridge."""
    return shut(objects) and lever_thrown(objects) and _cam_tip_under_bridge(objects, min_in_x=0.004)


def hands_off(objects) -> bool:
    """Every collision hull of both arms and grippers at least 20 mm from lower_sash, sash_lock and keeper."""
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES
    for name in ("lower_sash", "sash_lock", "keeper"):
        st = objects[name]
        lo_s, hi_s = np.asarray(st.low), np.asarray(st.high)
        for lo, hi in boxes:
            gap = np.maximum(np.maximum(lo_s - hi, lo - hi_s), 0.0)
            if float(np.linalg.norm(gap)) < CLEAR_M:
                return False
    return True


def _lift_rule(objects) -> None:
    """Active 30 N upward pry test once cam_under_keeper holds with arms clear and sash still for 0.5 s."""
    m = mk.MEMORY
    if not cam_under_keeper(objects):
        if m.get("lift_n"):
            kit.pull("lower_sash")
        m.update(held=False, lift_n=0, still_n=0)
        return
    if m.get("held"):
        return
    h, off = sash_up(objects), hands_off(objects)
    n = m.get("lift_n", 0)
    if n:
        m["lift_peak"] = max(m.get("lift_peak", 0.0), h - m["lift_from"])
        m["lift_max_h"] = max(m.get("lift_max_h", h), h)
        if n < LIFT_STEPS and off:
            m["lift_n"] = n + 1
            return
        kit.pull("lower_sash")
        m.update(lift_n=0, still_n=0, lifts=m.get("lifts", 0) + 1)
        m["held"] = bool(off and n >= LIFT_STEPS and m["lift_peak"] <= LIFT_MOVE and m["lift_max_h"] <= SHUT_M)
        return
    still = m.get("still_n", 0)
    if not off or abs(h - m.get("still_from", h)) > 0.0003:
        still = 0
    if not still:
        m["still_from"] = h
    m["still_n"] = still = still + 1 if off else 0
    if still >= 11:  # 11 samples span 0.5 s at 20 Hz
        m.update(lift_from=h, lift_peak=0.0, lift_max_h=h)
        if kit.pull("lower_sash", (0.0, 0.0, LIFT_N), kit.point(objects, "lower_sash", (0.0, 0.0, -RAIL_H / 2))):
            m["lift_n"] = 1


kit.on_step(_lift_rule)


def _trace(objects) -> None:
    path = os.environ.get("SASH_TRACE")
    if path:
        m = mk.MEMORY
        tcp = " ".join(f"{(t[0] - PIVOT[0]) * 1000:.1f},{(t[1] - PIVOT[1]) * 1000:.1f},{(t[2] - ZR) * 1000:.1f}"
                       for t in mk.tcps())
        with open(path, "a") as f:
            f.write(f"{sash_up(objects) * 1000:.2f} {lock_deg(objects):.1f} {kit.q(objects, 'seal_spring') * 1000:.2f} "
                    f"{int(hands_off(objects))} {m.get('still_n', 0)} {m.get('lift_n', 0)} "
                    f"{m.get('lift_peak', 0.0) * 1000:.2f} {int(bool(m.get('held')))} {score(objects):.2f} {tcp}\n")


kit.on_step(_trace)


def locked(objects) -> bool:
    """Cam under keeper, 30 N lift test passed, arms clear, sash and lock resting."""
    return bool(cam_under_keeper(objects) and mk.MEMORY.get("held") and hands_off(objects)
                and objects["lower_sash"].resting() and objects["sash_lock"].resting())


def score(objects) -> float:
    """Progressive stage score at the end of the episode:
      1.0: sash shut, cam locked under keeper, arms >= 20 mm clear, 30 N lift test passed, sash & lock at rest
      0.7: sash shut and lever thrown (>= 145 deg) with cam under keeper, before release / lift-test completion
      0.5: sash shut and cam engaged partway under keeper (120..145 deg), retaining sash against the sill seal
      0.3: sash shut (<= 2.4 mm above sill stops), lock not yet engaged under keeper
      0.0: sash not shut (including false lock with cam over keeper, or early release sprung back on weatherstrip)"""
    if cam_under_keeper(objects):
        return 1.0 if locked(objects) else 0.7
    if cam_engaged(objects):
        return 0.5
    if shut(objects):
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres the sash stands above shut plus 0.1 mm per degree the lever is short of thrown."""
    return float(max(0.0, sash_up(objects)) + 1e-4 * max(0.0, THROW - lock_deg(objects)))


# -- scripted demonstration & diagnostic wrong-way variants -------------------------------------------------------


class _Grip(float):
    """Waypoint yaw carrying a full 3x3 TCP rotation matrix."""

    rot = None


def _grip(approach, finger_axis) -> _Grip:
    a = np.asarray(approach, float)
    a /= np.linalg.norm(a)
    gx = np.asarray(finger_axis, float)
    gx = gx - a * float(gx @ a)
    gx /= np.linalg.norm(gx)
    g = _Grip(0.0)
    g.rot = np.column_stack([gx, np.cross(a, gx), a])
    return g


if not getattr(_harness_demo.grip_rotation, "_takes_grips", False):
    _plain_rotation = _harness_demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return _plain_rotation(yaw) if rot is None else rot

    _grip_rotation._takes_grips = True
    _harness_demo.grip_rotation = _grip_rotation


TIP = 0.013          # pad tips past TCP along approach axis
HOVER_DZ = 0.052
PRESS_Y = 0.185      # left arm presses lower meeting rail 18.5 cm left of lock (clear of stile and right arm)
PRESS_OVER = 0.038   # commanded below ZR so arm compliance holds ~10 N down against the 6 N sill bulb seal
GRIP_R = 0.025       # pinch radius on lever tab from pivot (clears 12 mm hub by > 11 mm)
LEVER_MID_Z = LEVER_Z + 0.0065  # 18.5 mm above meeting rail (middle of 12..26 mm lever tab)
OPEN_GAP = -0.020    # 40 mm finger opening around 5 mm thick lever tab
APP_L = np.array([0.65, -0.32, -0.69]) / np.linalg.norm([0.65, -0.32, -0.69])
APP_R = np.array([0.68, 0.22, -0.70]) / np.linalg.norm([0.68, 0.22, -0.70])
GRIP_L = _grip(APP_L, (1.0, 0.0, 0.0))
PARK_L = (0.145, 0.270, TOP + 0.230)
PARK_R = (0.145, -0.270, TOP + 0.230)


def _tcp_for(contact_xyz, app):
    return tuple(np.asarray(contact_xyz, float) - TIP * app)


def _lever_grip(deg) -> _Grip:
    ang = math.radians(LEVER0 + deg)
    return _grip(APP_R, (-math.sin(ang), math.cos(ang), 0.0))


def _lever_tcp(deg, sash_h=0.0, dz=0.0):
    x, y, _ = _polar(GRIP_R, LEVER0 + deg)
    return _tcp_for((PIVOT[0] + x, PIVOT[1] + y, ZR + sash_h + LEVER_MID_Z + dz), APP_R)


def _press(hold_s, lift_off=True):
    c = FINGER_CLOSED
    p_hover = _tcp_for((XL - 0.002, PRESS_Y, ZR + AJAR + HOVER_DZ), APP_L)
    p_touch = _tcp_for((XL - 0.002, PRESS_Y, ZR + AJAR + 0.003), APP_L)
    p_down = _tcp_for((XL - 0.002, PRESS_Y, ZR - PRESS_OVER), APP_L)
    plan = [
        Waypoint(1.8, p_hover, c, "over left side of lower meeting rail", GRIP_L),
        Waypoint(0.9, p_touch, c, "onto meeting rail", GRIP_L),
        Waypoint(1.4, p_down, c, "compress sill weatherstrip to sill stops", GRIP_L),
        Waypoint(hold_s, p_down, c, "hold sash shut against weatherstrip", GRIP_L),
    ]
    if lift_off:
        plan += [
            Waypoint(0.9, p_hover, c, "lift clear of meeting rail", GRIP_L),
            Waypoint(1.4, PARK_L, c, "park clear", GRIP_L),
        ]
    return plan


def _throw(wait_s, sash_h=0.0, target_deg=THROW, step=15):
    g0 = _lever_grip(0)
    plan = [
        Waypoint(2.0, _lever_tcp(0, sash_h, HOVER_DZ), OPEN_GAP, "above unlocked sash-lock lever", g0),
        Waypoint(wait_s, _lever_tcp(0, sash_h, HOVER_DZ), OPEN_GAP, "wait for sash to seat", g0),
        Waypoint(1.0, _lever_tcp(0, sash_h, 0.0), OPEN_GAP, "straddle lever tab", g0),
        Waypoint(0.7, _lever_tcp(0, sash_h, 0.0), FINGER_CLOSED, "pinch lever tab", g0),
    ]
    over_deg = target_deg + (22.0 if target_deg >= THROW - 1e-6 else 10.0)
    angles = list(range(step, int(over_deg), step)) + [over_deg]
    for d in angles:
        plan.append(Waypoint(0.32, _lever_tcp(d, sash_h, 0.0), FINGER_CLOSED, f"turn lever to {d:.0f} deg", _lever_grip(d)))
    go = _lever_grip(over_deg)
    gf = _lever_grip(target_deg)
    return plan + [
        Waypoint(0.5, _lever_tcp(over_deg, sash_h, 0.0), FINGER_CLOSED, "seat against stop", go),
        Waypoint(0.25, _lever_tcp(target_deg, sash_h, 0.0), FINGER_CLOSED, "relax wrist preload", gf),
        Waypoint(0.7, _lever_tcp(target_deg, sash_h, 0.0), OPEN_GAP, "release lever", gf),
        Waypoint(0.9, _lever_tcp(target_deg, sash_h, HOVER_DZ), OPEN_GAP, "lift clear of lock", gf),
        Waypoint(1.4, PARK_R, OPEN_GAP, "park clear", gf),
    ]


def demo():
    """Bimanual reference: left hand holds lower sash shut against 6 N sill bulb seal while right hand throws the
    cam sash lock 150 deg under the keeper, then both retreat clear so the 30 N lift test runs.
    SASH_DEMO selects diagnostic controls: false_lock | early_release | hold_only | partial_throw | hand_on | idle."""
    kind = os.environ.get("SASH_DEMO", "")
    idle_l = [Waypoint(3.0, PARK_L, FINGER_CLOSED, "wait", GRIP_L)]
    idle_r = [Waypoint(3.0, PARK_R, OPEN_GAP, "wait", _lever_grip(0))]
    if kind == "idle":
        return {"left": idle_l, "right": idle_r}
    if kind == "false_lock":  # throws lever while sash stays 12 mm ajar (cam swings over keeper -> 0.0)
        return {"left": idle_l, "right": _throw(0.5, sash_h=AJAR)}
    if kind == "early_release":  # pushes sash shut then releases before locking (bounces up ~4 mm, cam jams -> 0.0)
        return {"left": _press(0.5), "right": _throw(5.8, sash_h=0.004)}
    if kind == "hold_only":  # holds sash shut to the end without touching lock (-> 0.3)
        return {"left": _press(13.0, lift_off=False), "right": idle_r}
    if kind == "partial_throw":  # holds sash shut, engages cam halfway under keeper (~130 deg), releases (-> 0.5)
        return {"left": _press(8.0), "right": _throw(2.2, target_deg=128)}
    if kind == "hand_on":  # shut and locked to 150 deg, but left hand stays on the meeting rail (-> 0.7)
        return {"left": _press(16.0, lift_off=False), "right": _throw(2.2)}
    if kind:
        raise ValueError(f"Unknown SASH_DEMO={kind!r}")
    return {"left": _press(8.6), "right": _throw(2.2)}


# -- offline self-test & full 3D collision-hull clearance check ---------------------------------------------------


def _states(h=AJAR, deg=0.0, moving=False):
    import dataclasses

    st = kit.fake_states(q={"sash_slide": h, "lock_turn": math.radians(deg),
                            "seal_spring": -max(0.0, SEAL_FREE - h)})
    if moving:
        st["lower_sash"] = dataclasses.replace(st["lower_sash"], lin_vel=np.array([0.0, 0.0, 0.05]))
    return st


def self_test():
    """Verify scoring stages, 3D cam-under-keeper anti-cheat, 30 N lift rule, and physical contact regimes."""
    global _TEST_ROBOT_BOXES
    far = [(np.array([0.0, 0.30, TOP + 0.15]), np.array([0.05, 0.35, TOP + 0.25]))]
    on_rail = [(np.array([XL - 0.01, PRESS_Y - 0.01, ZR]), np.array([XL + 0.01, PRESS_Y + 0.01, ZR + 0.03]))]
    on_keeper = [(np.array([PIVOT[0] + 0.015, -0.01, ZR + 0.005]), np.array([PIVOT[0] + 0.030, 0.01, ZR + 0.025]))]
    cases = {
        "start: sash 12 mm ajar, lock open": (_states(), far, False, 0.0),
        "false lock: lever thrown 150 deg with sash 12 mm ajar (cam over keeper)": (_states(AJAR, 150), far, False, 0.0),
        "early release: sash bounced back 4.2 mm on seal, lever jammed at 115 deg": (_states(0.0042, 115), far, False, 0.0),
        "bypass attempt: sash 2.5 mm up with lever at 150 deg": (_states(0.0025, 150), far, True, 0.0),
        "sash held shut, lock untouched": (_states(0.0, 0), on_rail, False, 0.3),
        "sash held shut, lever at 110 deg (cam not yet under keeper)": (_states(0.0005, 110), on_rail, False, 0.3),
        "sash shut, cam engaged partway under keeper (130 deg)": (_states(0.0008, 130), far, False, 0.5),
        "locked under keeper, left hand still on rail": (_states(0.0005, 150), on_rail, True, 0.7),
        "locked under keeper, robot near keeper": (_states(0.0005, 150), on_keeper, True, 0.7),
        "locked under keeper, arms clear, lift test not yet run": (_states(0.0008, 150), far, False, 0.7),
        "locked under keeper, lift test passed, sash still moving": (_states(0.0008, 150, moving=True), far, True, 0.7),
        "locked under keeper, arms clear, lift test passed, at rest": (_states(0.0008, 150), far, True, 1.0),
        "lever at 146 deg, shut at 1.0 mm, lift test passed": (_states(0.0010, 146), far, True, 1.0),
    }
    try:
        for name, (st, boxes, held, want) in cases.items():
            mk.new_episode()
            _TEST_ROBOT_BOXES = boxes
            mk.MEMORY["held"] = held
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
        # Lift-test state machine: 11 clear settled samples start the 30 N pull; arm intrusion aborts/resets;
        # rising past LIFT_MOVE fails; unlocking cancels a passed test.
        mk.new_episode()
        _TEST_ROBOT_BOXES = far
        lock_st = _states(0.0008, 150)
        for _ in range(10):
            _lift_rule(lock_st)
        assert mk.MEMORY["still_n"] == 10 and not mk.MEMORY.get("lift_n")
        _TEST_ROBOT_BOXES = on_rail
        _lift_rule(lock_st)
        assert mk.MEMORY["still_n"] == 0, "arm near sash resets settle counter"
        _TEST_ROBOT_BOXES = far
        mk.MEMORY.update(lift_n=LIFT_STEPS, lift_from=0.0008, lift_peak=0.0, lift_max_h=0.0008)
        _lift_rule(_states(0.0016, 150))
        assert mk.MEMORY["held"] and mk.MEMORY["lifts"] == 1, "0.8 mm rise under 30 N passes"
        mk.MEMORY.update(held=False, lift_n=LIFT_STEPS, lift_from=0.0004, lift_peak=0.0, lift_max_h=0.0004)
        _lift_rule(_states(0.0026, 150))
        assert not mk.MEMORY["held"], "rising past SHUT_M / LIFT_MOVE fails"
        mk.MEMORY["held"] = True
        _lift_rule(_states(0.0005, 110))
        assert not mk.MEMORY["held"], "unlocking cancels passed lift test"
    finally:
        mk.new_episode()
        _TEST_ROBOT_BOXES = None

    # Physical regime invariants:
    assert abs(CAM_PLAY - 0.0015) < 1e-9 and abs(OVER_M - 0.0075) < 1e-9
    assert CAM_PLAY < SHUT_M < OVER_M < AJAR
    # Released from shut without locking, the 6 N bulb weatherstrip lifts the sash into the keeper jamming band:
    c = (SASH_FRICTION + BALANCE_K * SEAL_FREE) / (SEAL_K + BALANCE_K)
    bounce_h = SEAL_FREE - c
    assert CAM_PLAY + 0.0015 < bounce_h < OVER_M - 0.0015, bounce_h
    assert AJAR > SEAL_FREE and BALANCE_K * AJAR < SASH_FRICTION
    assert math.hypot(CAM_R1, CAM_W / 2) < KEEP_X1 - 0.003
    assert PIVOT[0] - XL + KEEP_X0 >= SASH_T / 2 + PARTING - 1e-9
    assert LEVER_Z > BRIDGE_Z1 and CAM_Z + CAM_T < BRIDGE_Z0
    return len(cases) + 5


if __name__ == "__main__":
    from yam_arena import analysis, ik, station
    from yam_arena.constants import FINGER_M_PER_RAD
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for kind in ("", "false_lock", "early_release", "hold_only", "partial_throw", "hand_on", "idle"):
        os.environ["SASH_DEMO"] = kind
        plans = {}
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (kind, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (kind, arm)
            plans[arm] = plan
            print(f"{kind or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
        if kind == "":
            # Full 20 Hz 3D URDF collision-hull check: zero arm-arm overlap and zero glazing/upper-sash contact!
            dur = max(p.duration for p in plans.values())
            min_arms, min_ug, min_lg = np.inf, np.inf, np.inf
            ug_c = (XU, 0.0, ZR - RAIL_H + RAIL_H + (UPPER_H - RAIL_H - OUTER_RAIL_H) / 2)
            ug_s = (PANE_T, SASH_W - 2 * STILE_W, UPPER_H - RAIL_H - OUTER_RAIL_H)
            lg_c = (XL, 0.0, ZR - RAIL_H - (LOWER_H - RAIL_H - OUTER_RAIL_H) / 2)
            lg_s = (PANE_T, SASH_W - 2 * STILE_W, LOWER_H - RAIL_H - OUTER_RAIL_H)
            for k in range(0, round(dur * hz) + 5, 25):
                t = (k + 1) / hz
                hsets = {}
                for arm, plan in plans.items():
                    a7 = plan.action7(t)[0]
                    f_m = max(-0.04695, min(0.0, float(a7[6]) * FINGER_M_PER_RAD))
                    qm = {f"joint{i + 1}": float(a7[i]) for i in range(6)}
                    qm["joint7"] = qm["joint8"] = f_m
                    hsets[arm] = analysis.world_hulls(analysis.fk_link_poses_world(qm, station.BASES[arm]))
                da, _ = analysis.hull_set_distance(hsets["left"], hsets["right"])
                min_arms = min(min_arms, da)
                for arm, hs in hsets.items():
                    min_ug = min(min_ug, analysis.hulls_to_box(hs, ug_c, np.eye(3), ug_s)[0])
                    if arm == "right":  # the left hand's commanded press overshoot is stopped by the meeting rail
                        min_lg = min(min_lg, analysis.hulls_to_box(hs, lg_c, np.eye(3), lg_s)[0])
            assert min_arms > 0.030 and min_ug > 0.012 and min_lg > 0.010, (min_arms, min_ug, min_lg)
            print(f"3D hull clearances: arm-arm >{min_arms*1000:.1f} mm, upper-pane {min_ug*1000:.1f} mm, "
                  f"lower-pane {min_lg*1000:.1f} mm")
    os.environ.pop("SASH_DEMO", None)
