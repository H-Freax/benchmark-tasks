"""A booster high chair strapped to the bench, a toddler manikin sitting in it with its forearms resting just over
the tray. The tray (1.2 kg) slides toward the robot along the chair's two tray rails. It is held by a ratchet with
four detents 30 mm apart and comes off the rails 120 mm out. Two spring release paddles, one in a pocket on each
side under the tray's edge (0.30 m apart), each 12 N at full 8 mm travel, unlock the ratchet only while BOTH are
squeezed past 6 mm. Let go of either and the ratchet catches again in the nearest detent. The tray can be tipped
up off its rails, but then its back edge lifts into the child's forearms.

The job: squeeze both paddles together and draw the tray straight out until it leaves the rails, then set it down
flat on the bench.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TRAY_C = (0.93, 0.93, 0.90)  # white polypropylene
PADDLE_C = (0.20, 0.45, 0.80)
FRAME_C = (0.55, 0.58, 0.62)
SEAT_C = (0.85, 0.55, 0.30)
SKIN = (0.88, 0.72, 0.60)
SHIRT = (0.35, 0.65, 0.45)
DARK = (0.15, 0.15, 0.16)

kit = mk.Kit()

# -- tray (reference pose = latched in its first detent) --------------------------------------------------------
TX0 = 0.370  # tray centre x at the start
TD, TW = 0.220, 0.340  # depth (x), width (y)
DECK_T = 0.012
DB = TOP + 0.085  # deck underside
SKIRT_H = 0.040  # skirt below the deck underside
NOTCH_IN = 0.118  # side pockets for the paddles: |y| 0.118-0.170, x within +-25 mm of the tray centre
NOTCH_HX = 0.025
FIN = (0.030, 0.006, 0.037)  # fixed fin and paddle: 30 mm long, 6 mm thick, 37 mm tall
FIN_Y = 0.143  # fixed fin centre |y|
PAD_Y0 = 0.159  # paddle centre |y| at rest; squeezed 8 mm toward the fixed fin
PAD_TRAVEL = 0.008
PAD_K = 600.0
PAD_PRELOAD = 7.2  # N at rest; 12 N at full travel
FIN_Z = DB - 0.025 + FIN[2] / 2  # fins span DB-25 mm to DB+12 mm


def _tray_boxes():
    zc = DECK_T / 2  # local origin: tray centre at the deck underside
    half = TD / 2
    seg = half - NOTCH_HX  # deck side strips and skirts, in front of and behind the pocket
    out = [mk.box((TD, 2 * NOTCH_IN, DECK_T), at=(0, 0, zc))]
    for s in (1, -1):
        for e in (1, -1):
            xc = e * (NOTCH_HX + seg / 2)
            out.append(mk.box((seg, TW / 2 - NOTCH_IN, DECK_T), at=(xc, s * (NOTCH_IN + TW / 4 - NOTCH_IN / 2), zc)))
            out.append(mk.box((seg, 0.006, SKIRT_H), at=(xc, s * (TW / 2 - 0.003), -SKIRT_H / 2)))
        # fixed fin, hung from the deck by a hanger and a bridge under the pocket
        out.append(mk.box(FIN, at=(0, s * FIN_Y, FIN_Z - DB), color=PADDLE_C))
        out.append(mk.box((FIN[0], FIN_Y + 0.003 - 0.112, 0.008), at=(0, s * (0.112 + FIN_Y + 0.003) / 2, -0.031)))
        out.append(mk.box((FIN[0], 0.006, 0.035), at=(0, s * 0.115, -0.0175)))
    # front skirt; the back edge has none, so the deck slides over the rail blocks
    out.append(mk.box((0.006, TW, SKIRT_H), at=(-(half - 0.003), 0, -SKIRT_H / 2)))
    return out


# hidden carriage that runs in the rails (under the chair, between the manikin's legs)
CAR0 = (0.450, 0.0, TOP + 0.012)
kit.part("rail_carriage", CAR0, [mk.box((0.020, 0.020, 0.012))], mass=0.10, color=DARK)
RAIL_OUT = 0.120
DETENT = 0.030
LATCH_K, LATCH_C = 20000.0, 300.0
kit.slider("rail", None, "rail_carriage", axis=(-1, 0, 0), lower=0.0, upper=RAIL_OUT, stiffness=LATCH_K,
           damping=LATCH_C)
kit.part("tray", (TX0, 0.0, DB), _tray_boxes(), mass=1.12, color=TRAY_C)
kit.slider("seat", "rail_carriage", "tray", axis=(0, 0, 1), lower=0.0, upper=0.060, damping=5.0)
for side, s in (("left", 1), ("right", -1)):
    kit.part(f"paddle_{side}", (TX0, s * PAD_Y0, FIN_Z), [mk.box(FIN)], mass=0.040, color=PADDLE_C)
    kit.slider(f"paddle_{side}_travel", "tray", f"paddle_{side}", axis=(0, -s, 0), lower=0.0, upper=PAD_TRAVEL,
               stiffness=PAD_K, rest=-PAD_PRELOAD / PAD_K, damping=1.0)

# -- the manikin's forearms: sprung (the arms give 1 mm per 1.5 N), resting 4 mm over the deck ---------------------
FA_K = 1500.0
FA_KG = 0.15
FA = (0.152, 0.045, 0.035)
FA_X = 0.461
FA_Z = DB + DECK_T + 0.004
for side, s in (("left", 1), ("right", -1)):
    kit.part(f"forearm_{side}", (FA_X, s * 0.050, FA_Z), [mk.box(FA, at=(0, 0, FA[2] / 2))], mass=FA_KG, color=SKIN)
    kit.slider(f"forearm_{side}_lift", None, f"forearm_{side}", axis=(0, 0, 1), lower=0.0, upper=0.080,
               stiffness=FA_K, rest=FA_KG * 9.81 / FA_K, damping=10.0)

RAIL_X = (0.330, 0.530)  # rail blocks under the deck
FIXTURES = [
    Box("rail_l", pos=(sum(RAIL_X) / 2, 0.080, TOP + 0.042), size=(RAIL_X[1] - RAIL_X[0], 0.016, 0.084), color=FRAME_C),
    Box("rail_r", pos=(sum(RAIL_X) / 2, -0.080, TOP + 0.042), size=(RAIL_X[1] - RAIL_X[0], 0.016, 0.084), color=FRAME_C),
    Box("seat_pad", pos=(0.620, 0.0, TOP + 0.005), size=(0.160, 0.280, 0.010), color=SEAT_C),
    Box("backrest", pos=(0.680, 0.0, TOP + 0.170), size=(0.020, 0.280, 0.320), color=SEAT_C),
    Box("side_l", pos=(0.600, 0.150, TOP + 0.080), size=(0.180, 0.020, 0.160), color=SEAT_C),
    Box("side_r", pos=(0.600, -0.150, TOP + 0.080), size=(0.180, 0.020, 0.160), color=SEAT_C),
    Box("torso", pos=(0.590, 0.0, TOP + 0.150), size=(0.100, 0.180, 0.280), color=SHIRT),
    Box("head", pos=(0.590, 0.0, TOP + 0.360), size=(0.120, 0.120, 0.130), color=SKIN),
    Box("upper_arm_l", pos=(0.545, 0.050, TOP + 0.150), size=(0.012, 0.045, 0.080), color=SHIRT),
    Box("upper_arm_r", pos=(0.545, -0.050, TOP + 0.150), size=(0.012, 0.045, 0.080), color=SHIRT),
    Box("leg_l", pos=(0.470, 0.050, TOP + 0.0175), size=(0.140, 0.036, 0.035), color=DARK),
    Box("leg_r", pos=(0.470, -0.050, TOP + 0.0175), size=(0.140, 0.036, 0.035), color=DARK),
]
OBJECTS = [*kit.objects(), *FIXTURES]

FORCE_LIMIT = 20.0
SQUEEZED = 0.006
FREE_AT = RAIL_OUT - 0.015  # within 15 mm of the rail ends the tray lifts off
DEBUG = os.environ.get("HCT_DEBUG")


def _set_latch(on: bool, target: float = 0.0) -> None:
    path = mk._SIM["joint_paths"].get((kit.id, "rail"))
    if not path or mk._SIM["stage"] is None:
        return
    from pxr import UsdPhysics

    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")
    d.GetTargetPositionAttr().Set(float(target))
    d.GetStiffnessAttr().Set(LATCH_K if on else 0.0)
    d.GetDampingAttr().Set(LATCH_C if on else 20.0)


def _tilt(q) -> float:
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def forearm_force(objects) -> float:
    return FA_K * max(kit.q(objects, "forearm_left_lift"), kit.q(objects, "forearm_right_lift"), 0.0)


def squeezed(objects, side) -> bool:
    return kit.q(objects, f"paddle_{side}_travel") > SQUEEZED


def _rule(objects):
    M = mk.MEMORY
    if not M.get("init"):
        M.update(init=True, latched=True, free=False, detent=0.0)
        mk._set_joint_enabled(kit, "seat", True)
        _set_latch(True, 0.0)
    q = kit.q(objects, "rail")
    M["q_max"] = max(M.get("q_max", 0.0), q)
    both = squeezed(objects, "left") and squeezed(objects, "right")
    if not M["free"]:
        if both and M["latched"]:
            M["latched"] = False
            _set_latch(False, q)
        elif not both and not M["latched"]:
            det = min(3, max(0, round(q / DETENT))) * DETENT
            M.update(latched=True, detent=det)
            _set_latch(True, det)
        if both and q >= FREE_AT:
            M["free"] = True
            mk._set_joint_enabled(kit, "seat", False)
    f = forearm_force(objects)
    M["f_max"] = max(M.get("f_max", 0.0), f)
    if f > FORCE_LIMIT:
        M["pressed_child"] = True
    st = objects["tray"]
    if float(st.lin_vel[2]) < -1.0 or float(st.low[2]) < TOP - 0.05:
        M["dropped"] = True
    if DEBUG and mk._EP["step"] % 10 == 0:
        p, qt = kit.pose(objects, "tray")
        print(f"[hct] step {mk._EP['step']} rail {q * 1000:.1f} mm paddles "
              f"{kit.q(objects, 'paddle_left_travel') * 1000:.1f}/{kit.q(objects, 'paddle_right_travel') * 1000:.1f} "
              f"tray {np.round(p, 4)} tilt {_tilt(qt):.1f} low {float(st.low[2]) - TOP:.4f} vz {float(st.lin_vel[2]):.3f} "
              f"F {f:.1f} tcp {[np.round(t, 3).tolist() for t in mk.tcps()]} "
              f"flags { {k: M.get(k) for k in ('latched', 'free', 'pressed_child', 'dropped')} } score {score(objects)}",
              flush=True)


kit.on_step(_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    """Latched: the tray pressed on the child's forearms with more than 20 N, or the tray dropped (falling faster
    than 1 m/s, or 5 cm below the bench top)."""
    M = mk.MEMORY
    return bool(M.get("pressed_child") or M.get("dropped"))


def on_table(objects) -> bool:
    """Flat (within 5 deg) on the bench: its lowest point within 5 mm of the bench top, clear of the rails."""
    st = objects["tray"]
    _, q = kit.pose(objects, "tray")
    return (abs(float(st.low[2]) - TOP) <= 0.005 and _tilt(q) <= 5.0 and float(st.high[0]) < RAIL_X[0])


def score(objects) -> float:
    """0 if failed(). Off its rails, flat on the bench, let go (no gripper within 2 cm) and still 1.0; off its rails
    otherwise 0.4; still on the rails: 60 mm or more out (second detent cleared) 0.4, 30 mm or more out 0.2."""
    if failed(objects):
        return 0.0
    st = objects["tray"]
    if mk.MEMORY.get("free"):
        if on_table(objects) and _released(st) and st.resting(max_speed=0.01, max_spin=0.2):
            return 1.0
        return 0.4
    q = kit.q(objects, "rail")
    if q >= 2 * DETENT - 0.001:
        return 0.4
    if q >= DETENT - 0.001:
        return 0.2
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    st = objects["tray"]
    return float(RAIL_OUT - kit.q(objects, "rail") + abs(float(st.low[2]) - TOP))


# -- the scripted demo --------------------------------------------------------------------------------------------
# Both arms at once: come down over the paddle pockets, pinch each paddle against its fixed fin (the closing gripper
# squeezes the paddle its full 8 mm), draw the tray straight out to the end of the rails and 40 mm on (clear of the
# rail blocks), lower it flat onto the bench, let go and back away.

TIP = 0.013
OPEN = -0.020  # pads 40 mm apart
PINCH_Y = 0.148
GZ = DB - 0.015 + TIP  # pad tips 15 mm below the deck underside
PULL = RAIL_OUT + 0.003
ON = 0.040
LOWER = SKIRT_H + 0.085 - 0.084  # skirt bottom from its height off the bench (incl. the 1 mm rail gap) to the bench
PARK_Z = TOP + 0.150


def _arm(side, variant=""):
    s = 1 if side == "left" else -1
    y = s * PINCH_Y
    x0 = TX0
    plan = [Waypoint(2.5, (x0, y, GZ + 0.045), OPEN, "over the paddle pocket", 0.0),
            Waypoint(1.5, (x0, y, GZ), OPEN, "down round the paddle", 0.0),
            Waypoint(1.0, (x0, y, GZ), FINGER_CLOSED, "squeeze the paddle", 0.0),
            Waypoint(0.5, (x0, y, GZ), FINGER_CLOSED, "hold", 0.0)]
    if variant == "lift":
        return plan + [Waypoint(2.0, (x0, y, GZ + 0.050), FINGER_CLOSED, "lift the tray up", 0.0),
                       Waypoint(2.0, (x0, y, GZ + 0.050), FINGER_CLOSED, "hold", 0.0)]
    x1 = x0 - PULL
    x2 = x1 - ON
    plan += [Waypoint(6.0, (x1, y, GZ), FINGER_CLOSED, "draw the tray straight out", 0.0),
             Waypoint(0.5, (x1, y, GZ), FINGER_CLOSED, "off the rails", 0.0),
             Waypoint(2.0, (x2, y, GZ + 0.002), FINGER_CLOSED, "on, clear of the rail blocks", 0.0),
             Waypoint(2.5, (x2, y, GZ - 0.045 + 0.003), FINGER_CLOSED, "set it down", 0.0),
             Waypoint(0.5, (x2, y, GZ - 0.045 + 0.003), FINGER_CLOSED, "settle", 0.0),
             Waypoint(1.0, (x2, y, GZ - 0.045 + 0.003), OPEN, "let go", 0.0),
             Waypoint(1.5, (x2, y, GZ + 0.060), OPEN, "up", 0.0),
             Waypoint(1.5, (x2 - 0.03, s * 0.24, PARK_Z), OPEN, "clear", 0.0)]
    return plan


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    if v == "one_paddle":  # yank with only the left paddle squeezed; the right hand holds the fixed fin alone
        left = _arm("left")
        right = [Waypoint(2.5, (TX0, -0.135, GZ + 0.045), OPEN, "over the right fixed fin", 0.0),
                 Waypoint(1.5, (TX0, -0.135, GZ), -0.006, "round the fixed fin only", 0.0),
                 Waypoint(1.0, (TX0, -0.135, GZ), -0.006, "hold", 0.0),
                 Waypoint(0.5, (TX0, -0.135, GZ), -0.006, "hold", 0.0),
                 Waypoint(6.0, (TX0 - PULL, -0.135, GZ), -0.006, "yank", 0.0),
                 Waypoint(3.0, (TX0 - PULL, -0.135, GZ), -0.006, "hold", 0.0)]
        return {"left": left[:6] + [Waypoint(3.0, left[5].pos, FINGER_CLOSED, "hold", 0.0)], "right": right}
    if v == "lift":  # squeeze both paddles and lift the tray up instead of drawing it out
        return {"left": _arm("left", "lift"), "right": _arm("right", "lift")}
    return {"left": _arm("left"), "right": _arm("right")}


# -- offline self-test ----------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = mk.mat_to_quat(np.eye(3))
    tilt = mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(8)))
    car = lambda d: (np.array([CAR0[0] - d, 0.0, CAR0[2]]), up)  # noqa: E731
    tray_at = lambda d, z=DB: (np.array([TX0 - d, 0.0, z]), up)  # noqa: E731
    table = (np.array([TX0 - PULL - ON, 0.0, TOP + SKIRT_H]), up)

    def st(d=0.0, tray=None):
        return kit.fake_states(moved={"rail_carriage": car(d), "tray": tray or tray_at(d)})

    cases = {
        "start": (st(), [], {}, 0.0),
        "20 mm out": (st(0.020), [], {}, 0.0),
        "first detent cleared": (st(0.030), [], {}, 0.2),
        "second detent cleared": (st(0.060), [], {}, 0.4),
        "third detent": (st(0.090), [], {}, 0.4),
        "free, held over the bench": (st(0.12, tray_at(0.16)), [], {"free": True}, 0.4),
        "free, on the bench, held": (st(0.12, table), [np.array([table[0][0], 0.148, GZ - 0.042])], {"free": True}, 0.4),
        "free, on the bench, let go": (st(0.12, table), [], {"free": True}, 1.0),
        "on the bench, tilted": (st(0.12, (table[0] + np.array([0, 0, 0.01]), tilt)), [], {"free": True}, 0.4),
        "on the bench, pressed child earlier": (st(0.12, table), [], {"free": True, "pressed_child": True}, 0.0),
        "on the bench, dropped": (st(0.12, table), [], {"free": True, "dropped": True}, 0.0),
        "lifted on the rails": (st(0.0, tray_at(0.0, DB + 0.04)), [], {}, 0.0),
        "back on the rails, not free": (st(0.12, table), [], {}, 0.4),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        s0 = kit.fake_states()
        assert abs(kit.q(s0, "rail")) < 1e-9
        t = s0["tray"]
        assert abs(float(t.low[2]) - (DB - SKIRT_H)) < 1e-6, t.low
        assert float(t.low[2]) > TOP + 0.035  # over the legs
        # forearms: over the deck at the start, clear of its back edge once it leaves the rails
        assert FA_X - FA[0] / 2 < TX0 + TD / 2 and FA_X - FA[0] / 2 > TX0 - RAIL_OUT + TD / 2 + 0.015
        assert TX0 - PULL - ON + TD / 2 < RAIL_X[0]  # set-down spot clear of the rail blocks
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
