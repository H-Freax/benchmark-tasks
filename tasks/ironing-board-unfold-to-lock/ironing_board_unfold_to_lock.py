"""A tabletop ironing board lies folded flat on the bench, its long axis across the robot (nose to the left). It
stands on a free base (two feet joined by a spine, two guide columns behind the board). The board rides up the
columns on a height rack; a ratchet pawl holds the rack at one of five notches (0 = folded, 1-4 working heights,
3 marked with a blue tag on the column). The pawl is lifted only while the release trigger is squeezed: a sprung
paddle behind the nose handle, squeezed toward the handle (about 10 N). With the trigger let go the rack is locked;
the board rides on the rack through the pawl's stiff spring, and pulling or pushing it more than 35 N against a locked
pawl strips the pawl (the ratchet stays disengaged and the board collapses).

The job: squeeze the trigger and lift both ends together to notch 3, let the trigger go so the pawl drops into notch
3, then let go of the board.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/ironing-board-unfold-to-lock/ironing_board_unfold_to_lock.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "Put the board up at its working height." --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

COVER = (0.80, 0.82, 0.86)  # board cover
STEEL = (0.45, 0.47, 0.50)
BLACK = (0.10, 0.10, 0.11)
RED = (0.85, 0.15, 0.12)
BLUE = (0.15, 0.35, 0.75)

# -- geometry (m) --------------------------------------------------------------------------------------------------
BX, BW, BL, BT = 0.270, 0.120, 0.450, 0.010  # board centre x, width (x), length (y), thickness
Z0 = TOP + 0.012  # board underside, folded
YN = BL / 2  # nose end (+y, left arm); tail end at -YN (right arm)
HB_T, HB_H, HB_W = 0.010, 0.020, 0.040  # end handles: thickness (y), height above the board top, width (x)
HB_TOP = Z0 + BT + HB_H
PAD_L, PAD_T, PAD_GAP = 0.028, 0.006, 0.003  # trigger paddle: length, thickness, gap to the handle when squeezed
PAD_Y = YN + HB_T + PAD_GAP + PAD_T / 2  # paddle centre, squeezed (vertical)
PAD_HINGE_Z = HB_TOP - PAD_L
TRIG_OPEN, TRIG_PRESSED = 32.0, 8.0  # trigger angle (deg) at rest / at or below which the pawl is lifted
COLX, COLY, COLH = 0.348, 0.150, 0.090  # guide columns
NOTCHES = (0.0, 0.015, 0.030, 0.045, 0.060)  # rack heights; 3 is the marked working height
TRAVEL = NOTCHES[-1]
WORK = 3
PAWL_K = 4000.0  # N/m, board on the rack through the pawl
STRIP_D = 0.009  # pawl spring deflection at ~36 N: the pawl strips

kit = mk.Kit()
kit.part("base", (BX, 0.0, TOP), [
    mk.box((0.140, 0.016, 0.010), at=(0, COLY, 0.005)),
    mk.box((0.140, 0.016, 0.010), at=(0, -COLY, 0.005)),
    mk.box((0.016, 2 * COLY, 0.010), at=(COLX - BX, 0, 0.005)),
    mk.box((0.016, 0.016, COLH), at=(COLX - BX, COLY, 0.010 + COLH / 2)),
    mk.box((0.016, 0.016, COLH), at=(COLX - BX, -COLY, 0.010 + COLH / 2)),
    mk.box((0.004, 0.024, 0.004), at=(COLX - BX - 0.010, COLY, Z0 - TOP + BT / 2 + NOTCHES[WORK]), color=BLUE),
], mass=0.8, color=STEEL)
kit.part("rack", (BX, 0.0, Z0), [
    mk.box((0.060, 0.016, 0.006), at=(0.045, COLY, -0.004)),
    mk.box((0.060, 0.016, 0.006), at=(0.045, -COLY, -0.004)),
], mass=0.3, color=STEEL)
kit.part("board", (BX, 0.0, Z0), [
    mk.box((BW, BL, BT), at=(0, 0, BT / 2), color=COVER),
    mk.box((HB_W, HB_T, HB_H + BT), at=(0, YN + HB_T / 2, (HB_H + BT) / 2), color=BLACK),
    mk.box((HB_W, HB_T, HB_H + BT), at=(0, -YN - HB_T / 2, (HB_H + BT) / 2), color=BLACK),
], mass=1.2, color=COVER)
kit.part("trigger", (BX, PAD_Y, PAD_HINGE_Z), [mk.box((HB_W, PAD_T, PAD_L), at=(0, 0, PAD_L / 2))], mass=0.03,
         color=RED)
kit.slider("rack_lift", "base", "rack", axis=(0, 0, 1), lower=0.0, upper=TRAVEL + 0.002, damping=2.0)
kit.slider("pawl", "rack", "board", axis=(0, 0, 1), lower=-0.012, upper=0.012, stiffness=PAWL_K, damping=40.0)
# spring torque 0.30 N*m/rad, preloaded to 36 deg: ~10 N squeeze at the fingertips' contact (19 mm from the pivot)
kit.hinge("trigger_hinge", "board", "trigger", anchor=(BX, PAD_Y, PAD_HINGE_Z), axis=(-1, 0, 0), lower_deg=0.0,
          upper_deg=TRIG_OPEN, start_deg=TRIG_OPEN, stiffness=0.30, rest_deg=36.0, damping=0.002)
for i, h in enumerate(NOTCHES):
    T = kit.parts["rack"].ref.copy()
    T[2, 3] += h
    kit.catch(f"notch{i}", "rack", T, parent="base", parent_T=kit.parts["base"].ref)

OBJECTS = [*kit.objects()]

# -- rules ---------------------------------------------------------------------------------------------------------
DT = 0.05
DROP_STEP = 0.006  # 60 mm per step at full size, 1:10 travel
ENGAGE_TOL = 0.0025
TIP_DEG = 25.0


def _disengage(name):
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _tilt(objects, part) -> float:
    z = mk.quat_to_mat(kit.pose(objects, part)[1])[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def pressed(objects) -> bool:
    return kit.q_deg(objects, "trigger_hinge") <= TRIG_PRESSED


def _ratchet_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    if m["n"] == 1:
        kit.engage("notch0")
        m["notch"] = 0
        return
    locked = m.get("notch") is not None
    if locked and abs(kit.q(objects, "pawl")) > STRIP_D:
        m["stripped"] = True
    if m.get("stripped") or pressed(objects):
        if locked:
            _disengage(f"notch{m['notch']}")
            m["notch"] = None
        return
    if not locked:
        h = kit.q(objects, "rack_lift")
        for i, nh in enumerate(NOTCHES):
            if abs(h - nh) <= ENGAGE_TOL:
                kit.engage(f"notch{i}")
                m["notch"] = i
                break


def _fail_rule(objects):
    m = mk.MEMORY
    z = float(kit.point(objects, "board", (0, 0, 0))[2])
    prev = m.get("z_prev")
    m["z_prev"] = z
    if prev is not None and m.get("notch") is None and prev - z > DROP_STEP:
        m["dropped"] = True
    if max(_tilt(objects, "board"), _tilt(objects, "base")) > TIP_DEG:
        m["tipped"] = True


def _debug_rule(objects):
    if os.environ.get("IRON_DEBUG") and mk.MEMORY.get("n", 0) % 20 == 0:
        print(f"[iron] n={mk.MEMORY.get('n')} rack={kit.q(objects, 'rack_lift'):.4f} pawl={kit.q(objects, 'pawl'):.4f}"
              f" trig={kit.q_deg(objects, 'trigger_hinge'):.1f} tilt={_tilt(objects, 'board'):.1f}"
              f" mem={ {k: v for k, v in mk.MEMORY.items() if k not in ('n', 'z_prev')} } score={score(objects)}",
              flush=True)


kit.on_step(_ratchet_rule)
kit.on_step(_fail_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring -------------------------------------------------------------------------------------------------------
_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def height(objects) -> float:
    """Board underside above its folded height, measured from the base's feet."""
    return float(kit.point(objects, "board", (0, 0, 0))[2] - kit.point(objects, "base", (0, 0, 0))[2]) - (Z0 - TOP)


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("dropped") or m.get("tipped") or m.get("stripped"))


def upright(objects) -> bool:
    return _tilt(objects, "board") <= 5.0 and _tilt(objects, "base") <= 5.0


def raised(objects) -> bool:
    return height(objects) > TRAVEL / 2 + 0.003


def locked_at_work(objects) -> bool:
    return (mk.MEMORY.get("notch") == WORK and not pressed(objects)
            and abs(height(objects) - NOTCHES[WORK]) <= 0.005)


def settled(objects) -> bool:
    return (all(_released(objects[n]) for n in ("board", "trigger"))
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("board", "base")))


def score(objects) -> float:
    """0 if dropped, tipped or the pawl stripped, or the board or base is tilted over 5 deg. 0.4 board raised more
    than half its travel; plus 0.6 the pawl in notch 3 with the trigger let go, both hands off and everything still."""
    if failed(objects) or not upright(objects) or not raised(objects):
        return 0.0
    return 1.0 if locked_at_work(objects) and settled(objects) else 0.4


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return abs(height(objects) - NOTCHES[WORK])


# -- scripted demo -------------------------------------------------------------------------------------------------
UP = 0.840
GZ = HB_TOP - 0.012 + 0.013  # TCP: pad tips 12 mm down the handles
NOSE_Y = (YN + PAD_Y + PAD_T / 2) / 2  # middle of handle + squeezed paddle
TAIL_Y = -YN - HB_T / 2
F_WIDE, F_SQUEEZE, F_OPEN, F_GRIP, F_NARROW = -0.033, 0.0, -0.020, 0.0, -0.008


class _Seq:
    def __init__(self):
        self.w, self.t, self.last = [], 0.0, None

    def go(self, d, pos, f, label):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, 0.0))
        self.t += d
        self.last = (tuple(pos), f)
        return self

    def until(self, t, label="wait"):
        pos, f = self.last
        return self.go(max(0.1, t - self.t), pos, f, label)


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    SAG = 0.013  # the arms sag about 13 mm under the board
    lift = {"": NOTCHES[WORK] + SAG, "notch1": NOTCHES[1] + SAG, "forced": 0.040, "on_end": 0.0}[variant]
    squeeze = variant not in ("forced", "on_end")
    if variant == "forced":  # the nose handle alone, fingers kept clear of the trigger
        hy = YN + HB_T / 2
        L.go(2.5, (BX, hy, UP), F_NARROW, "over the nose handle")
        L.go(1.5, (BX, hy, GZ), F_NARROW, "down round the handle only")
        L.go(1.0, (BX, hy, GZ), F_GRIP, "grip it")
    else:
        L.go(2.5, (BX, NOSE_Y, UP), F_WIDE, "over the nose handle")
        L.go(1.5, (BX, NOSE_Y, GZ), F_WIDE, "down round handle and trigger")
        L.go(1.0, (BX, NOSE_Y, GZ), F_SQUEEZE if squeeze else F_WIDE, "squeeze the trigger")
    R.go(2.5, (BX, TAIL_Y, UP), F_OPEN, "over the tail handle")
    R.go(1.5, (BX, TAIL_Y, GZ), F_OPEN, "down round it")
    R.go(1.0, (BX, TAIL_Y, GZ), F_GRIP, "grip it")
    if variant == "on_end":
        L.go(1.0, (BX, NOSE_Y, UP), F_WIDE, "hand off")
        R.go(5.0, (BX, TAIL_Y + 0.06, GZ + 0.10), F_GRIP, "swing the tail up, the board onto its nose")
        R.go(1.0, (BX, TAIL_Y + 0.06, GZ + 0.10), F_OPEN, "let go")
        R.go(1.5, (0.20, -0.30, UP), F_OPEN, "park")
        L.go(1.5, (0.20, 0.30, UP), F_WIDE, "park")
        L.until(R.t + 5.0)
        return {"left": L.w, "right": R.w}
    fl = F_SQUEEZE if squeeze else F_GRIP
    t0 = max(L.t, R.t) + 0.5
    L.until(t0)
    R.until(t0)
    L.go(5.0, (BX, L.last[0][1], GZ + lift), fl, "lift both ends")
    R.go(5.0, (BX, TAIL_Y, GZ + lift), F_GRIP, "lift both ends")
    L.go(1.0, (BX, L.last[0][1], GZ + lift), fl, "hold")
    R.go(1.0, (BX, TAIL_Y, GZ + lift), F_GRIP, "hold")
    L.go(1.0, (BX, L.last[0][1], GZ + lift), F_WIDE, "let the trigger go")
    L.go(1.0, (BX, L.last[0][1], GZ + lift), F_WIDE, "pawl drops in")
    L.go(1.5, (BX, L.last[0][1], UP + 0.01), F_WIDE, "up")
    R.until(L.t - 1.5)
    R.go(1.0, (BX, TAIL_Y, GZ + lift), F_OPEN, "let go")
    R.go(1.5, (BX, TAIL_Y, UP + 0.01), F_OPEN, "up")
    L.go(1.5, (0.20, 0.30, UP), F_WIDE, "park")
    R.go(1.5, (0.20, -0.30, UP), F_OPEN, "park")
    L.until(R.t + 4.0)
    return {"left": L.w, "right": R.w}


VARIANTS = ("", "notch1", "forced", "on_end")


def demo():
    # DEMO_VARIANT: notch1 | forced | on_end (degenerate checks; all score 0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ---------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS

    def st(h, pawl=-0.003, trig=TRIG_OPEN, **moved):
        return kit.fake_states(q={"rack_lift": h, "pawl": pawl, "trigger_hinge": math.radians(trig)}, moved=moved)

    tilt = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(8)))
    work = st(NOTCHES[WORK])
    hand = [kit.point(work, "board", (0, YN, 0.03))]
    cases = {
        "folded": (st(0.0), [], {"notch": 0}, 0.0),
        "notch 1": (st(NOTCHES[1]), [], {"notch": 1}, 0.0),
        "notch 2": (st(NOTCHES[2]), [], {"notch": 2}, 0.0),
        "notch 3 locked, released": (work, [], {"notch": 3}, 1.0),
        "notch 3 locked, still held": (work, hand, {"notch": 3}, 0.4),
        "notch 3, trigger squeezed": (st(NOTCHES[WORK], trig=2.0), hand, {"notch": None}, 0.4),
        "notch 4 locked": (st(NOTCHES[4]), [], {"notch": 4}, 0.4),
        "notch 3, stripped": (work, [], {"notch": None, "stripped": True}, 0.0),
        "notch 3, dropped earlier": (work, [], {"notch": 3, "dropped": True}, 0.0),
        "tipped": (work, [], {"notch": 3, "tipped": True}, 0.0),
        "base tilted 8 deg": (kit.fake_states(q={"rack_lift": NOTCHES[WORK], "pawl": -0.003,
                                                  "trigger_hinge": math.radians(TRIG_OPEN)},
                                               moved={"base": (np.array([BX, 0, TOP]), tilt)}), [], {"notch": 3}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
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
    for v in VARIANTS:
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
