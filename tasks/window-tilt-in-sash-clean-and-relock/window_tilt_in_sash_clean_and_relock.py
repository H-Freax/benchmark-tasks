"""A small double-hung tilt window unit lying on its back on the bench, its room side up and its sill end away from
the robot (a bench-height mock-up: the arms cannot reach the top of a 0.25 m sash standing upright). The lower sash
(0.36 m wide, 0.25 m from sill to top rail, 1.6 kg) runs in the jamb tracks on friction balance shoes and is raised
3 cm (drawn 3 cm toward the robot). Two tilt latches sit in its top rail; each is a spring-return bolt (6 N at full
retraction) whose 10 mm nose runs in the jamb track under the front stop. While a bolt is out, the stop holds the
top of the sash in the frame; with both drawn in, the top tips in toward the room (up) about the pivot bars at the
bottom of the sash. A thumb-turn sash lock sits on the middle of the top rail; turned 90 deg it hooks the keeper on
the upper sash's meeting rail.

The job, as a housekeeper does it to wipe the outside of the glass: draw both latches in together, tip the sash in
past 30 deg, tip it back into the frame so both latches snap into the tracks, lower it to the sill and turn the
lock shut.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/window-tilt-in-sash-clean-and-relock/window_tilt_in_sash_clean_and_relock.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1400 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
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

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
# Window frame: the sash's "up" (sill to head) is -x here, its room side +z.
WHITE = (0.93, 0.93, 0.91)  # vinyl frame and sash
GLASS = (0.62, 0.78, 0.86)
GREY_C = (0.55, 0.57, 0.60)
BRASS = (0.78, 0.62, 0.25)
BLUE = (0.15, 0.35, 0.75)

SASH_W, SASH_H, SASH_T = 0.360, 0.250, 0.030
RAIL = 0.030  # stile and rail width
ZC = TOP + 0.035  # sash mid-plane (faces at ZC -+ 0.015)
FACE = ZC + SASH_T / 2  # room-side face
X_SILL = 0.405  # the sash's bottom edge when down on the sill
X_HEAD_DOWN = X_SILL - SASH_H  # its top edge when down
RAISE0 = 0.030  # raised 3 cm at the start
PIVOT_D = 0.015  # pivot bars: on the mid-plane, 15 mm in from the bottom edge
TAB_D = SASH_H - PIVOT_D - RAIL / 2  # pivot to the tabs (on the top rail's centre line)
TOP_R = SASH_H - PIVOT_D  # pivot to the top edge
LATCH_Y = 0.150  # tab centres at y = +-0.15 (0.30 m apart)
BOLT_OUT = 0.008  # bolt nose past the stile when out
BOLT_H = 0.010
LATCH_TRAVEL = 0.012
LATCH_K = 500.0  # N/m: 6 N at full retraction
STOP_IN = SASH_W / 2 + 0.003  # inner edge of the front stops
STOP_Z = (ZC + BOLT_H / 2 + 0.002, ZC + BOLT_H / 2 + 0.027)  # front stop over the bolt track
SASH_KG = 1.6

kit = mk.Kit()
# Pivot shoe: the balance shoe in the jamb track (one body standing for both), on the sash's slide.
PIV = (X_SILL - PIVOT_D, 0.0, ZC)
kit.part("shoe", PIV, [mk.box((0.010, 0.010, 0.010), color=GREY_C)], mass=0.3, color=GREY_C)
# Balance shoes: the sash stays where it is put (25 N of track friction).
kit.slider("lift", None, "shoe", axis=(-1, 0, 0), lower=0.0, upper=0.060, start=RAISE0, damping=30.0, friction=25.0)

_cx = -SASH_H / 2
_sash = [
    mk.box((SASH_H, RAIL, SASH_T), at=(_cx, SASH_W / 2 - RAIL / 2, 0)),  # stiles
    mk.box((SASH_H, RAIL, SASH_T), at=(_cx, -SASH_W / 2 + RAIL / 2, 0)),
    mk.box((RAIL, SASH_W - 2 * RAIL, SASH_T), at=(-RAIL / 2, 0, 0)),  # bottom rail
    mk.box((RAIL, SASH_W - 2 * RAIL, SASH_T), at=(-SASH_H + RAIL / 2, 0, 0)),  # top (meeting) rail
    mk.box((SASH_H - 2 * RAIL, SASH_W - 2 * RAIL, 0.004), at=(_cx, 0, 0), color=GLASS),
]
kit.part("sash", (X_SILL, 0.0, ZC), _sash, mass=SASH_KG, color=WHITE)
# Tilt hinge: positive = top tipped in toward the room (up). 0 is the sash back in the frame, against its stops.
kit.hinge("tilt", "shoe", "sash", anchor=PIV, axis=(0, 1, 0), lower_deg=0.0, upper_deg=85.0, damping=0.4)

X_TAB_DOWN = X_SILL - PIVOT_D - TAB_D
# Tilt latches: bolt (in the top rail, nose 10 mm past the stile) and finger tab (18 mm above the face).
for side, s in (("l", 1), ("r", -1)):
    kit.part(f"latch_{side}", (X_TAB_DOWN, s * LATCH_Y, ZC),
             [mk.box((0.012, 0.050, BOLT_H), at=(0, s * (SASH_W / 2 + BOLT_OUT - 0.025 - LATCH_Y), 0), color=GREY_C),
              mk.box((0.012, 0.008, 0.020), at=(0, 0, SASH_T / 2 + 0.008), color=BLUE)],
             mass=0.05, color=GREY_C)
    kit.slider(f"latch_{side}_pull", "sash", f"latch_{side}", axis=(0, -s, 0), lower=0.0, upper=LATCH_TRAVEL,
               stiffness=LATCH_K, rest=0.0, damping=2.0)

# Thumb-turn sash lock on the middle of the top rail: a 40 mm turn bar, open at +45 deg, shut at -45 deg.
LOCK_AT = (X_TAB_DOWN, 0.0, FACE)
kit.part("lock", LOCK_AT, [mk.box((0.018, 0.018, 0.006), at=(0, 0, 0.003), color=BRASS),
                           mk.box((0.040, 0.006, 0.012), at=(0, 0, 0.012), color=BRASS, yaw=math.radians(45)),
                           mk.box((0.012, 0.004, 0.001), at=(0.010 * math.cos(math.radians(45)),
                                                             0.010 * math.sin(math.radians(45)), 0.0185),
                                  color=(0.8, 0.1, 0.1), yaw=math.radians(45))],
         mass=0.03, color=BRASS)
kit.hinge("lock_turn", "sash", "lock", anchor=LOCK_AT, axis=(0, 0, 1), lower_deg=-95.0, upper_deg=5.0,
          stiffness=0.05, rest_deg=0.0, damping=0.003)
LOCK_SHUT_DEG = -80.0

FX0, FX1 = X_HEAD_DOWN - 0.090, X_SILL + 0.025  # frame extent in x (head end cut short)
FIXTURES = [
    Box("sill", pos=(X_SILL + 0.0125, 0.0, TOP + 0.035), size=(0.025, 0.450, 0.070), color=WHITE),
    Box("jamb_l", pos=((FX0 + FX1) / 2, 0.215, TOP + 0.035), size=(FX1 - FX0, 0.020, 0.070), color=WHITE),
    Box("jamb_r", pos=((FX0 + FX1) / 2, -0.215, TOP + 0.035), size=(FX1 - FX0, 0.020, 0.070), color=WHITE),
]
for s, nm in ((1, "l"), (-1, "r")):
    wy = 0.205 - STOP_IN
    FIXTURES += [
        Box(f"front_stop_{nm}", pos=((FX0 + X_SILL) / 2, s * (STOP_IN + wy / 2), (STOP_Z[0] + STOP_Z[1]) / 2),
            size=(X_SILL - FX0, wy, STOP_Z[1] - STOP_Z[0]), color=WHITE),
        Box(f"track_{nm}", pos=((FX0 + X_SILL) / 2, s * (STOP_IN + wy / 2), TOP + 0.012),
            size=(X_SILL - FX0, wy, 0.024), color=WHITE),
    ]
# Upper sash (fixed, in the outer track: under the lower sash here): its meeting rail and a strip of its glass.
UP_X1 = X_HEAD_DOWN + 0.040
FIXTURES += [
    Box("upper_meeting_rail", pos=((FX0 + UP_X1) / 2, 0.0, TOP + 0.009), size=(UP_X1 - FX0, SASH_W, 0.018),
        color=WHITE),
    Box("keeper", pos=(X_HEAD_DOWN - 0.006, 0.0, TOP + 0.018 + 0.001), size=(0.012, 0.030, 0.002), color=BRASS),
]
OBJECTS = [*kit.objects(), *FIXTURES]

TILT_SCORE_DEG = 30.0
UPRIGHT_DEG = 1.5
SNAPPED_M = 0.0015  # a latch counts as snapped when drawn in less than this
DOWN_M = 0.002
FALL_DEG = 80.0
SLAM_MPS = 0.5


def tilt_deg(objects) -> float:
    """How far the top of the sash is tipped up out of the frame (deg), from the sash's own pose."""
    d = kit._pose_T(objects, "sash")[:3, :3] @ np.array([-1.0, 0.0, 0.0])
    return math.degrees(math.atan2(float(d[2]), float(-d[0])))


def lift_m(objects) -> float:
    """How far the sash's pivot bars sit up the tracks from the sill position (m)."""
    return float(X_SILL - PIVOT_D - kit.point(objects, "sash", (-PIVOT_D, 0.0, 0.0))[0])


def latch_in(objects, side) -> float:
    """How far latch ``side`` ('l' / 'r') is drawn in, measured in the sash's frame (m)."""
    s = 1 if side == "l" else -1
    local = mk._inv(kit._pose_T(objects, "sash")) @ np.append(kit.point(objects, f"latch_{side}", (0, 0, 0)), 1.0)
    return float(LATCH_Y - s * local[1])


def _mechanism_rule(objects):
    """Lock detents, the tilt record and the failure latches."""
    M = mk.MEMORY
    lk = kit.q_deg(objects, "lock_turn")
    det = -90.0 if lk < -45.0 else 0.0
    if M.get("detent") != det:
        M["detent"] = det
        kit.drive("lock_turn", det)
    t = tilt_deg(objects)
    M["max_tilt"] = max(M.get("max_tilt", 0.0), t)
    prev = M.get("prev_tilt", t)
    M["prev_tilt"] = t
    if t > FALL_DEG:
        M["fell"] = True
    w = float(np.linalg.norm(objects["sash"].ang_vel))
    if t < 5.0 and t < prev - 1e-4 and w * TOP_R > SLAM_MPS:
        M["slam"] = True
    if os.environ.get("WINDOW_DEBUG") and mk._EP["step"] % 20 == 0:
        print(f"[window] step {mk._EP['step']} tilt {t:.1f} lift {lift_m(objects) * 1000:.1f} mm "
              f"latch {latch_in(objects, 'l') * 1000:.1f}/{latch_in(objects, 'r') * 1000:.1f} mm "
              f"lock {lk:.0f} w {w:.2f} flags { {k: v for k, v in M.items() if k in ('max_tilt', 'fell', 'slam')} }"
              f" score {score(objects)}", flush=True)


kit.on_step(_mechanism_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    """Latched: the sash dropped open past 80 deg (the pivot bars jump the shoes), or its top came back against the
    stops faster than 0.5 m/s."""
    return bool(mk.MEMORY.get("fell") or mk.MEMORY.get("slam"))


def latches_snapped(objects) -> bool:
    return latch_in(objects, "l") < SNAPPED_M and latch_in(objects, "r") < SNAPPED_M


def shut_and_locked(objects) -> bool:
    """Upright (within 1.5 deg), down on the sill (within 2 mm), lock turned past 80 deg, nothing within 2 cm of the
    sash, the sash and lock still."""
    sash = objects["sash"]
    return (tilt_deg(objects) <= UPRIGHT_DEG and lift_m(objects) <= DOWN_M
            and kit.q_deg(objects, "lock_turn") <= LOCK_SHUT_DEG and _released(sash)
            and sash.resting(max_speed=0.01, max_spin=0.2) and objects["lock"].resting(max_speed=0.01, max_spin=0.3))


def score(objects) -> float:
    """0 if failed(), if the sash was never tipped in past 30 deg, or unless it ends shut and locked. Then: tipped in
    past 30 deg 0.4; both latches snapped out 0.3; locked down 0.3."""
    if failed(objects) or mk.MEMORY.get("max_tilt", 0.0) <= TILT_SCORE_DEG or not shut_and_locked(objects):
        return 0.0
    s = 0.4 + 0.3
    if latches_snapped(objects):
        s += 0.3
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(max(0.0, TILT_SCORE_DEG - mk.MEMORY.get("max_tilt", 0.0)) / 100.0 + tilt_deg(objects) / 100.0
                 + lift_m(objects) + max(0.0, kit.q_deg(objects, "lock_turn") - LOCK_SHUT_DEG) / 100.0)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Both arms pinch the latch tabs and draw them in together (10 mm), tip the sash in to 36 deg along its arc, hold it
# there, tip it back upright, let the latches out, push the sash down to the sill and let go. The left arm then
# turns the lock from +45 to -45 deg while the right arm stands clear.

TIP = 0.013
OPEN = -0.012
KNOB_OPEN = -0.030
TILT_TO = 36.0
GRIP_H = SASH_T / 2 + 0.004 + TIP  # TCP height over the mid-plane while pinching a tab (pad tips 4 mm over the face)
PULL = 0.017  # 12 mm of latch travel; the rest presses the latch against its stop


def _arc(y, theta_deg, raise_=RAISE0):
    th = math.radians(theta_deg)
    px = X_SILL - PIVOT_D - raise_
    return (px - TAB_D * math.cos(th) + GRIP_H * math.sin(th), y, ZC + TAB_D * math.sin(th) + GRIP_H * math.cos(th))


def _dur(plan):
    return sum(w.duration for w in plan)


def _wait(plan, until, label="wait"):
    t = _dur(plan)
    if until > t:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))
    return plan


def _arm(s, pull=True, tilt_to=TILT_TO, back=True, lower=True, park_y=None):
    y0 = s * LATCH_Y
    yin = y0 - s * PULL if pull else y0
    a0 = _arc(y0, 0.0)
    over = (a0[0], y0, a0[2] + 0.03)
    plan = [Waypoint(1.5, (0.16, y0, 0.86), OPEN, "lift the hand", 0.0),
            Waypoint(1.5, over, OPEN, "over the latch tab", 0.0),
            Waypoint(1.5, a0, OPEN, "down round the tab", 0.0),
            Waypoint(0.8, a0, FINGER_CLOSED, "pinch the tab", 0.0),
            Waypoint(1.2, (a0[0], yin, a0[2]), FINGER_CLOSED, "draw the latch in", 0.0),
            Waypoint(1.0, (a0[0], yin, a0[2]), FINGER_CLOSED, "hold", 0.0)]
    th = 0.0
    while th < tilt_to - 1e-6:
        th = min(tilt_to, th + 4.0)
        plan.append(Waypoint(0.45, _arc(yin, th), FINGER_CLOSED, "tip the sash in", 0.0))
    plan.append(Waypoint(1.5, _arc(yin, th), FINGER_CLOSED, "hold it in the cleaning position", 0.0))
    if not back:
        return plan + [Waypoint(12.0, _arc(yin, th), FINGER_CLOSED, "keep it tipped in", 0.0)]
    while th > 1e-6:
        th = max(0.0, th - 4.0)
        plan.append(Waypoint(0.9, _arc(yin, th), FINGER_CLOSED, "tip it back into the frame", 0.0))
    a = _arc(yin, 0.0)
    plan += [Waypoint(0.8, (a[0], yin, a[2]), FINGER_CLOSED, "hold it against the stop", 0.0),
             Waypoint(1.0, (a[0], y0, a[2]), FINGER_CLOSED, "let the latch out", 0.0),
             Waypoint(0.4, (a[0], y0, a[2]), FINGER_CLOSED, "settle", 0.0)]
    if lower:
        d = _arc(y0, 0.0, raise_=-0.004)
        plan += [Waypoint(2.0, d, FINGER_CLOSED, "push the sash down to the sill", 0.0),
                 Waypoint(0.5, d, FINGER_CLOSED, "settle", 0.0)]
    end = plan[-1].pos
    return plan + [Waypoint(0.8, end, OPEN, "let go", 0.0),
                   Waypoint(1.2, (end[0], end[1], end[2] + 0.05), OPEN, "up off the tab", 0.0),
                   Waypoint(1.5, (0.16, park_y if park_y is not None else s * 0.24, 0.88), OPEN, "clear", 0.0)]


def _lock(plan, start):
    c = (X_TAB_DOWN, 0.0, FACE + 0.006 + TIP)
    over = (c[0], c[1], c[2] + 0.04)
    y0, y1 = math.radians(45.0), math.radians(-47.0)
    plan = _wait(plan, start)
    return plan + [Waypoint(2.0, over, KNOB_OPEN, "over the lock", y0),
                   Waypoint(1.5, c, KNOB_OPEN, "down round the turn bar", y0),
                   Waypoint(0.8, c, FINGER_CLOSED, "grip the turn bar", y0),
                   Waypoint(2.0, c, FINGER_CLOSED, "turn the lock shut", y1),
                   Waypoint(0.6, c, KNOB_OPEN, "let go", y1),
                   Waypoint(1.2, over, KNOB_OPEN, "up off the lock", 0.0),
                   Waypoint(1.5, (0.16, 0.24, 0.88), OPEN, "clear", 0.0),
                   Waypoint(3.0, (0.16, 0.24, 0.88), OPEN, "wait", 0.0)]


def _plans(variant=""):
    if variant == "one_latch":  # only the left latch drawn in; both arms try to tip the sash in, then lock it
        left, right = _arm(1, back=True), _arm(-1, pull=False, back=True)
    elif variant == "leave_tilted":
        left, right = _arm(1, back=False), _arm(-1, back=False)
    else:
        left, right = _arm(1), _arm(-1)
    if variant != "leave_tilted" and variant != "no_lock":
        t = max(_dur(left), _dur(right)) + 0.3
        left = _lock(left, t)
    right = _wait(right, max(_dur(left), _dur(right)))
    left = _wait(left, max(_dur(left), _dur(right)))
    return {"left": left, "right": right}


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    down = {"lift": 0.0, "tilt": 0.0, "lock_turn": -90.0, "latch_l_pull": 0.0, "latch_r_pull": 0.0}
    cases = {  # name: (joint values, memory, tcps, score)
        "start": ({"lift": RAISE0, "tilt": 0.0, "lock_turn": 0.0}, {}, [], 0.0),
        "locked without tipping": (down, {"max_tilt": 0.0}, [], 0.0),
        "locked after 25 deg": (down, {"max_tilt": 25.0}, [], 0.0),
        "full": (down, {"max_tilt": 36.0}, [], 1.0),
        "full, one latch held in": ({**down, "latch_l_pull": 0.008}, {"max_tilt": 36.0}, [], 0.7),
        "left tilted 36 deg": ({**down, "lift": RAISE0, "tilt": 36.0, "lock_turn": 0.0}, {"max_tilt": 36.0}, [], 0.0),
        "shut but unlocked": ({**down, "lock_turn": 0.0}, {"max_tilt": 36.0}, [], 0.0),
        "locked but raised 3 cm": ({**down, "lift": RAISE0}, {"max_tilt": 36.0}, [], 0.0),
        "locked, 3 deg in": ({**down, "tilt": 3.0}, {"max_tilt": 36.0}, [], 0.0),
        "full, hand on the lock": (down, {"max_tilt": 36.0}, [np.array(LOCK_AT) + [0, 0, 0.02]], 0.0),
        "full but slammed": (down, {"max_tilt": 36.0, "slam": True}, [], 0.0),
        "full but dropped open": (down, {"max_tilt": 36.0, "fell": True}, [], 0.0),
    }
    try:
        for name, (q, mem, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            q = {k: (math.radians(v) if k in ("tilt", "lock_turn") else v) for k, v in q.items()}
            got = score(kit.fake_states(q=q))
            assert abs(got - want) < 1e-9, (name, got, want)
        st = kit.fake_states(q={"lift": RAISE0})
        assert abs(lift_m(st) - RAISE0) < 1e-9
        st = kit.fake_states(q={"lift": RAISE0, "tilt": math.radians(20), "latch_l_pull": 0.004, "latch_r_pull": 0.009})
        assert abs(tilt_deg(st) - 20) < 1e-6 and abs(latch_in(st, "l") - 0.004) < 1e-9 and abs(latch_in(st, "r") - 0.009) < 1e-9
        # the bolt nose sits behind the front stop when out, clear of it when drawn in 8 mm
        nose = SASH_W / 2 + BOLT_OUT
        assert nose > STOP_IN + 0.004 and nose - 0.006 < STOP_IN
        # the tilt the sash needs before the bolts' back faces pass the front stop
        clear = next(d for d in np.arange(0, 30, 0.1)
                     if TAB_D * math.sin(math.radians(d)) - BOLT_H / 2 * math.cos(math.radians(d)) > STOP_Z[1] - ZC)
        assert 8.0 < clear < 12.0, clear
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
    for v in ("", "leave_tilted", "no_lock", "one_latch"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm")
    os.environ.pop("DEMO_VARIANT")
