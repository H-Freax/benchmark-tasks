"""On the bench is the top-front section of a front-loading washing machine at full size: a 60 cm wide panel, 11 cm
tall and 34 cm deep, with the detergent dispenser drawer at its top left and the control panel to its right. The
drawer is shut. It runs on a slide out to a 15 cm stop, with drag from its runners, and is divided across its width
into three compartments: pre-wash on the left (marked I), main wash in the middle (II, the widest) and softener on
the right (with a blue siphon cap). Two compressed powder detergent tablets lie on the bench to the left of the
machine. The job is to dose this (normal, no pre-wash) cycle: both tablets in the main-wash compartment and the
drawer shut again, so the wash water flushes them in.

A tablet in the pre-wash compartment is flushed out with the first rinse water and the main wash runs without
detergent; one in the softener compartment never dissolves properly and can block the siphon. Scored from the
drawer's slide travel and the tablets' poses.

Mechanism parts come from mechkit.py (in this folder), which adds the slide to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/detergent-drawer-main-wash/detergent_drawer_main_wash.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (slide and per-episode hooks, installed in memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
WHITE = (0.93, 0.93, 0.92)
PANEL_GREY = (0.80, 0.81, 0.82)
DARK = (0.18, 0.19, 0.21)
BLUE = (0.25, 0.45, 0.80)
TABLET_COLOR = (0.95, 0.95, 0.97)

# Machine section (fixed): front face at x FRONT, back at x BACK, y from -0.15 to 0.45 (60 cm), z TOP to TOP+0.11.
FRONT, BACK = 0.380, 0.720
Y_LO, Y_HI = -0.150, 0.450
H = 0.110
PLINTH = 0.015  # the drawer runs on this
COVER_Z0 = TOP + 0.080  # underside of the worktop over the drawer
CAV_Y = (0.068, 0.272)  # drawer cavity, y

# Drawer (one moving part, reference pose = shut). Body x 0.382..0.682, y 0.072..0.268, floor bottom at TOP+0.017.
D_X0, D_X1 = 0.382, 0.682
D_Y0, D_Y1 = 0.072, 0.268
D_Z0 = TOP + PLINTH + 0.002
WALL, FLOOR_T, D_H = 0.004, 0.004, 0.060
FLOOR_TOP = D_Z0 + FLOOR_T
DIV = 0.003
# compartments (y ranges, inside the walls): pre-wash left, main middle, softener right
COMP = {
    "prewash": (0.204, D_Y1 - WALL),
    "main": (0.111, 0.201),
    "softener": (D_Y0 + WALL, 0.108),
}
COMP_X = (D_X0 + WALL, D_X1 - WALL)
FASCIA_X = (0.365, 0.380)
TAB_X = (0.335, 0.365)  # pull tab under the fascia's top edge
TAB_Y, TAB_W = 0.170, 0.012
TAB_Z = (TOP + 0.052, TOP + 0.082)
TRAVEL_MAX = 0.150  # slide stop
DRAWER_KG = 0.45
SLIDE_FRICTION = 2.0  # N, runner drag
SLIDE_DAMPING = 8.0  # N*s/m


def _drawer_boxes():
    """Drawer in its own frame: origin at the shut drawer's front-bottom-left corner of the body (D_X0, D_Y0, D_Z0)."""
    lx, ly = D_X1 - D_X0, D_Y1 - D_Y0
    b = [
        mk.box((lx, ly, FLOOR_T), at=(lx / 2, ly / 2, FLOOR_T / 2)),
        mk.box((WALL, ly, D_H), at=(WALL / 2, ly / 2, D_H / 2)),  # front wall
        mk.box((WALL, ly, D_H), at=(lx - WALL / 2, ly / 2, D_H / 2)),  # back wall
        mk.box((lx, WALL, D_H), at=(lx / 2, WALL / 2, D_H / 2)),  # right wall
        mk.box((lx, WALL, D_H), at=(lx / 2, ly - WALL / 2, D_H / 2)),  # left wall
    ]
    for y0 in (0.108, 0.201):  # dividers
        b.append(mk.box((lx - 2 * WALL, DIV, D_H - 0.006), at=(lx / 2, y0 + DIV / 2 - D_Y0, (D_H - 0.006) / 2)))
    # siphon cap in the softener compartment, at the back
    b.append(mk.box((0.040, 0.026, 0.030), at=(lx - 0.060, (0.090 - D_Y0), 0.015 + FLOOR_T), color=BLUE))
    # fascia (the drawer front the user sees) and the pull tab under its top edge
    fz0, fz1 = TOP + 0.012 - D_Z0, TOP + 0.090 - D_Z0
    b.append(mk.box((FASCIA_X[1] - FASCIA_X[0], 0.212, fz1 - fz0),
                    at=((FASCIA_X[0] + FASCIA_X[1]) / 2 - D_X0, 0.170 - D_Y0, (fz0 + fz1) / 2), color=WHITE))
    b.append(mk.box((TAB_X[1] - TAB_X[0], TAB_W, TAB_Z[1] - TAB_Z[0]),
                    at=((TAB_X[0] + TAB_X[1]) / 2 - D_X0, TAB_Y - D_Y0, (TAB_Z[0] + TAB_Z[1]) / 2 - D_Z0),
                    color=PANEL_GREY))
    # compartment marks on the fascia's top edge: I (pre-wash), II (main), flower (softener)
    for yc, w in ((0.234, 0.004), (0.152, 0.004), (0.160, 0.004), (0.090, 0.014)):
        b.append(mk.box((0.002, w, 0.012), at=(FASCIA_X[0] - 0.001 - D_X0, yc - D_Y0, fz1 - 0.012), color=DARK))
    return b


TABLET = (0.045, 0.032, 0.018)
TABLET_KG = 0.040
TABLET_XY = {"tablet_a": (0.230, 0.370), "tablet_b": (0.310, 0.370)}
TABLETS = tuple(TABLET_XY)

kit = mk.Kit()
kit.part("drawer", (D_X0, D_Y0, D_Z0), _drawer_boxes(), mass=DRAWER_KG, color=PANEL_GREY)
kit.slider("slide", None, "drawer", axis=(-1, 0, 0), lower=0.0, upper=TRAVEL_MAX, start=0.0,
           damping=SLIDE_DAMPING, friction=SLIDE_FRICTION)
for _n, (_x, _y) in TABLET_XY.items():
    kit.part(_n, (_x, _y, TOP + TABLET[2] / 2), [mk.box(TABLET)], mass=TABLET_KG, color=TABLET_COLOR)


def _box(name, x0, x1, y0, y1, z0, z1, color=WHITE):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0), color=color)


FIXTURES = [
    _box("plinth", FRONT, BACK, Y_LO, Y_HI, TOP, TOP + PLINTH),
    _box("worktop", FRONT, BACK, Y_LO, Y_HI, COVER_Z0, TOP + H),
    _box("body_left", FRONT, BACK, CAV_Y[1], Y_HI, TOP + PLINTH, COVER_Z0),
    _box("body_right", FRONT, BACK, Y_LO, CAV_Y[0], TOP + PLINTH, COVER_Z0),
    _box("back", D_X1 + 0.004, BACK, CAV_Y[0], CAV_Y[1], TOP + PLINTH, COVER_Z0),
    _box("display", FRONT - 0.002, FRONT, -0.090, 0.010, TOP + 0.035, TOP + 0.075, color=DARK),
    _box("dial", FRONT - 0.020, FRONT, 0.020, 0.055, TOP + 0.038, TOP + 0.073, color=PANEL_GREY),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

SHUT_M = 0.005  # shut: the drawer within 5 mm of fully in
ON_FLOOR_M = 0.006  # a tablet's lowest point within 6 mm of the drawer floor (a tablet is 18 mm thick)
_TEST_TCPS = None


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _compartment(objects, t):
    """Which compartment tablet ``t`` lies in (wholly inside its walls, on the drawer floor), or None. The drawer
    only translates, so the compartments' world x is their shut x minus the slide travel."""
    st = objects[t]
    lo, hi = np.asarray(st.low, float), np.asarray(st.high, float)
    d = kit.q(objects, "slide")
    if not (COMP_X[0] - d <= lo[0] and hi[0] <= COMP_X[1] - d) or lo[2] > FLOOR_TOP + ON_FLOOR_M:
        return None
    for name, (y0, y1) in COMP.items():
        if y0 <= lo[1] and hi[1] <= y1:
            return name
    return None


def _wrong(objects) -> bool:
    """Latched: a tablet let go and resting in the pre-wash or softener compartment at any step."""
    if mk.MEMORY.get("wrong_compartment"):
        return True
    for t in TABLETS:
        if _compartment(objects, t) in ("prewash", "softener") and objects[t].resting() and _released(objects[t]):
            mk.MEMORY["wrong_compartment"] = True
    return bool(mk.MEMORY.get("wrong_compartment"))


kit.on_step(_wrong)


def _shut(objects) -> bool:
    return kit.q(objects, "slide") <= SHUT_M


def _in_main(objects, t) -> bool:
    st = objects[t]
    return _compartment(objects, t) == "main" and st.resting() and _released(st)


def score(objects) -> float:
    """0 if a tablet was ever left in the pre-wash or softener compartment (latched). Otherwise, by tablets in the
    main-wash compartment (on its floor, let go, still) and the drawer: both in, shut 1.0, open 0.6; one in, shut
    0.5, open 0.3; none 0."""
    if _wrong(objects):
        return 0.0
    n = sum(_in_main(objects, t) for t in TABLETS)
    shut = _shut(objects)
    return {2: (1.0 if shut else 0.6), 1: (0.5 if shut else 0.3)}.get(n, 0.0)


def success(objects) -> bool:
    """Both tablets in the main-wash compartment and the drawer shut; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from each tablet to the middle of the main-wash compartment, plus the slide travel left to close."""
    d = kit.q(objects, "slide")
    goal = ((COMP_X[0] + 0.050) - d, sum(COMP["main"]) / 2)
    out = sum(math.dist([float(objects[t].pos[0]), float(objects[t].pos[1])], goal) for t in TABLETS)
    return float(out + d)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm only, open loop: grip the pull tab and draw the drawer out to its stop; pick each tablet from the bench
# and set it on the floor of the main-wash compartment; grip the tab again and push the drawer shut.

TIP = 0.013
TRAVEL_Z = TOP + 0.160
TAB_GX = (TAB_X[0] + TAB_X[1]) / 2
TAB_GZ = TAB_Z[1] - 0.020 + TIP  # pad tips 20 mm below the tab's top
PULL = TRAVEL_MAX + 0.020  # command a little past the stop
PUSH = 0.010  # and past fully shut
PINCH = FINGER_CLOSED
OPEN_T = FINGER_OPEN
OPEN_TAB = -0.020
DROP = {"tablet_a": 0.300, "tablet_b": 0.352}  # tablet centre x in the pulled-out compartment
MAIN_Y = sum(COMP["main"]) / 2


def _left():
    o, c = OPEN_TAB, PINCH
    gx, gy, gz = TAB_GX, TAB_Y, TAB_GZ
    out = gx - PULL
    plan = [
        Waypoint(2.0, (gx - 0.040, gy, TRAVEL_Z), o, "in front of the tab"),
        Waypoint(1.5, (gx, gy, gz + 0.030), o, "over the tab"),
        Waypoint(1.0, (gx, gy, gz), o, "fingers either side of the tab"),
        Waypoint(0.8, (gx, gy, gz), c, "grip the tab"),
        Waypoint(0.4, (gx, gy, gz), c, "settle"),
        Waypoint(3.0, (out, gy, gz), c, "pull the drawer out to its stop"),
        Waypoint(0.5, (out, gy, gz), c, "settle"),
        Waypoint(0.8, (out, gy, gz), o, "let go of the tab"),
        Waypoint(1.0, (out, gy, TRAVEL_Z), o, "up"),
    ]
    floor = TOP + 0.003 + TIP
    for t in TABLETS:
        tx, ty = TABLET_XY[t]
        dx = DROP[t] - TRAVEL_MAX * 0  # the compartment is already out
        put = FLOOR_TOP + 0.004 + 0.003 + TIP
        plan += [
            Waypoint(2.0, (tx, ty, TRAVEL_Z), OPEN_T, f"over {t}"),
            Waypoint(1.2, (tx, ty, floor + 0.030), OPEN_T, "descend"),
            Waypoint(0.8, (tx, ty, floor), OPEN_T, "fingers either side"),
            Waypoint(0.4, (tx, ty, floor), OPEN_T, "settle"),
            Waypoint(0.8, (tx, ty, floor), c, "grip"),
            Waypoint(0.4, (tx, ty, floor), c, "settle"),
            Waypoint(1.0, (tx, ty, TRAVEL_Z), c, "lift"),
            Waypoint(2.0, (dx, MAIN_Y, TRAVEL_Z), c, "over the main-wash compartment"),
            Waypoint(1.5, (dx, MAIN_Y, put + 0.030), c, "lower in"),
            Waypoint(1.0, (dx, MAIN_Y, put), c, "onto the floor"),
            Waypoint(0.4, (dx, MAIN_Y, put), c, "settle"),
            Waypoint(0.8, (dx, MAIN_Y, put), -0.028, "let go"),
            Waypoint(1.6, (dx, MAIN_Y, TRAVEL_Z), -0.028, "up out"),
        ]
    plan += [
        Waypoint(2.0, (out, gy, TRAVEL_Z), o, "back over the tab"),
        Waypoint(1.2, (out, gy, gz + 0.030), o, "down"),
        Waypoint(1.0, (out, gy, gz), o, "fingers either side of the tab"),
        Waypoint(0.8, (out, gy, gz), c, "grip the tab"),
        Waypoint(0.4, (out, gy, gz), c, "settle"),
        Waypoint(3.0, (gx + PUSH, gy, gz), c, "push the drawer shut"),
        Waypoint(0.6, (gx + PUSH, gy, gz), c, "settle"),
        Waypoint(0.8, (gx + PUSH, gy, gz), o, "let go"),
        Waypoint(1.2, (gx - 0.040, gy, TRAVEL_Z), o, "up and back"),
        Waypoint(1.5, (0.200, 0.330, TRAVEL_Z), o, "clear"),
    ]
    return plan


def _right():
    return [Waypoint(2.0, (0.200, -0.300, TRAVEL_Z), FINGER_CLOSED, "wait clear of the machine")]


def demo():
    """The left arm opens the drawer, doses both tablets into the main-wash compartment and shuts it."""
    return {"left": _left(), "right": _right()}


# -- offline self-test ------------------------------------------------------------------------------------------


def _state(travel=0.0, where=None):
    """Kit states with the drawer pulled out ``travel`` and each tablet named in ``where`` set flat in the middle
    (front half) of compartment ``where[t]``, or at an (x, y, z) given directly."""
    moved = {}
    for t, w in (where or {}).items():
        if isinstance(w, str):
            y0, y1 = COMP[w]
            x = DROP[t] if t in DROP else 0.30
            xyz = (x + TRAVEL_MAX - travel, (y0 + y1) / 2, FLOOR_TOP + TABLET[2] / 2)
            if w != "main":  # the narrow compartments: turned 90 deg so it fits
                xyz = (0.300 + TRAVEL_MAX - travel + (0.05 if t == "tablet_b" else 0.0), (y0 + y1) / 2, xyz[2])
                moved[t] = (np.array(xyz), mk.yaw_quat(math.pi / 2) if w == "softener" else mk.yaw_quat(0.0))
                continue
        else:
            xyz = w
        moved[t] = (np.array(xyz, float), mk.yaw_quat(0.0))
    return kit.fake_states(q={"slide": travel}, moved=moved)


def self_test():
    global _TEST_TCPS
    import dataclasses

    out = TRAVEL_MAX
    both_shut = _state(0.0, {"tablet_a": "main", "tablet_b": "main"})
    on_top = _state(0.0, {"tablet_a": (0.45, MAIN_Y, TOP + H + TABLET[2] / 2),
                          "tablet_b": (0.52, MAIN_Y, TOP + H + TABLET[2] / 2)})
    moving = dict(both_shut)
    moving["tablet_b"] = dataclasses.replace(moving["tablet_b"], lin_vel=np.array([0.0, 0.0, -0.05]))
    cases = {  # name: (state, tcps, pre-latched, score)
        "start": (kit.fake_states(), [], False, 0.0),
        "drawer pulled out, nothing in": (_state(out), [], False, 0.0),
        "both in main, shut": (both_shut, [], False, 1.0),
        "both in main, left open": (_state(out, {"tablet_a": "main", "tablet_b": "main"}), [], False, 0.6),
        "both in main, half shut": (_state(0.05, {"tablet_a": "main", "tablet_b": "main"}), [], False, 0.6),
        "one in main, shut": (_state(0.0, {"tablet_a": "main"}), [], False, 0.5),
        "one in main, open": (_state(out, {"tablet_b": "main"}), [], False, 0.3),
        "one main one pre-wash, shut": (_state(0.0, {"tablet_a": "main", "tablet_b": "prewash"}), [], False, 0.0),
        "both in softener, shut": (_state(0.0, {"tablet_a": "softener"}), [], False, 0.0),
        "pre-wash dose taken out again (latched)": (both_shut, [], True, 0.0),
        "tablets on the worktop over the drawer": (on_top, [], False, 0.0),
        "a tablet still falling": (moving, [], False, 0.5),
        "a tablet still held": (both_shut, [(0.27 - out, MAIN_Y, FLOOR_TOP + 0.02)], False, 1.0),
    }
    # the held tablet is at the drawer-shut position, the gripper where it would be with the drawer out: not near
    cases["a tablet still held"] = (_state(out, {"tablet_a": "main", "tablet_b": "main"}),
                                    [(DROP["tablet_a"], MAIN_Y, FLOOR_TOP + 0.02)], False, 0.3)
    try:
        for name, (st, tcps, latched, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            if latched:
                mk.MEMORY["wrong_compartment"] = True
            got, got_success = score(st), success(st)
            assert abs(got - want) < 1e-9 and got_success == (want >= 1.0), (name, got, want)
        assert abs(kit.q(_state(0.1), "slide") - 0.1) < 1e-9
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
