"""On a restaurant expo counter stand two closed moulded-fibre takeaway boxes (24 x 18 x 7 cm), one per side. Each lid
carries a printed order code (A17 on the left box, B24 on the right) and a blue panel where the tamper seal goes.
Between them, on its release backing, lies the one tamper seal for this order: a thin blue paper label with a
folded pull tab at its front edge. A folded napkin lies across the label and hides the order code printed on it.
The job: find out which order the seal is for and stick it on that box's panel.

A tamper seal is single-use: once its adhesive face touches a lid it is stuck there. So the seal touching the other
box's lid (resting on it, held or not) latches the episode to 0. Scored from the label's pose: on the matching
box's panel, wholly inside it, face up and flat, let go and still.

Parts come from mechkit.py (in this folder), which builds multi-box bodies in the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/service-takeaway-box-seal/service_takeaway_box_seal.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
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

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
MAT_T = 0.004  # grey nitrile service mat
S = TOP + MAT_T  # the mat's top: everything on the counter stands on it

FIBER = (0.76, 0.69, 0.56)
FIBER_EDGE = (0.64, 0.57, 0.44)
SEAM = (0.20, 0.15, 0.10)
PANEL_BLUE = (0.06, 0.30, 0.52)
LABEL_BLUE = (0.08, 0.42, 0.73)
INK = (0.025, 0.030, 0.032)
WHITE = (0.93, 0.92, 0.86)
MAT_GREY = (0.28, 0.31, 0.31)
STEEL = (0.56, 0.60, 0.62)

# -- the two takeaway boxes -----------------------------------------------------------------------------------------

BOX = (0.24, 0.18, 0.07)
BOX_X = 0.39
BOX_Y = {"A17": 0.18, "B24": -0.18}  # left box A17, right box B24
LID_TOP = S + BOX[2]
PANEL = (0.10, 0.12)  # the blue seal panel on each lid (x, y), printed flush
PANEL_X = 0.35
PANEL_TOP = LID_TOP + 0.0004
CODE_X = 0.455  # order code printed on the lid behind the panel
SEAL_CODE = "B24"  # the order the seal is for: printed on the seal, hidden under the napkin at the start


def _tilt_quat(axis, deg):
    return tuple(mk.mat_to_quat(mk.axis_angle(axis, math.radians(deg))))


def _box_boxes():
    """A closed clamshell in its own frame (origin centre of the base, on the mat): tapered shell, lid with rim,
    hinge at the back, closure latch at the front, fibre flutes and the lid seam, plus the flush blue panel."""
    b = mk.box
    out = [
        b((0.215, 0.155, 0.052), (0.0, 0.0, 0.026)),
        b((0.012, 0.170, 0.054), (-0.112, 0.0, 0.029), quat=_tilt_quat((0, 1, 0), -6.0)),
        b((0.012, 0.170, 0.054), (0.112, 0.0, 0.029), quat=_tilt_quat((0, 1, 0), 6.0)),
        b((0.225, 0.012, 0.054), (0.0, 0.083, 0.029), quat=_tilt_quat((1, 0, 0), 6.0)),
        b((0.225, 0.012, 0.054), (0.0, -0.083, 0.029), quat=_tilt_quat((1, 0, 0), -6.0)),
        b((0.226, 0.166, 0.015), (0.0, 0.0, 0.0625)),  # lid
        b((0.010, 0.176, 0.010), (-0.116, 0.0, 0.057), FIBER_EDGE),  # lid rim
        b((0.010, 0.176, 0.010), (0.116, 0.0, 0.057), FIBER_EDGE),
        b((0.232, 0.008, 0.010), (0.0, 0.086, 0.057), FIBER_EDGE),
        b((0.232, 0.008, 0.010), (0.0, -0.086, 0.057), FIBER_EDGE),
        b((0.012, 0.115, 0.012), (0.118, 0.0, 0.050), FIBER_EDGE),  # hinge
        b((0.010, 0.050, 0.018), (-0.119, 0.0, 0.047), FIBER_EDGE),  # closure latch
        b((0.002, 0.180, 0.004), (-0.1225, 0.0, 0.0515), SEAM),  # lid seam
        b((0.002, 0.180, 0.004), (0.1225, 0.0, 0.0515), SEAM),
        b((0.246, 0.002, 0.004), (0.0, 0.0905, 0.0515), SEAM),
        b((0.246, 0.002, 0.004), (0.0, -0.0905, 0.0515), SEAM),
        b((PANEL[0], PANEL[1], 0.0004), (PANEL_X - BOX_X, 0.0, BOX[2] + 0.0002), PANEL_BLUE),  # seal panel
    ]
    for fy in (-0.060, -0.030, 0.0, 0.030, 0.060):
        out.append(b((0.003, 0.010, 0.043), (-0.119, fy, 0.028), FIBER_EDGE))
    for sx, sy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        out.append(b((0.010, 0.010, 0.050), (0.106 * sx, 0.077 * sy, 0.032), FIBER_EDGE))
    return out


# Seven-segment glyphs: (right, up, width, height) of each segment, for characters 22 x 44 mm at scale 1.
SEGMENTS = {"a": (0.0, 0.018, 0.022, 0.008), "b": (0.011, 0.009, 0.008, 0.018), "c": (0.011, -0.009, 0.008, 0.018),
            "d": (0.0, -0.018, 0.022, 0.008), "e": (-0.011, -0.009, 0.008, 0.018), "f": (-0.011, 0.009, 0.008, 0.018),
            "g": (0.0, 0.0, 0.022, 0.008)}
DIGITS = {"1": "bc", "2": "abdeg", "4": "bcfg", "7": "abc", "A": "abcefg", "B": "cdefg"}


def _code(code, centre, scale, z, h=0.0004):
    """Printed code reading upright from the robot: character 'up' is +x, 'right' is -y."""
    out = []
    for i, ch in enumerate(code):
        r0 = (i - 1) * 0.035 * scale
        for seg in DIGITS[ch]:
            dr, du, w, hh = SEGMENTS[seg]
            out.append(mk.box((hh * scale, w * scale, h), (centre[0] + du * scale, centre[1] - (r0 + dr * scale),
                                                          z + h / 2), INK))
    return out


kit = mk.Kit()
for _code_name, _y in BOX_Y.items():
    kit.part(f"box_{_code_name}", (BOX_X, _y, S),
             [*_box_boxes(), *_code(_code_name, (CODE_X - BOX_X, 0.0), 1.2, BOX[2])], color=FIBER)

# -- the seal on its backing, and the napkin over it ----------------------------------------------------------------

BACKING = (0.050, 0.070, 0.0006)
BACKING_XY = (0.200, 0.0)
LABEL = (0.045, 0.064, 0.0015)  # the seal's paper (collider a little thicker than real 0.1 mm stock)
LABEL_KG = 0.010
TAB_AT, TAB = -0.0295, (0.014, 0.030, 0.018)  # folded pull tab at the front edge (part frame x, size)
LABEL_START = (BACKING_XY[0], BACKING_XY[1], S + BACKING[2] + LABEL[2] / 2)  # the label part's origin
GLYPH_H = 0.0004
kit.part("seal_label", LABEL_START, [
    mk.box(LABEL, (0.0, 0.0, 0.0)),
    mk.box(TAB, (TAB_AT, 0.0, TAB[2] / 2 - LABEL[2] / 2)),
    *_code(SEAL_CODE, (0.003, 0.0), 0.55, LABEL[2] / 2, GLYPH_H),
], mass=LABEL_KG, color=LABEL_BLUE)

NAPKIN = (0.070, 0.064, 0.010)
NAPKIN_KG = 0.020
NAPKIN_XY = (0.214, 0.0)
NAPKIN_Z = S + BACKING[2] + LABEL[2] + GLYPH_H + NAPKIN[2] / 2  # lying on the seal
kit.part("napkin", (NAPKIN_XY[0], NAPKIN_XY[1], NAPKIN_Z), [
    mk.box(NAPKIN, (0.0, 0.0, 0.0)),
    mk.box((0.035, NAPKIN[1], 0.0015), (0.0175, 0.0, NAPKIN[2] / 2 + 0.00075)),  # the top fold
    mk.box((0.0015, NAPKIN[1] - 0.002, 0.0003), (0.0, 0.0, NAPKIN[2] / 2 + 0.0015), (0.78, 0.77, 0.70)),  # crease
], mass=NAPKIN_KG, color=WHITE)

FIXTURES = [
    Box("service_mat", pos=(0.33, 0.0, TOP + MAT_T / 2), size=(0.62, 0.50, MAT_T), color=MAT_GREY),
    Box("release_backing", pos=(BACKING_XY[0], BACKING_XY[1], S + BACKING[2] / 2), size=BACKING, color=WHITE),
    Box("counter_edge", pos=(0.655, 0.0, TOP - 0.020), size=(0.030, 0.50, 0.050), color=STEEL),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

FLAT_DEG = 12.0
ON_LID_M = 0.004  # the seal's underside within 4 mm of a lid's top: touching it
PEEL_M = 0.010  # the seal lifted 1 cm above its backing at least once: peeled
CORNERS = [(sx * LABEL[0] / 2, sy * LABEL[1] / 2, -LABEL[2] / 2) for sx in (-1, 1) for sy in (-1, 1)]
_TEST_TCPS = None


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(st) -> float:
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _corners(objects) -> np.ndarray:
    return np.array([kit.point(objects, "seal_label", c) for c in CORNERS])


def _touching(objects, code) -> bool:
    """Any of the seal's paper over this box's lid and its underside within 4 mm of the lid (stuck on contact)."""
    c = _corners(objects)
    centre = c.mean(axis=0)
    y = BOX_Y[code]
    over = (abs(centre[0] - BOX_X) <= BOX[0] / 2 and abs(centre[1] - y) <= BOX[1] / 2)
    return bool(over and c[:, 2].min() <= LID_TOP + 0.0004 + ON_LID_M)


def _track(objects) -> None:
    """Per step: latch the peel and any touch on the wrong box."""
    st = objects["seal_label"]
    if float(np.asarray(st.low)[2]) > S + BACKING[2] + PEEL_M:
        mk.MEMORY["peeled"] = True
    for code in BOX_Y:
        if code != SEAL_CODE and _touching(objects, code):
            mk.MEMORY["wrong_box"] = True


kit.on_step(_track)


def _placed(objects) -> tuple[bool, bool]:
    """(on the matching lid let go, flat, still; and wholly inside its panel)."""
    st = objects["seal_label"]
    if not (_touching(objects, SEAL_CODE) and _tilt(st) <= FLAT_DEG and st.resting() and _released(st)):
        return False, False
    c = _corners(objects)
    px, py = PANEL_X, BOX_Y[SEAL_CODE]
    inside = bool(np.all(np.abs(c[:, 0] - px) <= PANEL[0] / 2) and np.all(np.abs(c[:, 1] - py) <= PANEL[1] / 2))
    return True, inside


def score(objects) -> float:
    """0 for good once the seal touched the other box's lid. Otherwise 1.0 seal wholly inside the matching box's
    panel, face up and flat (within 12 deg), let go and still; 0.5 on the matching lid that way but over the panel's
    edge; 0.2 peeled off its backing (lifted 1 cm) but not placed; else 0."""
    _track(objects)
    if mk.MEMORY.get("wrong_box"):
        return 0.0
    on_lid, inside = _placed(objects)
    if inside:
        return 1.0
    if on_lid:
        return 0.5
    return 0.2 if mk.MEMORY.get("peeled") else 0.0


def success(objects) -> bool:
    """The seal on the matching panel, let go and still, never on the other box; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the fixed layout. Left arm: pinch the napkin across its width, lift it off and lay it on the
# counter in front of the left box (the seal's code, B24, is now visible). Right arm: pinch the seal's pull tab,
# peel it up off the backing, carry it to the B24 box and lay it flat on the panel, let go and lift clear.

TIP = 0.013
TRAVEL = TOP + 0.140
NAP_Z = S - 0.002 + TIP  # pad tips just over the mat, beside the napkin's overhang
NAP_DROP = (0.150, 0.190)
TAB_X = LABEL_START[0] + TAB_AT - 0.002  # pads a little in front of the tab's centre, clear of the label
TAB_Z = S + 0.004 + TIP  # pad tips 4 mm above the mat, clear of it while closing
GRIP_DZ = TAB_Z - LABEL_START[2]  # TCP above the label origin while held
TARGET = (PANEL_X + 0.004, BOX_Y[SEAL_CODE] + 0.025)  # TCP placing the hanging seal centred on the panel
PUT_Z = PANEL_TOP + LABEL[2] / 2 + GRIP_DZ - 0.002  # press 2 mm


def _left():
    x, y = NAPKIN_XY
    o, c = FINGER_OPEN, FINGER_CLOSED
    dx, dy = NAP_DROP
    return [
        Waypoint(2.0, (x, y, TRAVEL), o, "over the napkin"),
        Waypoint(1.5, (x, y, NAP_Z + 0.03), o, "descend"),
        Waypoint(1.0, (x, y, NAP_Z), o, "pads beside the napkin"),
        Waypoint(1.0, (x, y, NAP_Z), c, "pinch the napkin"),
        Waypoint(1.0, (x, y, TRAVEL), c, "lift it off the seal"),
        Waypoint(1.5, (dx, dy, TRAVEL - 0.05), c, "carry to the counter in front of the left box"),
        Waypoint(1.0, (dx, dy, S + 0.012 + TIP), c, "lower"),
        Waypoint(0.8, (dx, dy, S + 0.012 + TIP), o, "let go"),
        Waypoint(1.0, (dx - 0.07, dy, S + 0.012 + TIP), o, "slide the fingers off the napkin"),
        Waypoint(1.0, (dx - 0.07, dy, TRAVEL), o, "up"),
        Waypoint(1.5, (0.12, 0.30, TRAVEL), o, "clear"),
    ]


def _right(wait_s):
    o, c = FINGER_OPEN, FINGER_CLOSED
    tx, ty = TARGET
    return [
        Waypoint(2.0, (0.12, -0.20, TRAVEL), o, "wait clear"),
        Waypoint(wait_s, (0.12, -0.20, TRAVEL), o, "wait for the napkin to be off"),
        Waypoint(2.0, (TAB_X, 0.0, TRAVEL), o, "over the pull tab"),
        Waypoint(1.5, (TAB_X, 0.0, TAB_Z + 0.03), o, "descend"),
        Waypoint(1.0, (TAB_X, 0.0, TAB_Z), o, "pads beside the tab"),
        Waypoint(1.2, (TAB_X, 0.0, TAB_Z), c, "pinch the tab"),
        Waypoint(2.5, (TAB_X, 0.0, TAB_Z + 0.03), c, "peel the seal up off its backing"),
        Waypoint(1.0, (TAB_X - 0.01, 0.0, TAB_Z + 0.06), c, "lift it clear"),
        Waypoint(1.0, (TAB_X, 0.0, TRAVEL), c, "lift"),
        Waypoint(2.5, (tx, ty, TRAVEL), c, "carry to the B24 box"),
        Waypoint(1.5, (tx, ty, PUT_Z + 0.03), c, "lower over the panel"),
        Waypoint(1.5, (tx, ty, PUT_Z), c, "lay it flat on the panel and press"),
        Waypoint(0.5, (tx, ty, PUT_Z), c, "settle"),
        Waypoint(1.0, (tx, ty, PUT_Z), o, "let go"),
        Waypoint(1.0, (tx, ty, PUT_Z + 0.06), o, "lift clear"),
        Waypoint(1.5, (0.15, -0.25, TRAVEL), o, "retreat"),
    ]


def demo():
    """Left arm takes the napkin off (about 12 s), then the right arm seals the B24 box. About 30 s."""
    left = _left()
    return {"left": left, "right": _right(max(0.5, sum(w.duration for w in left[:-1]) - 2.0 - 2.0))}


# -- offline self-test ------------------------------------------------------------------------------------------


def _label_at(xy, z_bottom, yaw_deg=0.0, tilt_deg=0.0):
    r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg)) @ mk.axis_angle((1, 0, 0), math.radians(tilt_deg))
    pos = np.array([xy[0], xy[1], z_bottom + LABEL[2] / 2])
    return kit.fake_states(moved={"seal_label": (pos, mk.mat_to_quat(r))})


def self_test():
    """The scoring rules on hand-built poses; each case is one episode (latches start clear)."""
    global _TEST_TCPS
    import dataclasses

    bx, by = PANEL_X, BOX_Y[SEAL_CODE]
    good = _label_at((bx, by), PANEL_TOP)
    held_tcp = [(bx + TAB_AT, by, PANEL_TOP + 0.016)]
    moving = dict(good)
    moving["seal_label"] = dataclasses.replace(moving["seal_label"], lin_vel=np.array([0.0, 0.0, -0.05]))
    lifted = _label_at(LABEL_START[:2], S + 0.05)
    wrong = _label_at((PANEL_X, BOX_Y["A17"]), PANEL_TOP)
    cases = {  # name: (sequence of states seen in the episode, gripper TCPs at the end, score)
        "start: on its backing": ([kit.fake_states()], [], 0.0),
        "sealed on the B24 panel": ([lifted, good], [], 1.0),
        "on the B24 panel, still held": ([lifted, good], held_tcp, 0.2),
        "on the B24 panel, still falling": ([lifted, moving], [], 0.2),
        "on the B24 lid, over the panel edge": ([lifted, _label_at((bx + 0.05, by), PANEL_TOP)], [], 0.5),
        "on the B24 panel, tilted 20 deg": ([lifted, _label_at((bx, by), PANEL_TOP + 0.01, tilt_deg=20)], [], 0.2),
        "peeled and held up": ([lifted], [(0.17, 0.0, S + 0.06)], 0.2),
        "sealed on the A17 box (wrong order)": ([lifted, wrong], [], 0.0),
        "touched A17 then moved to B24": ([lifted, wrong, good], [], 0.0),
        "dropped on the counter": ([lifted, _label_at((0.25, -0.05), S)], [], 0.2),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for st in seq[:-1]:
                _track(st)
            _TEST_TCPS = tcps
            got = score(seq[-1])
            assert abs(got - want) < 1e-9 and success(seq[-1]) == (want >= 1.0), (name, got, want)
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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

