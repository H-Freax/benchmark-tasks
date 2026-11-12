"""On the bench is an aluminium air manifold with three push-in ports on top. Two are already blanked with stem
plugs; the middle one has an 8 mm polyurethane air line pushed into its push-to-connect fitting. The line rises out
of the fitting, bends over and runs along the bench to a small pneumatic cylinder on the right, whose own fitting it
is pushed into. A parts tray on the left holds one spare 8 mm stem plug standing head up. The air is off.

The job is to take the line off the manifold (the cylinder is being swapped) and blank the open port with the
plug. A push-to-connect fitting grips the tube with a toothed collet: the tube only comes out while the collet ring
is held pressed down against the fitting body, so one hand holds the collet down while the other draws the tube out.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/air-line-push-fitting/air_line_push_fitting.py \\
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
ALU = (0.70, 0.72, 0.75)
BRASS = (0.78, 0.62, 0.30)
BLACK = (0.10, 0.10, 0.11)
COLLET_BLUE = (0.15, 0.35, 0.80)
TUBE_BLUE = (0.20, 0.50, 0.90)
PLUG_GREY = (0.30, 0.31, 0.33)

# Manifold block, ports along y at its top
MANI = (0.040, 0.120, 0.020)
MANI_XY = (0.250, 0.000)
MT = TOP + MANI[2]  # manifold top
PORT_Y = (0.040, 0.000, -0.040)
PORT = (MANI_XY[0], 0.000)  # the live port

# Push-to-connect fitting (real 8 mm fitting proportions)
TUBE_D = 0.008
BORE = 0.009  # bore and collet inner opening (real collet ID equals the tube OD; 0.5 mm play each side here)
BODY_W, BODY_H = 0.018, 0.014
FLOOR = MT + 0.002  # tube stop inside the body
COLLET_W, COLLET_H = 0.018, 0.004  # release ring with its flange
COLLET_TOP = MT + BODY_H + COLLET_H
COLLET_TRAVEL = 0.002
RELEASE_AT = 0.0015  # collet pressed this far: teeth off the tube
INSERT = 0.012  # tube depth in the fitting (body top to floor)


def _square_ring(outer, inner, h, z0, color=None):
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    return [
        mk.box((outer, w, h), at=(0, c, z0 + h / 2), color=color),
        mk.box((outer, w, h), at=(0, -c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(c, 0, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(-c, 0, z0 + h / 2), color=color),
    ]


kit = mk.Kit()
# fitting body (fixed): square ring around the bore plus the floor/stop at the bottom
kit.part("fitting", (PORT[0], PORT[1], MT),
         [*_square_ring(BODY_W, BORE, BODY_H - 0.002, 0.002), mk.box((BODY_W, BODY_W, 0.002), at=(0, 0, 0.001))],
         mass=None, color=BRASS)
# collet ring: slides down 2 mm against a spring
kit.part("collet", (PORT[0], PORT[1], MT + BODY_H), _square_ring(COLLET_W, BORE, COLLET_H, 0.0), mass=0.030,
         color=COLLET_BLUE)
kit.slider("collet_travel", "fitting", "collet", axis=(0, 0, -1), lower=0.0, upper=COLLET_TRAVEL, stiffness=600.0,
           rest=-0.001, damping=5.0)

# Air line: a chain of rigid 8 mm segments on spring hinges (the PU tube's bending stiffness)
RISE = 0.050  # tube end segment length (12 mm in the fitting, 38 mm above it)
SEG = 0.040
N_RUN = 4  # horizontal run away from the robot (+x) to the cylinder
ELBOW_Z = FLOOR + RISE  # the bend
TUBE_KG = 0.004  # per segment (8x5.5 mm PU, about 25 g/m)
kit.part("tube_end", (PORT[0], PORT[1], FLOOR), [mk.box((TUBE_D, TUBE_D, RISE), at=(0, 0, RISE / 2))],
         mass=TUBE_KG * 1.5, color=TUBE_BLUE)
kit.bond("collet_grip", "fitting", "tube_end", anchor=(PORT[0], PORT[1], FLOOR + INSERT / 2), slip_mm=1.5)
_prev = "tube_end"
for _i in range(N_RUN):
    _name = f"tube_{_i}"
    x0 = PORT[0] + _i * SEG
    kit.part(_name, (x0, PORT[1], ELBOW_Z), [mk.box((SEG, TUBE_D, TUBE_D), at=(SEG / 2, 0, -TUBE_D / 2))],
             mass=TUBE_KG, color=TUBE_BLUE)
    _axis = (0, 1, 0) if _i % 2 == 0 else (0, 0, 1)  # alternate: bend up/down, bend sideways
    kit.hinge(f"bend_{_i}", _prev, _name, anchor=(x0, PORT[1], ELBOW_Z - TUBE_D / 2), axis=_axis,
              lower_deg=-90, upper_deg=90, stiffness=0.15, damping=0.002)
    _prev = _name
TUBE_PARTS = ["tube_end", *[f"tube_{i}" for i in range(N_RUN)]]
CYL_PORT_X = PORT[0] + N_RUN * SEG  # where the run ends (in the cylinder's fitting)

# Lockout ball valve on the manifold's supply: lever in line with the pipe = open (6 bar on the line), turned a
# quarter turn across it = closed and the downstream side exhausted (a 3-way lockout valve).
VALVE_XY = (0.250, 0.095)  # on the supply line, at the manifold's left end
VALVE = 0.025
VALVE_TOP = TOP + VALVE
LEVER_L = 0.060
CLOSED_DEG = 80.0
kit.part("valve_body", (VALVE_XY[0], VALVE_XY[1], TOP), [mk.box((VALVE, VALVE, VALVE), at=(0, 0, VALVE / 2))],
         mass=None, color=BRASS)
kit.part("valve_lever", (VALVE_XY[0], VALVE_XY[1], VALVE_TOP),
         [mk.box((0.006, 0.006, 0.006), at=(0, 0, 0.003)),
          mk.box((0.012, LEVER_L, 0.005), at=(0, LEVER_L / 2, 0.0085), color=(0.85, 0.12, 0.10))],
         mass=0.030, color=(0.6, 0.6, 0.62))
kit.hinge("valve_turn", "valve_body", "valve_lever", anchor=(VALVE_XY[0], VALVE_XY[1], VALVE_TOP), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=90.0, friction=0.08)

# Spare stem plug: 8 mm stem, 14 mm head
STEM_L, HEAD_W, HEAD_H = 0.016, 0.014, 0.008
TRAY_XY = (0.200, 0.130)
TRAY = (0.060, 0.060, 0.012)
TRAY_TOP = TOP + TRAY[2]
kit.part("plug", (TRAY_XY[0], TRAY_XY[1], TRAY_TOP - 0.010),
         [mk.box((0.0065, 0.0065, 0.002), at=(0, 0, 0.001)),  # lead-in chamfer at the stem's end
          mk.box((TUBE_D, TUBE_D, STEM_L - 0.002), at=(0, 0, 0.002 + (STEM_L - 0.002) / 2)),
          mk.box((HEAD_W, HEAD_W, HEAD_H), at=(0, 0, STEM_L + HEAD_H / 2))],
         mass=0.004, color=PLUG_GREY)


def _blanked_port(y):
    """A port already blanked: fitting body, collet and a plug head (fixed, scenery)."""
    x = PORT[0]
    return [
        Box(f"fit_{'L' if y > 0 else 'R'}", pos=(x, y, MT + BODY_H / 2), size=(BODY_W, BODY_W, BODY_H), color=BRASS),
        Box(f"col_{'L' if y > 0 else 'R'}", pos=(x, y, MT + BODY_H + COLLET_H / 2),
            size=(COLLET_W, COLLET_W, COLLET_H), color=COLLET_BLUE),
        Box(f"cap_{'L' if y > 0 else 'R'}", pos=(x, y, COLLET_TOP + HEAD_H / 2), size=(HEAD_W, HEAD_W, HEAD_H),
            color=PLUG_GREY),
    ]


def _tray():
    """Parts tray with a 9 mm hole holding the plug's stem (four blocks around the hole)."""
    (x, y), (lx, ly, h) = TRAY_XY, TRAY
    g = BORE / 2
    side = (lx / 2 - g)
    return [
        Box("tray_a", pos=(x + g + side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_b", pos=(x - g - side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_c", pos=(x, y + g + (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
        Box("tray_d", pos=(x, y - g - (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
        Box("tray_floor", pos=(x, y, TOP + 0.001), size=(2 * g, 2 * g, 0.002), color=BLACK),
    ]


FIXTURES = [
    Box("manifold", pos=(MANI_XY[0], MANI_XY[1], TOP + MANI[2] / 2), size=MANI, color=ALU),
    Box("supply_nipple", pos=(VALVE_XY[0], (MANI[1] / 2 + VALVE_XY[1] - VALVE / 2) / 2, TOP + 0.0125),
        size=(0.014, VALVE_XY[1] - VALVE / 2 - MANI[1] / 2, 0.014), color=BRASS),
    Box("supply_pipe", pos=(VALVE_XY[0], VALVE_XY[1] + VALVE / 2 + 0.040, TOP + 0.007), size=(0.014, 0.080, 0.014),
        color=(0.25, 0.25, 0.27)),
    *_blanked_port(PORT_Y[0]),
    *_blanked_port(PORT_Y[2]),
    # cylinder on its mounting foot, its port facing the manifold (-x) at the tube run's height
    Box("cyl_barrel", pos=(CYL_PORT_X + 0.016 + 0.050, PORT[1], ELBOW_Z - TUBE_D / 2), size=(0.100, 0.026, 0.026), color=ALU),
    Box("cyl_foot", pos=(CYL_PORT_X + 0.060, PORT[1], (TOP + ELBOW_Z - TUBE_D / 2 - 0.013) / 2),
        size=(0.030, 0.040, ELBOW_Z - TUBE_D / 2 - 0.013 - TOP), color=(0.35, 0.36, 0.38)),
    Box("cyl_fit_top", pos=(CYL_PORT_X + 0.008, PORT[1], ELBOW_Z - TUBE_D / 2 + 0.006), size=(0.016, 0.014, 0.003), color=BRASS),
    Box("cyl_fit_bot", pos=(CYL_PORT_X + 0.008, PORT[1], ELBOW_Z - TUBE_D / 2 - 0.006), size=(0.016, 0.014, 0.003), color=BRASS),
    Box("cyl_fit_l", pos=(CYL_PORT_X + 0.008, PORT[1] + 0.006, ELBOW_Z - TUBE_D / 2), size=(0.016, 0.003, 0.009), color=BRASS),
    Box("cyl_fit_r", pos=(CYL_PORT_X + 0.008, PORT[1] - 0.006, ELBOW_Z - TUBE_D / 2), size=(0.016, 0.003, 0.009), color=BRASS),
    *_tray(),
]
OBJECTS = [*kit.objects(), *FIXTURES]


def _release_rule(objects):
    """The collet teeth let go of the tube only while the collet is pressed down."""
    if kit.q(objects, "collet_travel") >= RELEASE_AT and "collet_grip" not in kit._released:
        kit.release("collet_grip")
        mk.MEMORY["released_by_collet"] = True
        if kit.q_deg(objects, "valve_turn") < CLOSED_DEG:
            mk.MEMORY["live_disconnect"] = True  # tube let go with 6 bar behind it: it whips out


kit.on_step(_release_rule)


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(st) -> float:
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def tube_out(objects) -> bool:
    """The tube end is out of the fitting (its tip 3 mm or more above the collet top, or off the port's axis) and
    nothing holds the line."""
    tip = kit.point(objects, "tube_end", (0, 0, 0))
    off_axis = math.dist((float(tip[0]), float(tip[1])), PORT) > BORE
    clear = float(tip[2]) >= COLLET_TOP + 0.003 or off_axis
    return clear and all(_released(objects[p]) for p in TUBE_PARTS)


def plug_seated(objects) -> bool:
    """Plug stem down in the bore (stem end within 3 mm of the floor), upright within 10 deg, let go and still."""
    st = objects["plug"]
    tip = kit.point(objects, "plug", (0, 0, 0))
    on_axis = math.dist((float(tip[0]), float(tip[1])), PORT) <= 0.002
    return (on_axis and float(tip[2]) <= FLOOR + 0.003 and _tilt(st) <= 10.0 and st.resting() and _released(st))


def valve_closed(objects) -> bool:
    return kit.q_deg(objects, "valve_turn") >= CLOSED_DEG


def live_disconnect(objects) -> bool:
    """The collet was released while the supply valve was open (latched for the episode)."""
    return bool(mk.MEMORY.get("live_disconnect"))


def score(objects) -> float:
    """0 if the line was ever released from the fitting with the supply valve open. Otherwise: 1.0 valve closed,
    line off and the port blanked with the plug, all let go; 0.5 valve closed and line off, port left open; 0.1
    valve closed only; else 0."""
    if live_disconnect(objects) or not valve_closed(objects):
        return 0.0
    if not tube_out(objects):
        return 0.1
    return 1.0 if plug_seated(objects) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    tip = kit.point(objects, "tube_end", (0, 0, 0))
    plug = kit.point(objects, "plug", (0, 0, 0))
    d = max(0.0, COLLET_TOP + 0.010 - float(tip[2]))
    return float(d + math.dist([float(v) for v in plug], [PORT[0], PORT[1], FLOOR]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: sweeps the valve lever closed, then comes down with its fingers 6.5 mm either side of the tube (the
# fingers close along y; each pad is 15 mm wide in x) onto the collet ring and holds it pressed. Right arm: grips the
# tube's run 100 mm from the bend, lifts it (the tube end slides up out of the fitting), swings it aside and lays it
# on the bench. Left arm lets the collet up, takes the plug from the tray, starts its stem in the bore, lets go and
# pushes it home by the head with closed fingers.

TIP = 0.013
TRAVEL = TOP + 0.130
STRADDLE = -0.0065  # each finger 6.5 mm off the axis: inside faces clear the 8 mm tube, pads on the collet ring
PRESS_Z = COLLET_TOP - 0.006 + TIP  # pushes the collet onto its 2 mm stop
GRIP_RUN_X = PORT[0] + 0.100  # on the run, 100 mm from the bend
PRESS_X = PORT[0] - 0.008  # pads (15 mm wide in x) over the ring's near half, the hand kept back from the other arm
RUN_Z = ELBOW_Z - TUBE_D / 2
YAW90 = math.pi / 2


LEVER_Z = VALVE_TOP + 0.0085 + TIP - 0.004  # finger tips 4 mm below the lever's top face


def _valve_sweep():
    """Closed fingers come down just on the near (-x) side of the lever, 45 mm out, and sweep it a quarter turn from
    along the pipe (+y, open) to across it (+x, closed)."""
    (vx, vy), r, off = VALVE_XY, 0.045, 0.014

    def at(deg):
        a = math.radians(deg)
        return (vx + r * math.sin(a) - off * math.cos(a), vy + r * math.cos(a) + off * math.sin(a))

    plan = [Waypoint(2.0, (*at(-5), TRAVEL), FINGER_CLOSED, "over the valve lever", 0.0),
            Waypoint(1.5, (*at(-5), LEVER_Z), FINGER_CLOSED, "down beside the lever", 0.0)]
    plan += [Waypoint(1.2, (*at(d), LEVER_Z), FINGER_CLOSED, f"sweep the lever to {d} deg", 0.0)
             for d in range(10, 125, 15)]
    plan.append(Waypoint(1.0, (*at(115), LEVER_Z), FINGER_CLOSED, "hold it across the pipe", 0.0))
    return plan + [Waypoint(1.0, (*at(105), TRAVEL), FINGER_CLOSED, "up off the closed valve", 0.0)]


def _left():
    px, py = PORT
    tx, ty = TRAY_XY
    grip_z = TRAY_TOP - 0.010 + 0.008 + TIP  # pad tips 8 mm up the plug: on the upper stem and the head
    start_z = COLLET_TOP - 0.006 + 0.008 + TIP  # stem 6 mm into the collet, pads still above it
    head_top = FLOOR + STEM_L + HEAD_H  # the head's top face with the plug home
    return _valve_sweep() + [
        Waypoint(2.0, (px, py, TRAVEL), STRADDLE, "over the fitting", 0.0),
        Waypoint(1.0, (PRESS_X, py, TRAVEL), STRADDLE, "fingers either side of the tube", 0.0),
        Waypoint(1.5, (PRESS_X, py, COLLET_TOP + TIP + 0.010), STRADDLE, "down beside the tube", 0.0),
        Waypoint(1.0, (PRESS_X, py, PRESS_Z), STRADDLE, "press the collet ring down", 0.0),
        Waypoint(12.0, (PRESS_X, py, PRESS_Z), STRADDLE, "hold it down while the tube comes out", 0.0),
        Waypoint(1.0, (px, py, TRAVEL), STRADDLE, "let the collet up", 0.0),
        Waypoint(2.0, (tx, ty, TRAVEL), FINGER_OPEN * 0.4, "over the plug", 0.0),
        Waypoint(1.5, (tx, ty, grip_z), FINGER_OPEN * 0.4, "down round the plug", 0.0),
        Waypoint(1.0, (tx, ty, grip_z), FINGER_OPEN * 0.4, "settle", 0.0),
        Waypoint(1.0, (tx, ty, grip_z), FINGER_CLOSED, "grip it", 0.0),
        Waypoint(1.0, (tx, ty, TRAVEL), FINGER_CLOSED, "lift", 0.0),
        Waypoint(2.0, (px, py, TRAVEL), FINGER_CLOSED, "over the port", 0.0),
        Waypoint(1.5, (px, py, start_z + 0.010), FINGER_CLOSED, "stem over the bore", 0.0),
        Waypoint(1.0, (px, py, start_z + 0.010), FINGER_CLOSED, "settle", 0.0),
        Waypoint(1.0, (px, py, start_z), FINGER_CLOSED, "stem into the collet", 0.0),
        Waypoint(0.8, (px, py, start_z), FINGER_OPEN * 0.4, "let go", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP + 0.015), FINGER_OPEN * 0.4, "up", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP + 0.015), FINGER_CLOSED, "close the fingers", 0.0),
        Waypoint(1.2, (px, py, head_top + TIP - 0.003), FINGER_CLOSED, "push the plug home", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP - 0.003), FINGER_CLOSED, "hold", 0.0),
        Waypoint(1.5, (px, py + 0.10, TRAVEL), FINGER_CLOSED, "clear", 0.0),
    ]


def _right():
    px = PORT[0]
    wait = sum(w.duration for w in _valve_sweep()) + 2.0 + 1.0 + 1.5 + 1.0 + 0.5  # until the collet is held down
    return [
        Waypoint(wait, (0.20, -0.20, TRAVEL), FINGER_OPEN * 0.5, "wait for the collet to be held", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, -0.120, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "out to the far side of the run", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "over the tube run", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, 0.0, RUN_Z + TIP - 0.010), FINGER_OPEN * 0.5, "down round the run", 0.0),
        Waypoint(1.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP - 0.010), FINGER_CLOSED, "grip the tube", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP + 0.035), FINGER_CLOSED, "draw it up out of the fitting", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, -0.090, RUN_Z + TIP + 0.035), FINGER_CLOSED, "swing it aside", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, -0.090, TOP + TUBE_D / 2 + TIP), FINGER_CLOSED, "lay it down", 0.0),
        Waypoint(1.0, (GRIP_RUN_X, -0.090, TOP + TUBE_D / 2 + TIP), FINGER_OPEN * 0.5, "let go", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, -0.200, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "up and away", 0.0),
        Waypoint(1.5, (0.20, -0.20, TRAVEL), FINGER_OPEN * 0.5, "clear", 0.0),
    ]


def demo():
    return {"left": _left(), "right": _right()}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = np.array([0, 0, 0.040])
    V = {"valve_turn": 90.0}
    start = kit.fake_states()
    end_pos, end_q = kit.pose(start, "tube_end")
    out = {"tube_end": (end_pos + up, end_q)}
    seat = (np.array([PORT[0], PORT[1], FLOOR]), np.array([0, 0, 0, 1.0]))

    def st(q=V, **moved):
        return kit.fake_states(q=q, deg=True, moved=moved)

    plugged = st(**out, plug=seat)
    cases = {  # name: (state, tcps, live_disconnect latched, score)
        "start": (start, [], False, 0.0),
        "valve closed only": (st(), [], False, 0.1),
        "valve half turned, nothing else": (st(q={"valve_turn": 45.0}), [], False, 0.0),
        "valve closed, tube pulled up only 4 mm": (st(tube_end=(end_pos + np.array([0, 0, 0.004]), end_q)), [], False,
                                                   0.1),
        "valve closed, tube out, port open": (st(**out), [], False, 0.5),
        "full: valve closed, tube out, plug seated": (plugged, [], False, 1.0),
        "plug half in": (st(**out, plug=(seat[0] + np.array([0, 0, 0.008]), seat[1])), [], False, 0.5),
        "plug tilted 23 deg": (st(**out, plug=(seat[0], mk.mat_to_quat(mk.axis_angle((1, 0, 0), 0.4)))), [], False,
                               0.5),
        "tube still held by a gripper": (plugged, [kit.point(plugged, "tube_0", (0, -0.02, 0))], False, 0.1),
        "plug still held": (plugged, [np.array([PORT[0], PORT[1], FLOOR + 0.019])], False, 0.5),
        "pulled live, valve closed afterwards, all else done": (plugged, [], True, 0.0),
        "valve open, tube out, plug seated": (kit.fake_states(moved={**out, "plug": seat}), [], False, 0.0),
    }
    try:
        for name, (s_, tcps, live, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            if live:
                mk.MEMORY["live_disconnect"] = True
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
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
