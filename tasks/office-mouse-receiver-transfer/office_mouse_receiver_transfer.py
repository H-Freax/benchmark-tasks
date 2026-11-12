"""On the desk, to the robot's left, lies a wireless mouse, nose away from the robot. Its USB-A receiver is
stowed in the storage slot in the mouse's back end: the metal plug is inside the mouse and the receiver's dark body
sticks out 28 mm toward the robot. A snap detent holds it, so pulling it out takes a firm tug (about 3 N), more
than it takes to slide the light mouse across the desk. To the robot's right stands an open laptop, turned so its
right side faces the robot; that side has a USB-A port at desk level. The job is to take the receiver out of the
mouse and plug it into the laptop.

Scored from the parts' poses: the receiver out of the mouse; the receiver's plug in the laptop's port (centred,
square to it, pushed in), let go and still; nothing counts if the mouse was dragged more than 2 cm or knocked off
the desk.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/office-mouse-receiver-transfer/office_mouse_receiver_transfer.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (bond and per-episode hooks, installed in memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
MOUSE_XY = (0.300, 0.100)  # mouse centre on the desk, nose toward +x
MOUSE_KG = 0.095
MOUSE_L = 0.118
SHELL = (0.20, 0.21, 0.23)
DARK = (0.06, 0.06, 0.07)
METAL = (0.78, 0.79, 0.80)
ALU = (0.66, 0.67, 0.70)

kit = mk.Kit()
kit.part("mouse", (MOUSE_XY[0], MOUSE_XY[1], TOP), [
    mk.box((MOUSE_L, 0.062, 0.020), at=(0.0, 0.0, 0.010)),
    mk.box((0.096, 0.054, 0.012), at=(0.004, 0.0, 0.026)),
    mk.box((0.066, 0.044, 0.006), at=(0.008, 0.0, 0.035)),
    mk.box((0.040, 0.002, 0.0005), at=(0.032, 0.0, 0.03825), color=DARK),  # button split
    mk.box((0.012, 0.006, 0.002), at=(0.024, 0.0, 0.0385), color=DARK),  # wheel
    mk.box((0.0008, 0.017, 0.009), at=(-MOUSE_L / 2 - 0.0004, 0.0, 0.010), color=DARK),  # slot mouth
], mass=MOUSE_KG, color=SHELL)

# Receiver frame: origin where plug meets body, on the plug axis; +x toward the plug tip.
PLUG = (0.012, 0.012, 0.0045)
BODY = (0.028, 0.015, 0.0075)
RECV_KG = 0.020
SLOT = (MOUSE_XY[0] - MOUSE_L / 2, MOUSE_XY[1], TOP + 0.010)  # the receiver's origin when stowed
kit.part("receiver", SLOT, [
    mk.box(PLUG, at=(PLUG[0] / 2, 0.0, 0.0), color=METAL),
    mk.box(BODY, at=(-BODY[0] / 2, 0.0, 0.0), color=DARK),
], mass=RECV_KG, color=DARK)
DETENT_N = 3.0
kit.bond("detent", "mouse", "receiver", break_force=DETENT_N)

# Laptop: fixed, left side toward the robot at x LAP_X0, keyboard deck 20 mm tall, screen open along its far side.
LAP_X0, LAP_X1 = 0.300, 0.600
LAP_Y0, LAP_Y1 = -0.255, -0.045
LAP_H = 0.020
PORT_Y, PORT_Z = -0.100, TOP + 0.010  # port axis
PORT_W, PORT_H, PORT_D = 0.014, 0.0055, 0.014  # opening (plug clear by 1 mm a side across, 0.5 mm top and bottom)
_zl, _zh = PORT_Z - PORT_H / 2, PORT_Z + PORT_H / 2
_yl, _yh = PORT_Y - PORT_W / 2, PORT_Y + PORT_W / 2
_cx, _lx = (LAP_X0 + LAP_X1) / 2, LAP_X1 - LAP_X0


def _box(name, x0, x1, y0, y1, z0, z1, color=ALU):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0),
               color=color)


FIXTURES = [
    _box("laptop_bottom", LAP_X0, LAP_X1, LAP_Y0, LAP_Y1, TOP, _zl),
    _box("laptop_top", LAP_X0, LAP_X1, LAP_Y0, LAP_Y1, _zh, TOP + LAP_H),
    _box("laptop_mid_near", LAP_X0, LAP_X1, _yh, LAP_Y1, _zl, _zh),
    _box("laptop_mid_far", LAP_X0, LAP_X1, LAP_Y0, _yl, _zl, _zh),
    _box("laptop_port_back", LAP_X0 + PORT_D, LAP_X1, _yl, _yh, _zl, _zh, color=DARK),
    _box("laptop_keyboard", LAP_X0 + 0.040, LAP_X1 - 0.040, LAP_Y0 + 0.020, LAP_Y1 - 0.090, TOP + LAP_H,
         TOP + LAP_H + 0.001, color=DARK),
    _box("laptop_screen", LAP_X0, LAP_X1, LAP_Y0, LAP_Y0 + 0.006, TOP + LAP_H, TOP + LAP_H + 0.200),
    _box("laptop_display", LAP_X0 + 0.010, LAP_X1 - 0.010, LAP_Y0 + 0.006, LAP_Y0 + 0.007, TOP + LAP_H + 0.012,
         TOP + LAP_H + 0.190, color=(0.10, 0.12, 0.16)),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

OUT_M = 0.015  # out of the bay: the plug fully out of the slot (12 mm plug) with margin
LATERAL_M = 0.0015
AXIS_DEG = 8.0
DEPTH_M = 0.009  # plug at least 9 of its 12 mm inside the port face
DRAG_M = 0.020
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _mouse_spoiled(objects) -> bool:
    """Latched: the mouse dragged more than 2 cm across the desk, or off it, at any step this episode."""
    if mk.MEMORY.get("mouse_spoiled"):
        return True
    pos, _ = kit.pose(objects, "mouse")
    if math.dist([float(pos[0]), float(pos[1])], MOUSE_XY) > DRAG_M or float(pos[2]) < TOP - 0.02:
        mk.MEMORY["mouse_spoiled"] = True
    return bool(mk.MEMORY.get("mouse_spoiled"))


kit.on_step(_mouse_spoiled)


def _out_of_bay(objects) -> bool:
    pos, _ = kit.pose(objects, "receiver")
    return float(np.linalg.norm(np.asarray(pos) - np.asarray(SLOT))) >= OUT_M


def _in_port(objects) -> bool:
    """Plug centred on the port axis within 1.5 mm, square to it within 8 deg, at least 9 mm in; let go, still."""
    origin, q = kit.pose(objects, "receiver")
    ax = mk.quat_to_mat(q)[:, 0]
    if math.degrees(math.acos(max(-1.0, min(1.0, float(ax[0]))))) > AXIS_DEG:
        return False
    mid = np.asarray(origin) + ax * (PLUG[0] / 2)
    if math.hypot(float(mid[1]) - PORT_Y, float(mid[2]) - PORT_Z) > LATERAL_M:
        return False
    tip = np.asarray(origin) + ax * PLUG[0]
    if float(tip[0]) - LAP_X0 < DEPTH_M:
        return False
    st = objects["receiver"]
    return st.resting() and _released(st)


def score(objects) -> float:
    """0 if the mouse was dragged over 2 cm or fell (latched); 1.0 receiver in the laptop port (centred within
    1.5 mm, within 8 deg, 9 mm in, let go, still); 0.3 receiver out of the mouse's bay; else 0."""
    if _mouse_spoiled(objects):
        return 0.0
    if _in_port(objects):
        return 1.0
    return 0.3 if _out_of_bay(objects) else 0.0


def success(objects) -> bool:
    """Receiver plugged in, let go and still, mouse not dragged; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from the plug's tip to the bottom of the port."""
    origin, q = kit.pose(objects, "receiver")
    tip = np.asarray(origin) + mk.quat_to_mat(q)[:, 0] * PLUG[0]
    return float(np.linalg.norm(tip - np.array([LAP_X0 + PLUG[0], PORT_Y, PORT_Z])))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: closed fingers pressed down on the mouse's front half, holding it while the receiver is pulled.
# Right arm: open fingers around the receiver's body behind the mouse, close, pull straight back (the detent lets
# go), lift, carry over to the laptop, line the plug up with the port, push it in, open the fingers and lift away.

TIP = 0.013  # TCP above the pad tips
GRIP_BACK = 0.024  # TCP this far behind the receiver's origin: pads on the back of the body, so the fingers
# (which reach ~13 mm ahead of the TCP) stay clear of the laptop when the plug is in
PINCH = -0.005  # finger target: pads a few mm into the 15 mm body, a firm but gentle pinch
HELD_DZ = 0.0068  # TCP command above the held receiver's axis (it hangs ~6.4 mm below the TCP)
PUSH = 0.010  # the receiver slips ~5 mm back in the pads during the pull; push on until its body meets the laptop
PRESS_X = MOUSE_XY[0] + 0.030
PRESS_Z = TOP + 0.038 + TIP - 0.004  # pad tips 4 mm into the mouse's top: a firm press
TRAVEL = TOP + 0.090


def _left():
    x, y = PRESS_X, MOUSE_XY[1]
    c = FINGER_CLOSED
    return [
        Waypoint(2.0, (x, y, TRAVEL), c, "over the mouse"),
        Waypoint(1.5, (x, y, PRESS_Z + 0.015), c, "down"),
        Waypoint(1.0, (x, y, PRESS_Z), c, "press down on the mouse"),
        Waypoint(10.5, (x, y, PRESS_Z), c, "hold it while the receiver comes out"),
        Waypoint(1.5, (x, y, TRAVEL), c, "lift off"),
        Waypoint(1.5, (x - 0.120, y + 0.100, TRAVEL), c, "clear"),
    ]


def _right():
    sx, sy, sz = SLOT
    gx, gz = sx - GRIP_BACK, sz + 0.006  # pad tips 3 mm above the desk
    px = LAP_X0 - GRIP_BACK  # TCP with the receiver's origin at the port face
    iz = PORT_Z + HELD_DZ
    o, c = FINGER_OPEN, PINCH
    return [
        Waypoint(2.5, (gx - 0.030, sy, TRAVEL), o, "behind the mouse"),
        Waypoint(1.5, (gx, sy, TRAVEL), o, "over the receiver"),
        Waypoint(1.5, (gx, sy, gz + 0.020), o, "descend"),
        Waypoint(1.0, (gx, sy, gz), o, "fingers either side of the body"),
        Waypoint(0.5, (gx, sy, gz), o, "settle"),
        Waypoint(1.0, (gx, sy, gz), c, "grip"),
        Waypoint(0.5, (gx, sy, gz), c, "settle"),
        Waypoint(2.0, (gx - 0.035, sy, gz), c, "pull straight back out of the slot"),
        Waypoint(0.5, (gx - 0.035, sy, gz), c, "settle"),
        Waypoint(1.2, (gx - 0.035, sy, TRAVEL), c, "lift"),
        Waypoint(2.5, (px - 0.030, PORT_Y, TRAVEL), c, "carry to the laptop"),
        Waypoint(1.5, (px - 0.030, PORT_Y, iz), c, "down to the port"),
        Waypoint(0.5, (px - 0.030, PORT_Y, iz), c, "line up"),
        Waypoint(2.0, (px + PUSH, PORT_Y, iz), c, "push the plug in"),
        Waypoint(0.5, (px + PUSH, PORT_Y, iz), c, "settle"),
        Waypoint(1.0, (px + PUSH, PORT_Y, iz), o, "let go"),
        Waypoint(1.5, (px + PUSH - 0.004, PORT_Y, TRAVEL), o, "lift away"),
        Waypoint(1.5, (px - 0.100, PORT_Y - 0.080, TRAVEL), o, "clear"),
    ]


def demo():
    """The left arm holds the mouse down; the right arm pulls the receiver and plugs it into the laptop."""
    return {"left": _left(), "right": _right()}


# -- offline self-test ------------------------------------------------------------------------------------------


def _receiver_at(origin, yaw_deg=0.0, pitch_deg=0.0, mouse_xy=None):
    r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg)) @ mk.axis_angle((0, 1, 0), math.radians(pitch_deg))
    moved = {"receiver": (np.asarray(origin, float), mk.mat_to_quat(r))}
    if mouse_xy is not None:
        moved["mouse"] = (np.array([mouse_xy[0], mouse_xy[1], TOP]), np.array([0.0, 0.0, 0.0, 1.0]))
        moved["receiver"] = (np.asarray(origin, float), mk.mat_to_quat(r))
    return kit.fake_states(moved=moved)


def self_test():
    """The scoring rules on hand-built poses."""
    global _TEST_TCPS
    import dataclasses

    seated = (LAP_X0 - 0.001, PORT_Y, PORT_Z)
    on_desk = (0.200, 0.000, TOP + BODY[2] / 2)
    cases = {  # name: (state, tcps, pre-latched mouse, score)
        "start": (kit.fake_states(), [], False, 0.0),
        "plugged in": (_receiver_at(seated), [], False, 1.0),
        "plugged in, mouse dragged earlier": (_receiver_at(seated), [], True, 0.0),
        "out, on the desk": (_receiver_at(on_desk), [], False, 0.3),
        "out, mouse dragged 3 cm": (_receiver_at(on_desk, mouse_xy=(MOUSE_XY[0] - 0.03, MOUSE_XY[1])), [], False,
                                    0.0),
        "plug only 6 mm in": (_receiver_at((LAP_X0 - 0.006, PORT_Y, PORT_Z)), [], False, 0.3),
        "plug 2 mm off axis": (_receiver_at((LAP_X0 - 0.001, PORT_Y + 0.002, PORT_Z)), [], False, 0.3),
        "plug tilted 12 deg": (_receiver_at(seated, pitch_deg=12.0), [], False, 0.3),
        "plug in but still held": (_receiver_at(seated), [(LAP_X0 - 0.010, PORT_Y, PORT_Z + 0.013)], False, 0.3),
    }
    moving = dict(_receiver_at(seated))
    moving["receiver"] = dataclasses.replace(moving["receiver"], lin_vel=np.array([0.0, 0.0, -0.05]))
    cases["plug in, still moving"] = (moving, [], False, 0.3)
    try:
        for name, (st, tcps, spoiled, want) in cases.items():
            mk.new_episode()
            if spoiled:
                mk.MEMORY["mouse_spoiled"] = True
            _TEST_TCPS = tcps
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    # geometry: the plug clears the port by 1 mm a side (0.5 mm top and bottom), the body does not fit
    assert abs((PORT_W - PLUG[1]) / 2 - 0.001) < 1e-9 and abs((PORT_H - PLUG[2]) / 2 - 0.0005) < 1e-9
    assert BODY[1] > PORT_W and BODY[2] > PORT_H
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
