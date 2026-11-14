"""A drop-in clamshell receipt printer stands on the bench, its lid latched shut over an empty paper core. A new
80 mm thermal roll lies on the bench in front of it (toward the right arm), its leader lying flat beside it.

The job: open the lid (a release button on the printer's near side lets the lid's spring lift it), take the old
core out, drop the new roll into the cradle so the paper comes off the bottom of the roll toward the tear bar,
bring the leader over the platen and out of the paper slot, and press the lid shut until its latch snaps
(a 10 N press against the latch spring).

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/receipt-printer-roll-clamshell/receipt_printer_roll_clamshell.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY = (0.20, 0.21, 0.22)  # dark grey housing
LID_C = (0.28, 0.29, 0.31)
GREY = (0.55, 0.56, 0.58)
BLUE = (0.15, 0.35, 0.75)
PAPER = (0.97, 0.97, 0.95)
CORE_C = (0.62, 0.48, 0.32)
PLATEN_C = (0.10, 0.10, 0.10)

# -- housing ------------------------------------------------------------------------------------------------------
X0, X1 = 0.230, 0.350  # 120 mm wide (the lid's width), along x
YF, YB = -0.065, 0.055  # front (paper slot, tear bar) toward -y; hinge along the rear top edge
ZT = TOP + 0.045  # housing top
ZF = TOP + 0.008  # paper bay floor
SIDE = 0.010
PLATEN_Y = -0.050  # inner edge of the platen block: paper going out passes over it
SLOT_Y = YF  # the tear bar: paper beyond this line is outside the printer
YC = -0.0025  # cradle axis (y); the roll axis runs along x
XC = (X0 + X1) / 2

# -- roll and core ------------------------------------------------------------------------------------------------
RO, R_HOLE, RW = 0.030, 0.006, 0.080  # 60 mm roll on a 12 mm hole, 80 mm wide
RC = ZF + 0.036  # roll axis height when seated on the cradle pads
PAD_DY = 0.020
PAD_TOP = RC - math.sqrt(RO ** 2 - PAD_DY ** 2)
CORE_S = 0.016  # old core: 18 mm across (two 16 mm squares at 45 deg)
PL, PT, PW, NPL = 0.024, 0.003, 0.052, 4  # leader: 4 plates 24 mm long, 3 mm thick, 52 mm wide

NEW0 = (XC, -0.170, TOP + RO)  # the new roll on the bench, leader lying out toward -y
CORE0 = (XC, YC, ZF + CORE_S / 2)


def _qx(a):
    return tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), a)))


def _roll_boxes():
    """A 12-sided 60 mm roll round a 12 mm hole, axis along the part's x, frame at its centre."""
    w = 2 * RO * math.tan(math.pi / 12)
    t = RO - R_HOLE
    rc = (RO + R_HOLE) / 2
    out = []
    for i in range(12):
        a = 2 * math.pi * i / 12
        out.append(mk.box((RW, w, t), at=(0, rc * math.cos(a), rc * math.sin(a)), quat=_qx(a - math.pi / 2)))
    return out


kit = mk.Kit()

kit.part("roll", NEW0, _roll_boxes(), mass=0.15, color=PAPER)
# the leader: four plates hinged edge to edge along x, the first hinged to the roll at the bottom of the roll
prev = "roll"
for i in range(NPL):
    y0 = NEW0[1] - i * PL
    kit.part(f"leaf{i}", (XC, y0, TOP + PT / 2), [mk.box((PW, PL, PT), at=(0, -PL / 2, 0))], mass=0.02, color=PAPER)
    kit.hinge(f"leaf{i}_fold", prev, f"leaf{i}", anchor=(XC, y0, TOP + PT / 2), axis=(1, 0, 0), lower_deg=-150.0,
              upper_deg=150.0, damping=0.0002)
    prev = f"leaf{i}"

kit.part("core", CORE0, [mk.box((RW, CORE_S, CORE_S)), mk.box((RW, CORE_S, CORE_S), quat=_qx(math.pi / 4))],
         mass=0.02, color=CORE_C)

# lid: a 120 x 122 mm shell 36 mm tall hinged along the rear top edge; a spring lifts it open (stop at 100 deg)
# once the latch lets go; it closes onto a sprung latch pawl that needs about 10 N to push home
CD = 0.042
LID_Y0 = YF - 0.002
LD = YB - LID_Y0  # 122 mm deep
HINGE = (XC, YB, ZT)
OPEN_DEG = 100.0
SPRING_DEG = 125.0  # the opening spring's free angle (beyond the 100 deg stop)
HOLD_DEG = -1500.0  # while latched, the hinge drive stands in for the latch: it holds the lid down on the pawl
_h = CD - 0.0055
_lid = [mk.box((X1 - X0, LD, 0.004), at=(0, -LD / 2, CD - 0.002)),
        mk.box((0.004, LD, _h), at=(X0 - XC + 0.002, -LD / 2, 0.0015 + _h / 2)),
        mk.box((0.004, LD, _h), at=(X1 - XC - 0.002, -LD / 2, 0.0015 + _h / 2)),
        mk.box((X1 - X0 - 0.008, 0.004, _h), at=(0, -0.002, 0.0015 + _h / 2)),
        mk.box((X1 - X0 - 0.008, 0.004, CD - 0.014), at=(0, -LD + 0.002, 0.010 + (CD - 0.014) / 2))]  # front skirt: a 10 mm paper exit gap over the tear bar
kit.part("hinge_block", (XC, YB - 0.005, ZT - 0.006), [mk.box((0.090, 0.010, 0.012))], mass=None, color=BODY)
kit.part("lid", HINGE, _lid, mass=0.10, color=LID_C)
kit.hinge("lid_hinge", "hinge_block", "lid", anchor=HINGE, axis=(-1, 0, 0), lower_deg=0.0, upper_deg=OPEN_DEG, start_deg=0.0,
          stiffness=0.06, rest_deg=HOLD_DEG, damping=0.03)
kit.catch("lid_latch", "lid", kit.parts["lid"].ref)

# latch pawl: under the lid's left wall at the front corner, sprung up (7 N preload, 600 N/m)
PAWL = (X0 + 0.006, YF + 0.0075)
PAWL_TRAVEL, PAWL_CLOSED = 0.005, 0.004
kit.part("pawl_well", (X0 + 0.007, (YF + PLATEN_Y) / 2, ZF), [mk.box((0.014, PLATEN_Y - YF, ZT - 0.010 - ZF), at=(0, 0, (ZT - 0.010 - ZF) / 2))],
         mass=None, color=BODY)
kit.part("pawl", (*PAWL, ZT + 0.0005), [mk.box((0.010, 0.010, 0.010))], mass=0.02, color=GREY)
kit.slider("pawl_push", "pawl_well", "pawl", axis=(0, 0, -1), lower=0.0, upper=PAWL_TRAVEL, start=PAWL_CLOSED,
           stiffness=600.0, rest=-0.012, damping=1.0)

# release button on a boss beside the near side wall
REL = (X0 - 0.050, -0.030)
REL_TOP = ZT
BTN_TRAVEL, BTN_PRESSED = 0.006, 0.004
kit.part("rel_base", (*REL, TOP), [mk.box((0.018, 0.018, REL_TOP - 0.012 - TOP), at=(0, 0, (REL_TOP - 0.012 - TOP) / 2))],
         mass=None, color=BODY)
kit.part("rel_btn", (*REL, REL_TOP - 0.012), [mk.box((0.014, 0.014, 0.012), at=(0, 0, 0.006))], mass=0.020, color=BLUE)
kit.slider("rel_push", "rel_base", "rel_btn", axis=(0, 0, -1), lower=0.0, upper=BTN_TRAVEL, stiffness=400.0,
           rest=-0.002, damping=2.0)


def _bx(name, x0, x1, y0, y1, z0, z1, color=BODY):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0), color=color)


FIXTURES = [
    _bx("base", X0, X1, YF, YB, TOP, ZF),
    _bx("wall_near", X0, X0 + SIDE, PLATEN_Y, YB, ZF, ZT),
    _bx("wall_far", X1 - SIDE, X1, PLATEN_Y, YB, ZF, ZT),
    _bx("wall_rear", X0 + SIDE, X1 - SIDE, YB - 0.010, YB, ZF, ZT - 0.012),
    _bx("platen_block", X0 + 0.014, X1, YF, PLATEN_Y, ZF, ZT - 0.004),
    _bx("platen", X0 + 0.014, X1 - SIDE, PLATEN_Y - 0.010, PLATEN_Y, ZT - 0.004, ZT, PLATEN_C),
    _bx("tear_bar", X0 + 0.014, X1, YF, PLATEN_Y - 0.010, ZT - 0.004, ZT, GREY),
    *[_bx(f"pad{i}{j}", XC + sx * RW / 2 - (0.010 if sx > 0 else -0.001) - 0.0, XC + sx * RW / 2 + (0.0 if sx > 0 else 0.011) - (0.001 if sx > 0 else 0),
          YC + sy * PAD_DY - 0.004, YC + sy * PAD_DY + 0.004, ZF, PAD_TOP, GREY)
      for i, sx in enumerate((-1, 1)) for j, sy in enumerate((-1, 1))],
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules --------------------------------------------------------------------------------------------------------
LATCH_DEG = 0.5  # the pawl snaps into the lid's keeper only with the lid pushed this far home
JAM_DEG = 5.0  # a lid down this far over a roll seals whatever path the paper has


def _disengage(name):
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _lid_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    q = kit.q_deg(objects, "lid_hinge")
    pressed = kit.q(objects, "rel_push") >= BTN_PRESSED
    if m["n"] == 1:
        kit.engage("lid_latch")
        m["latched"] = True
        return
    if m.get("latched"):
        if pressed:
            _disengage("lid_latch")
            kit.drive("lid_hinge", SPRING_DEG)
            m["latched"] = False
            m["opened"] = True
        return
    if q < JAM_DEG and roll_in_bay(objects) and (unwind_dot(objects) < 0.0 or protrusion(objects) < PROTRUDE):
        m["jammed"] = True
    if q < LATCH_DEG and not pressed:
        kit.engage("lid_latch")
        kit.drive("lid_hinge", HOLD_DEG)
        m["latched"] = True
        m["relatched"] = True


def _floor_rule(objects):
    for n in ("roll", "core"):
        if float(kit.point(objects, n, (0, 0, 0))[2]) < TOP - 0.05:
            mk.MEMORY["floor"] = True


def _debug_rule(objects):
    if os.environ.get("RECEIPT_DEBUG") == "2" and mk.MEMORY.get("n", 0) < 3:
        print("[receipt] paths", mk._SIM["catch_paths"], mk._SIM["joint_paths"].get((kit.id, "lid_hinge")),
              {n: (tuple(round(float(v), 4) for v in objects[n].pos), tuple(round(float(v), 4) for v in objects[n].quat_xyzw))
               for n in ("lid", "hinge_block", "pawl", "pawl_well")}, flush=True)
    every = 2 if os.environ.get("RECEIPT_DEBUG") == "2" else 20
    if os.environ.get("RECEIPT_DEBUG") and mk.MEMORY.get("n", 0) % every == 0:
        f = lambda v: "(" + ",".join(f"{float(c):.4f}" for c in v) + ")"
        print(f"[receipt] n={mk.MEMORY.get('n')} roll={f(kit.point(objects, 'roll', (0, 0, 0)))} "
              f"core={f(kit.point(objects, 'core', (0, 0, 0)))} tab={f(leader_points(objects)[-1])} "
              f"lid={kit.q_deg(objects, 'lid_hinge'):.1f} btn={kit.q(objects, "rel_push"):.4f} pawl={kit.q(objects, 'pawl_push'):.4f} "
              f"dot={unwind_dot(objects):.2f} prot={protrusion(objects):.4f} "
              f"mem={ {k: v for k, v in mk.MEMORY.items() if k != 'n' and not isinstance(k, tuple)} } "
              f"score={score(objects)}", flush=True)


kit.on_step(_lid_rule)
kit.on_step(_floor_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ------------------------------------------------------------------------------------------------------
_TEST_TCPS = None
SEAT_TOL, SEAT_DEG = 0.008, 6.0
PROTRUDE = 0.015


def leader_points(objects):
    """The leader as a polyline: where it leaves the roll, each fold, and the tab's end."""
    pts = [kit.point(objects, f"leaf{i}", (0, 0, 0)) for i in range(NPL)]
    pts.append(kit.point(objects, f"leaf{NPL - 1}", (0, -PL, 0)))
    return [np.asarray(p, float) for p in pts]


def protrusion(objects) -> float:
    """Length of leader beyond the tear bar line (outside the printer's front), along the leader."""
    pts = leader_points(objects)
    out = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        ya, yb = float(a[1]) - SLOT_Y, float(b[1]) - SLOT_Y
        seg = float(np.linalg.norm(b - a))
        if ya < 0 and yb < 0:
            out += seg
        elif ya < 0 or yb < 0:
            out += seg * (-min(ya, yb)) / (abs(ya) + abs(yb))
    return out


def unwind_dot(objects) -> float:
    """Direction the paper leaves the bottom of the roll (roll axis x down), dotted with the way out (-y)."""
    _, q = kit.pose(objects, "roll")
    a = mk.quat_to_mat(q) @ np.array([-1.0, 0.0, 0.0])
    u = np.cross(a, [0.0, 0.0, -1.0])
    n = float(np.linalg.norm(u))
    return float(-u[1] / n) if n > 1e-9 else 0.0


def roll_in_bay(objects) -> bool:
    c = kit.point(objects, "roll", (0, 0, 0))
    return X0 < float(c[0]) < X1 and PLATEN_Y < float(c[1]) < YB and float(c[2]) < ZT + 0.020


def roll_seated(objects) -> bool:
    """Roll axis within 8 mm of the cradle axis and within 6 deg of it: down in the cradle on its pads."""
    c = kit.point(objects, "roll", (0, 0, 0))
    _, q = kit.pose(objects, "roll")
    ax = mk.quat_to_mat(q)[:, 0]
    return (math.dist([float(v) for v in c], [XC, YC, RC]) <= SEAT_TOL
            and abs(float(ax[0])) >= math.cos(math.radians(SEAT_DEG)))


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def core_out(objects) -> bool:
    """The old core is out of the printer, lying on the bench, let go."""
    c = kit.point(objects, "core", (0, 0, 0))
    inside = X0 - 0.01 < float(c[0]) < X1 + 0.01 and YF - 0.01 < float(c[1]) < YB + 0.01
    return not inside and float(c[2]) < TOP + 0.02 and _released(objects["core"])


def roll_ok(objects) -> bool:
    return roll_seated(objects) and unwind_dot(objects) > 0.8 and _released(objects["roll"])


def leader_across(objects) -> bool:
    """The tab's end is past the platen, inside the printer's width (it can only get there over the platen)."""
    e = leader_points(objects)[-1]
    return float(e[1]) < PLATEN_Y and X0 < float(e[0]) < X1


def lid_locked(objects) -> bool:
    return bool(mk.MEMORY.get("latched")) and bool(mk.MEMORY.get("opened")) and kit.q_deg(objects, "lid_hinge") < 1.0


def settled(objects) -> bool:
    parts = ("roll", "lid", "rel_btn", *[f"leaf{i}" for i in range(NPL)])
    return (all(_released(objects[n]) for n in parts)
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("roll", "lid")))


def failed(objects) -> bool:
    return bool(mk.MEMORY.get("jammed") or mk.MEMORY.get("floor"))


def score(objects) -> float:
    """0 if failed (lid closed over a reversed roll or a leader left inside; a roll or core off the bench).
    Otherwise 0.2 old core out; 0.3 new roll seated the right way round (paper off the bottom toward the slot);
    with that, 0.2 leader across the platen; with both, 0.3 lid latched with at least 15 mm of leader out the
    slot, everything let go and still."""
    if failed(objects):
        return 0.0
    s = 0.2 * core_out(objects)
    if roll_ok(objects):
        s += 0.3
        if leader_across(objects):
            s += 0.2
            if lid_locked(objects) and protrusion(objects) >= PROTRUDE and settled(objects):
                s += 0.3
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c = kit.point(objects, "roll", (0, 0, 0))
    return float(math.dist([float(v) for v in c], [XC, YC, RC]))


# -- the scripted demo --------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = 0.840
F_ROLL_OPEN, F_ROLL_GRIP, F_ROLL_LET = -0.046, -0.026, -0.040
F_CORE_OPEN = -0.024
CARRY = 0.838
BTN_Z = REL_TOP - BTN_TRAVEL - 0.006 + TIP
LID_TOP = ZT + CD
PRESS = (0.262, -0.052)  # where the demo presses the lid: near its front, toward the latch side


class _Seq:
    def __init__(self):
        self.w, self.t = [], 0.0
        self.last = None

    def go(self, d, pos, f, label, yaw=0.0):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, yaw))
        self.t += d
        self.last = (tuple(pos), f, yaw)
        return self

    def until(self, t, label="wait"):
        pos, f, yaw = self.last
        return self.go(max(0.1, t - self.t), pos, f, label, yaw)


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    # R: press the release button; the lid's spring lifts it
    bx, by = REL
    R.go(2.0, (bx, by, BTN_Z + 0.040), 0.0, "over the release button")
    R.go(1.0, (bx, by, BTN_Z), 0.0, "press it")
    R.go(1.5, (bx, by, BTN_Z), 0.0, "hold while the lid lifts")
    R.go(0.8, (bx, by, BTN_Z + 0.040), 0.0, "let go")
    L.go(2.0, (0.20, 0.25, TRAVEL), 0.0, "start")
    L.until(R.t + 0.5)
    # L: the old core out onto the bench
    cz = ZF + CORE_S / 2 + 0.008
    L.go(1.5, (0.175, 0.12, TRAVEL), F_CORE_OPEN, "round the near side, clear of the open lid")
    L.go(1.5, (0.175, YC, TRAVEL), F_CORE_OPEN, "beside the printer")
    L.go(1.5, (XC, YC, TRAVEL), F_CORE_OPEN, "over the old core")
    L.go(1.5, (XC, YC, cz), F_CORE_OPEN, "down round it")
    L.go(1.0, (XC, YC, cz), 0.0, "grip")
    L.go(1.5, (XC, YC, TRAVEL), 0.0, "lift it out")
    L.go(1.5, (0.175, YC, TRAVEL), 0.0, "out to the near side")
    L.go(1.5, (0.175, 0.17, TRAVEL), 0.0, "over the bench")
    L.go(0.5, (0.20, 0.17, TRAVEL), 0.0, "over the bench")
    L.go(1.5, (0.20, 0.17, TOP + CORE_S / 2 + 0.010), 0.0, "down")
    L.go(0.8, (0.20, 0.17, TOP + CORE_S / 2 + 0.010), F_CORE_OPEN, "let go")
    L.go(1.0, (0.20, 0.17, TRAVEL), F_CORE_OPEN, "up")
    t_core = L.t
    L.go(1.5, (0.20, 0.30, 0.90), 0.0, "park")
    if variant == "close_only":
        R.go(1.5, (0.20, -0.33, TRAVEL), 0.0, "clear")
        L.until(R.t)
        L.w, L.t = [], 0.0
        L.go(2.0, (0.20, 0.25, TRAVEL), 0.0, "start")
        L.until(R.t + 1.5)
        _close_lid(L)
        return {"left": L.w, "right": R.w}
    # R: the new roll, its leader trailing toward the front
    nx, ny, nz = NEW0
    yaw = 0.0
    R.go(2.5, (nx, ny, TRAVEL), F_ROLL_OPEN, "over the new roll")
    R.go(1.5, (nx, ny, nz + 0.003), F_ROLL_OPEN, "down round it")
    R.go(1.0, (nx, ny, nz + 0.003), F_ROLL_GRIP, "grip")
    R.go(0.5, (nx, ny, nz + 0.003), F_ROLL_GRIP, "settle")
    R.until(t_core - 1.0)
    R.go(2.0, (nx, ny, CARRY), F_ROLL_GRIP, "lift, the leader hanging")
    if variant == "reversed":  # end for end in two turns (the wrist will not do 180 deg in one): 143 deg, regrasp, 37 deg
        a = 2.5
        R.go(2.5, (nx, ny, CARRY), F_ROLL_GRIP, "turn it 143 deg", a)
        R.go(1.5, (nx, ny, nz + 0.004), F_ROLL_GRIP, "set it down", a)
        R.go(0.8, (nx, ny, nz + 0.004), F_ROLL_OPEN, "let go", a)
        R.go(1.0, (nx, ny, nz + 0.075), F_ROLL_OPEN, "up clear of it", a)
        R.go(1.5, (nx, ny, nz + 0.075), F_ROLL_OPEN, "turn the wrist back", a - math.pi)
        R.go(1.5, (nx, ny, nz + 0.003), F_ROLL_OPEN, "down round it", a - math.pi)
        R.go(1.0, (nx, ny, nz + 0.003), F_ROLL_GRIP, "grip", a - math.pi)
        R.go(2.0, (nx, ny, CARRY), F_ROLL_GRIP, "lift", a - math.pi)
        R.go(2.0, (nx, ny, CARRY), F_ROLL_GRIP, "turn it the rest of the way", 0.0)
    R.go(3.5, (XC, YC, CARRY), F_ROLL_GRIP, "in over the tear bar, the leader dragging over it", yaw)
    R.go(1.0, (XC, YC, CARRY), F_ROLL_GRIP, "settle", yaw)
    R.go(2.0, (XC, YC, RC + 0.001), F_ROLL_GRIP, "down into the cradle", yaw)
    R.go(0.5, (XC, YC, RC + 0.001), F_ROLL_GRIP, "settle", yaw)
    if variant == "tucked":  # lift it, draw it rearward to pull the leader in, and set it back low so it is not pushed out
        R.go(1.5, (XC, YC, CARRY), F_ROLL_GRIP, "lift it again", yaw)
        R.go(2.0, (XC, YC + 0.018, CARRY), F_ROLL_GRIP, "draw it rearward", yaw)
        R.go(2.0, (XC, YC + 0.018, RC + 0.012), F_ROLL_GRIP, "down behind the cradle", yaw)
        R.go(1.5, (XC, YC, RC + 0.012), F_ROLL_GRIP, "forward over the cradle", yaw)
        R.go(1.0, (XC, YC, RC + 0.001), F_ROLL_GRIP, "down", yaw)
        R.go(0.5, (XC, YC, RC + 0.001), F_ROLL_GRIP, "settle", yaw)
    R.go(0.8, (XC, YC, RC + 0.001), F_ROLL_LET, "let go", yaw)
    R.go(1.5, (XC, YC, TRAVEL + 0.03), F_ROLL_LET, "up", yaw)
    R.go(2.0, (0.20, -0.33, 0.90), 0.0, "park")
    L.until(R.t - 1.5)
    _close_lid(L)
    R.until(L.t)
    return {"left": L.w, "right": R.w}


def _close_lid(s):
    """Sweep the open lid over from behind, then press its front down until the latch snaps."""
    s.go(2.0, (XC, 0.16, TRAVEL + 0.006), 0.0, "behind the open lid")
    s.go(4.0, (XC, -0.020, TRAVEL + 0.006), 0.0, "sweep it over and down")
    s.go(1.0, (PRESS[0], PRESS[1], LID_TOP + TIP + 0.002), 0.0, "onto its front corner, over the latch")
    s.go(1.2, (PRESS[0], PRESS[1], LID_TOP + TIP - 0.030), 0.0, "press it home")
    s.go(1.0, (PRESS[0], PRESS[1], LID_TOP + TIP - 0.030), 0.0, "hold")
    s.go(1.0, (XC, -0.030, LID_TOP + 0.030), 0.0, "up")
    s.go(1.5, (0.20, 0.30, 0.90), 0.0, "park")
    s.go(3.0, (0.20, 0.30, 0.90), 0.0, "wait")


def demo():
    # DEMO_VARIANT: close_only | reversed | tucked (degenerate checks; all score 0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test --------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])
    flip = mk.mat_to_quat(mk.axis_angle((0, 0, 1), math.pi))

    def st(q=None, **moved):
        return kit.fake_states(q=q, moved=moved)

    def chain_routed(roll_q=I):
        """Roll seated in the cradle, leader up over the platen, the last 1.5 plates out of the slot."""
        mv = {"roll": (np.array([XC, YC, RC]), roll_q), "core": (np.array([0.20, 0.17, TOP + CORE_S / 2]), I)}
        p0 = np.array([XC, YC, RC - RO])
        pts = [p0, np.array([XC, -0.030, ZF + 0.020]), np.array([XC, -0.052, ZT + PT / 2]),
               np.array([XC, -0.052 - PL, ZT + PT / 2]), None]
        for i in range(NPL):
            a = pts[i]
            b = pts[i + 1]
            if b is None:
                mv[f"leaf{i}"] = (a, I)
                continue
            d = (b - a) / np.linalg.norm(b - a)
            ang = math.atan2(float(d[2]), float(-d[1]))
            mv[f"leaf{i}"] = (a, mk.mat_to_quat(mk.axis_angle((1, 0, 0), -ang)))
            pts[i + 1] = a + PL * np.array([0, float(d[1]), float(d[2])])
        return mv

    open_ = {"lid_hinge": math.radians(OPEN_DEG)}
    closed = {"lid_hinge": 0.0}
    L = {"latched": True, "opened": True}
    good = chain_routed()
    cases = {
        "start": (st(), [], {"latched": True}, 0.0),
        "start, never opened": (st(closed), [], {"latched": True}, 0.0),
        "core out": (st(open_, core=(np.array([0.20, 0.17, TOP + CORE_S / 2]), I)), [], {"opened": True}, 0.2),
        "core held over bench": (st(open_, core=(np.array([0.20, 0.17, TOP + CORE_S / 2]), I)),
                                 [np.array([0.20, 0.17, TOP + 0.02])], {"opened": True}, 0.0),
        "core out, closed over nothing": (st(closed, core=(np.array([0.20, 0.17, TOP + CORE_S / 2]), I)), [], L, 0.2),
        "routed, lid open": (st(open_, **good), [], {"opened": True}, 0.7),
        "routed, lid latched": (st(closed, **good), [], L, 1.0),
        "routed, lid latched, hand on lid": (st(closed, **good), [np.array([XC, -0.04, LID_TOP + 0.01])], L, 0.7),
        "routed, lid shut not latched": (st({"lid_hinge": math.radians(1.5)}, **good), [], {"opened": True}, 0.7),
        "reversed roll, lid open": (st(open_, **chain_routed(flip)), [], {"opened": True}, 0.2),
        "reversed roll, latched": (st(closed, **chain_routed(flip)), [], {**L, "jammed": True}, 0.0),
        "leader inside, latched": (st(closed, **good), [], {**L, "jammed": True}, 0.0),
        "roll on the floor": (st(open_, **good), [], {"opened": True, "floor": True}, 0.0),
        "roll held in the cradle": (st(open_, **good), [np.array([XC, YC, RC + 0.003])], {"opened": True}, 0.2),
        "roll on the core": (st(open_, roll=(np.array([XC, YC, RC + 0.012]), I)), [], {"opened": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert abs(unwind_dot(st()) - 1.0) < 1e-9
        assert unwind_dot(st(roll=(np.array(NEW0), flip))) < -0.99
        assert protrusion(st(closed, **good)) >= 0.020, protrusion(st(closed, **good))
        assert protrusion(st()) > 0.08  # lying on the bench in front of the printer
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
    for v in ("", "close_only", "reversed", "tucked"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
