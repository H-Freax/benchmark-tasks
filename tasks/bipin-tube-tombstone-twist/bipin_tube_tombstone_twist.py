"""A 2 ft T8 fluorescent strip fixture lies on its back on the bench, its two rotary ("tombstone") lampholders
facing each other across it: the left one fixed, the right one on a sprung slide. A new T8 tube (25.4 mm glass,
589.8 mm between end-cap faces, two 2 mm pins at each end, 12.7 mm apart) lies on the bench in front of it.

The job: lower the tube with both pin pairs upright into the holders' slots, which means pressing the sprung holder
4 mm outward (the gap between the holders at rest is 4 mm short of the tube), then turn the tube a quarter turn so
the rotors lock the pins across the slot. Unturned pins only sit in an open slot: let go of the tube unturned and it
counts as fallen out.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/bipin-tube-tombstone-twist/bipin_tube_tombstone_twist.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1600 \\
           --instruction "Install the new tube in the fixture." --scorer task_score ...
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

WHITE = (0.93, 0.93, 0.91)
IVORY = (0.88, 0.86, 0.80)
GLASS = (0.95, 0.96, 0.97)
METAL = (0.72, 0.72, 0.70)
GREY = (0.55, 0.56, 0.58)

# -- tube -------------------------------------------------------------------------------------------------------
R_T = 0.0127  # T8: 1 inch
L_T = 0.5898  # between end-cap faces
PIN_L, PIN_D, PIN_HALF = 0.0066, 0.002, 0.00635  # G13 pins, 12.7 mm apart
TUBE_M = 0.18

# -- fixture ----------------------------------------------------------------------------------------------------
AX = 0.300  # tube axis over the fixture, x
ZP = TOP + 0.006  # fixture channel face
Z_AX = ZP + 0.030  # lampholder axis height
YF = L_T / 2  # fixed (left) holder's face; seated tube centred at y 0
COMP = 0.004  # the sprung holder at rest stands this far inside the tube's length
YR0 = -(L_T / 2) + COMP  # sprung (right) holder's face at rest
SLOT = 0.004  # slot width (pins 2 mm)
SPRING_K, SPRING_PRELOAD = 200.0, 3.0  # N/m, N at rest
SLIDE_TRAVEL = 0.008
DETENT_DEG = 90.0

TUBE0 = (0.170, 0.000, TOP + R_T)  # start: on the bench in front of the fixture, pins upright


def _tube_boxes():
    out = []
    s = 2 * R_T * math.tan(math.pi / 12)
    for i in range(6):  # 12-sided glass cylinder from six crossed slabs
        a = math.pi / 6 * i
        q = mk.mat_to_quat(mk.axis_angle((0, 1, 0), a))
        out.append(mk.box((s, L_T - 0.030, 2 * R_T), quat=q, color=GLASS))
    for sgn in (-1, 1):  # aluminium end caps
        for i in range(6):
            a = math.pi / 6 * i
            q = mk.mat_to_quat(mk.axis_angle((0, 1, 0), a))
            out.append(mk.box((s, 0.015, 2 * R_T), at=(0, sgn * (L_T / 2 - 0.0075), 0), quat=q, color=METAL))
        for pz in (-PIN_HALF, PIN_HALF):
            out.append(mk.box((PIN_D, PIN_L, PIN_D), at=(0, sgn * (L_T / 2 + PIN_L / 2), pz), color=METAL))
    return out


def _bevel(o, x, cw, z0, z1, run):
    """Guard block whose inner top edge is bevelled: the face at y 0 from z0, sloping out ``run`` by z1."""
    out = [mk.box((cw, 0.012 - run, z1 - z0), at=(x, o * (run + (0.012 - run) / 2), (z0 + z1) / 2))]
    dy, dz = o * run, z1 - z0
    ln = math.hypot(dy, dz)
    nin = (o * dz / ln, -abs(dy) / ln)  # into the material
    c = (dy / 2 + 0.0015 * nin[0], z0 + dz / 2 + 0.0015 * nin[1])
    out.append(mk.box((cw, ln, 0.003), at=(x, c[0], c[1]),
                      quat=mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.atan2(dz, dy)))))
    return out


def _rotor_boxes(o, top):
    """Rotor of a lampholder, frame at the seat (tube end-face centre); o = +1 outward is +y (left), -1 (right).
    Two cheeks either side of a 4 mm slot open at the top, the slot floor under the lower pin."""
    cw, depth, h = 0.008, 0.012, 0.030
    out = []
    for sx in (-1, 1):
        out.append(mk.box((cw, depth, top + h / 2), at=(sx * (SLOT / 2 + cw / 2), o * depth / 2, (top - h / 2) / 2)))
    floor = h / 2 - (PIN_HALF + PIN_D / 2)
    out.append(mk.box((SLOT, depth, floor), at=(0, o * depth / 2, -h / 2 + floor / 2)))
    return out


def _housing_boxes(o, lift=0.0, guard=False):
    """The lampholder body behind the rotor and its foot on the channel; frame at the seat. A slotted guard over
    the rotor has its inner edge bevelled 78 deg (5 mm in over 23.5 mm): a tube end cap lowered onto the sprung
    holder's bevel cams it outward."""
    out = [mk.box((0.024, 0.010, 0.036), at=(0, o * 0.017, 0.003)),
           mk.box((0.024, 0.022, Z_AX - 0.015 - ZP - lift), at=(0, o * 0.011, -0.015 - (Z_AX - 0.015 - ZP - lift) / 2))]
    if guard:
        for sx in (-1, 1):
            out += _bevel(o, sx * (SLOT / 2 + 0.007), 0.014, 0.0075, 0.031, 0.005)
    return out


kit = mk.Kit()
kit.part("holder_l", (AX, YF, Z_AX), _housing_boxes(1, guard=True), mass=None, color=WHITE)
kit.part("rotor_l", (AX, YF, Z_AX), _rotor_boxes(1, 0.009), mass=0.02, color=IVORY)
kit.hinge("turn_l", "holder_l", "rotor_l", anchor=(AX, YF, Z_AX), axis=(0, 1, 0), lower_deg=0.0, upper_deg=95.0,
          stiffness=0.02, damping=0.0005)
kit.part("holder_r", (AX, YR0, Z_AX), _housing_boxes(-1, lift=0.0008, guard=True), mass=0.05, color=WHITE)
kit.slider("slide_r", None, "holder_r", axis=(0, -1, 0), lower=0.0, upper=SLIDE_TRAVEL, stiffness=SPRING_K,
           rest=-SPRING_PRELOAD / SPRING_K, damping=2.0)
kit.part("rotor_r", (AX, YR0, Z_AX), _rotor_boxes(-1, 0.009), mass=0.02, color=IVORY)
kit.hinge("turn_r", "holder_r", "rotor_r", anchor=(AX, YR0, Z_AX), axis=(0, 1, 0), lower_deg=0.0, upper_deg=95.0,
          stiffness=0.02, damping=0.0005)
kit.part("tube", TUBE0, _tube_boxes(), mass=TUBE_M, color=GLASS)

SEAT_T = mk._T(None, (AX, 0.0, Z_AX))  # the tube seated, pins upright
kit.catch("pins_l", "tube", SEAT_T, parent="rotor_l", parent_T=kit.parts["rotor_l"].ref)
kit.catch("pins_r", "tube", SEAT_T, parent="rotor_r", parent_T=mk._T(None, (0, -COMP, 0)) @ kit.parts["rotor_r"].ref)

FIXTURES = [
    Box("channel", pos=(AX, 0, TOP + 0.003), size=(0.120, 0.640, 0.006), color=WHITE),
    Box("lip_near", pos=(AX - 0.058, 0, ZP + 0.006), size=(0.004, 0.640, 0.012), color=WHITE),
    Box("lip_far", pos=(AX + 0.058, 0, ZP + 0.006), size=(0.004, 0.640, 0.012), color=WHITE),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules ------------------------------------------------------------------------------------------------------
SEAT_TOL, SEAT_DEG = 0.0015, 8.0  # end-face centre within 1.5 mm of the seat; pins within 8 deg of the slot
LOCK_DEG = 85.0
SLIP_DEG = 5.0
DROP_M = 0.10
NEAR = 0.035  # a TCP this close to the tube axis (pad within ~2 cm of the glass) counts as touching it


def _tube_R(objects):
    return mk.quat_to_mat(kit.pose(objects, "tube")[1])


def _end(objects, sgn):
    return kit.point(objects, "tube", (0, sgn * L_T / 2, 0))


def pins_in_slot(objects, side) -> bool:
    """That end's face centre within 1.5 mm of its holder's seat, the tube along the holder axis (1 deg) and both
    pins within 8 deg of the slot's direction (the rotor's z)."""
    rotor = f"rotor_{side}"
    e = _end(objects, 1 if side == "l" else -1)
    s = kit.point(objects, rotor, (0, 0, 0))
    Rt = _tube_R(objects)
    Rr = mk.quat_to_mat(kit.pose(objects, rotor)[1])
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(Rt[:, 2] @ Rr[:, 2])))))
    a, b = np.asarray(_end(objects, 1), float), np.asarray(_end(objects, -1), float)
    along = math.degrees(math.atan2(math.hypot(a[0] - b[0], a[2] - b[2]), abs(a[1] - b[1])))
    d = np.asarray(e, float) - np.asarray(s, float)
    return bool(np.all(np.abs(d) <= SEAT_TOL) and ang <= SEAT_DEG and along <= 1.0)


def _axis_dist(objects, p):
    c = np.asarray(kit.point(objects, "tube", (0, 0, 0)), float)
    a = _tube_R(objects)[:, 1]
    v = np.asarray(p, float) - c
    t = max(-L_T / 2, min(L_T / 2, float(v @ a)))
    return float(np.linalg.norm(v - t * a))


_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def touched(objects) -> bool:
    return any(_axis_dist(objects, t) < NEAR for t in _tcps())


def _seat_rule(objects):
    m = mk.MEMORY
    if not m.get("in_l") and pins_in_slot(objects, "l") and pins_in_slot(objects, "r"):
        for side in ("l", "r"):  # both pin pairs at their slot floors: the rotors now carry the tube
            kit.engage(f"pins_{side}")
            m[f"in_{side}"] = True
    for side in ("l", "r"):  # rotor detents at 0 and 90 deg
        det = DETENT_DEG if kit.q_deg(objects, f"turn_{side}") >= 45.0 else 0.0
        if m.get(f"det_{side}") != det:
            m[f"det_{side}"] = det
            kit.drive(f"turn_{side}", det)


def _latch_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    z = float(kit.point(objects, "tube", (0, 0, 0))[2])
    if m.get("in_l") or m.get("in_r"):
        m.pop("free_top", None)
    elif any(_axis_dist(objects, t) < 0.05 for t in _tcps()):
        m.pop("free_top", None)
    else:
        top = max(m.get("free_top", z), z)
        m["free_top"] = top
        if top - z > DROP_M:
            m["dropped"] = True
    hist = m.setdefault("hist", [])  # tube centre and rotor angle over the last 0.5 s, for stillness
    hist.append((tuple(float(v) for v in kit.point(objects, "tube", (0, 0, 0))), kit.q_deg(objects, "turn_l")))
    del hist[:-11]
    if z < TOP - 0.05:
        m["floor"] = True
    if abs(kit.q_deg(objects, "turn_l") - kit.q_deg(objects, "turn_r")) > SLIP_DEG and (m.get("in_l") or m.get("in_r")):
        m["slip"] = True  # one end turned and the other not: the tube would be twisted between its ends


def _debug_rule(objects):
    if os.environ.get("BIPIN_DEBUG") and mk.MEMORY.get("n", 0) % 20 == 0:
        f = lambda v: "(" + ",".join(f"{float(c):.4f}" for c in v) + ")"
        print(f"[bipin] n={mk.MEMORY.get('n')} tube={f(kit.point(objects, 'tube', (0, 0, 0)))} "
              f"endL={f(_end(objects, 1))} seatL={f(kit.point(objects, 'rotor_l', (0, 0, 0)))} "
              f"endR={f(_end(objects, -1))} seatR={f(kit.point(objects, 'rotor_r', (0, 0, 0)))} "
              f"slide={kit.q(objects, 'slide_r'):.4f} hold={f(kit.point(objects, 'holder_r', (0, 0, 0)))} turnL={kit.q_deg(objects, 'turn_l'):.1f} "
              f"turnR={kit.q_deg(objects, 'turn_r'):.1f} pin={pins_in_slot(objects, 'l')},{pins_in_slot(objects, 'r')} touched={touched(objects)} "
              f"v={np.linalg.norm(objects['tube'].lin_vel):.4f},{np.linalg.norm(objects['tube'].ang_vel):.3f} tcp={[f(t) for t in mk.tcps()]} mem={ {k: v for k, v in mk.MEMORY.items() if k != 'n'} } score={score(objects)}", flush=True)


kit.on_step(_seat_rule)
kit.on_step(_latch_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


# -- scoring ----------------------------------------------------------------------------------------------------

def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("dropped") or m.get("floor") or m.get("slip"))


def seated(objects) -> bool:
    return bool(mk.MEMORY.get("in_l") and mk.MEMORY.get("in_r"))


def locked(objects) -> bool:
    return seated(objects) and all(kit.q_deg(objects, f"turn_{s}") >= LOCK_DEG for s in ("l", "r"))


def still(objects) -> bool:
    """Over the last 0.5 s the tube centre moved under 2.5 mm and the rotors under 3 deg (5 mm/s, 6 deg/s)."""
    h = mk.MEMORY.get("hist")
    if h is None:  # offline states: use the reported velocities
        return objects["tube"].resting(max_speed=0.005, max_spin=0.1)
    if len(h) < 11:
        return False
    return (max(math.dist(h[-1][0], p) for p, _ in h) < 0.0025
            and max(abs(h[-1][1] - a) for _, a in h) < 3.0)


def score(objects) -> float:
    """0 if failed() (dropped 10 cm, on the floor, ends turned apart, or let go seated but unturned). Otherwise
    both pin pairs in their slots 0.3; plus both rotors at the 90 deg detent 0.5; plus let go (no TCP within
    3.5 cm of the tube axis) with the tube still 0.2."""
    if failed(objects) or not seated(objects):
        return 0.0
    if not locked(objects) and not touched(objects):
        return 0.0  # let go with the pins only sitting in open slots: the tube falls out
    s = 0.3
    if locked(objects):
        s += 0.5
        if not touched(objects) and still(objects):
            s += 0.2
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(math.dist([float(v) for v in kit.point(objects, "tube", (0, 0, 0))], [AX, 0.0, Z_AX]))


# -- the scripted demo ------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = 0.880
F_OPEN, F_GRIP = -0.024, 0.0
GY = 0.200  # grasp points, y = +-GY on the tube
GYAW = math.radians(85.0)  # fingers close 30 deg off the x axis (fingers close 5 deg off the x axis)
GDZ = 0.006  # TCP above the tube axis while gripping
DX_L, DX_R, DZ_SAG = 0.0055, -0.0055, 0.0  # hand corrections for the load (tuned in sim)
Y_AIM = -0.0005
PRESS_Y = 0.265  # fingertips pressed on the glass 3 cm in from each end cap  # tube centre while lowering: fixed-end face 0.5 mm clear of its holder


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
    arms = {"left": (_Seq(), 1, GYAW, DX_L), "right": (_Seq(), -1, -GYAW, DX_R)}
    x0, z0 = TUBE0[0], TUBE0[2]
    for s, sg, yaw, dx in arms.values():
        y = sg * GY
        s.go(2.5, (x0, y, TRAVEL), F_OPEN, "over the tube", yaw)
        s.go(1.5, (x0, y, z0 + GDZ), F_OPEN, "down either side of it", yaw)
        s.go(1.0, (x0, y, z0 + GDZ), F_GRIP, "grip", yaw)
        s.go(0.5, (x0, y, z0 + GDZ), F_GRIP, "settle", yaw)
        s.go(1.5, (x0, y, z0 + 0.06), F_GRIP, "lift", yaw)
        if variant == "drop":
            s.go(2.0, (x0, y, z0 + 0.15), F_GRIP, "lift high", yaw)
            s.go(0.6, (x0, y, z0 + 0.15), F_OPEN, "let go", yaw)
            s.go(2.0, (x0, y, TRAVEL), F_OPEN, "up", yaw)
            continue
        zh = Z_AX + 0.065
        s.go(2.5, (AX + dx, y + Y_AIM, zh + DZ_SAG), F_GRIP, "over the holders", yaw)
        s.go(1.0, (AX + dx, y + Y_AIM, zh + DZ_SAG), F_GRIP, "settle", yaw)
        s.go(3.0, (AX + dx, y + Y_AIM, Z_AX + GDZ - 0.002 + DZ_SAG), F_GRIP, "pins down the slots, pressing the "
               "sprung holder out", yaw)
        s.go(0.8, (AX + dx, y + Y_AIM, Z_AX + GDZ - 0.002 + DZ_SAG), F_GRIP, "settle", yaw)
        s.go(0.8, (AX + dx, y + Y_AIM, Z_AX + GDZ - 0.002 + DZ_SAG), F_OPEN, "let go" if variant == "no_turn"
               else "open", yaw)
        s.go(1.2, (AX + dx, y, Z_AX + 0.06), F_OPEN, "up", yaw)
        # the sprung holder holds the tube's right end up on its bevel: press the tube down with closed fingertips
        zr = Z_AX + R_T - 0.003 + TIP
        yp = sg * PRESS_Y
        s.go(1.5, (AX, yp, zr + 0.05), F_GRIP, "over the tube end, fingers closed", 0.0)
        s.go(3.0, (AX, yp, zr - 0.004), F_GRIP, "press it down into the slots", 0.0)
        s.go(1.0, (AX, yp, zr - 0.004), F_GRIP, "hold", 0.0)
        s.go(0.8, (AX, yp, zr + 0.02), F_GRIP, "up", 0.0)
        if variant == "no_turn":
            s.go(1.0, (AX, y, zr + 0.04), F_GRIP, "up", 0.0)
            s.go(2.0, (0.20, sg * 0.30, TRAVEL), F_OPEN, "park", 0.0)
            continue
        # a quarter turn, both hands together: closed fingertips pressed on top of the glass, swept +x
        stroke = 0.032 if variant != "half_turn" else 0.010
        zp = zr - 0.006
        for k in range(1 if variant == "half_turn" else 4):  # a few sweeps; each turns it some 30-40 deg
            s.go(1.0, (AX - 0.002, y, zr + 0.01), F_GRIP, "back, over the glass", 0.0)
            s.go(1.0, (AX - 0.002, y, zp), F_GRIP, "press on the glass", 0.0)
            s.go(2.5, (AX - 0.002 + stroke, y, zp), F_GRIP, "roll it on toward a quarter turn", 0.0)
            s.go(0.6, (AX - 0.002 + stroke, y, zr + 0.01), F_GRIP, "up", 0.0)
        s.go(0.8, (AX - 0.002 + stroke, y, zr + 0.04), F_GRIP, "up", 0.0)
        s.go(2.0, (0.20, sg * 0.30, TRAVEL), F_GRIP, "park", 0.0)
    return {k: v[0].w for k, v in arms.items()}


def demo():
    # DEMO_VARIANT: no_turn | half_turn | drop (degenerate checks; all score 0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------

def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])
    seat = (np.array([AX, 0.0, Z_AX]), I)

    def st(q=None, **moved):
        return kit.fake_states(q=q, moved=moved)

    turned = {"turn_l": math.radians(90), "turn_r": math.radians(90), "slide_r": COMP}
    rot90 = mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(90)))
    done = st(turned, tube=(seat[0], rot90))
    held = st({"slide_r": COMP}, tube=seat)
    near = [np.array([AX, 0.2, Z_AX + 0.02])]
    S = {"in_l": True, "in_r": True}
    cases = {
        "start": (st(), [], {}, 0.0),
        "seated, held": (held, near, S, 0.3),
        "seated, let go unturned": (held, [], S, 0.0),
        "turned, held": (done, near, S, 0.8),
        "turned, let go": (done, [], S, 1.0),
        "one end seated, turned": (st({"turn_l": math.radians(90)}), [], {"in_l": True, "slip": True}, 0.0),
        "dropped": (done, [], {**S, "dropped": True}, 0.0),
        "half turned, let go": (st({"turn_l": math.radians(40), "turn_r": math.radians(40), "slide_r": COMP}),
                                [], S, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert pins_in_slot(held, "l") and pins_in_slot(held, "r")
        assert not pins_in_slot(st(), "l")
        assert not pins_in_slot(st({"slide_r": COMP}, tube=(seat[0] + np.array([0, 0, 0.003]), I)), "l")
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
    for v in ("", "no_turn", "half_turn", "drop"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
