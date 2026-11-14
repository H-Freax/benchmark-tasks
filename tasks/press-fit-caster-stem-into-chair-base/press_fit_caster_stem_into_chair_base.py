"""An office-chair 5-star base (scaled to a 0.38 m span) lies upside down on the bench, resting on its hub, its five
arm tips 30 mm clear of the bench top with their caster sockets facing up. Five twin-wheel casters stand stem-down
in two parts racks, three on the left and two on the right.

The job: fit the five casters. Each 11 mm stem carries a split ring: set into its socket the stem drops 6 mm and
stops on the ring. It goes home (22 mm) only when pushed with 35 N or more along the socket axis (within 5 deg).
A sideways load above 8 N at the ring cams it back out of the bore; a push more than 12 deg off the axis under more
than 30 N bends the stem. The base rests on its hub alone, so a hard push on one arm tip rocks the base and tilts
that socket off the line of the push: the other hand has to hold the opposite side of the base down.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.
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

NYLON = (0.12, 0.12, 0.13)  # black glass-filled nylon base
STEEL = (0.70, 0.71, 0.73)
WHEEL = (0.22, 0.23, 0.25)
RACK = (0.80, 0.55, 0.20)

# -- base: frame at the hub's centre on the bench top ---------------------------------------------------------------
C = (0.262, 0.0)
ARMS_DEG = (180.0, 252.0, 324.0, 36.0, 108.0)  # arm k points this way (0 = +x, away from the robot)
RS = 0.165  # socket axis radius
HUB, HUB_H, WELL = 0.072, 0.060, 0.028  # hub 72 mm square, 60 mm tall, column bore 28 mm
ARM_Z0, ARM_Z1, ARM_W = 0.030, 0.058, 0.030
BOSS, BORE = 0.034, 0.015
BOSS_Z0, BOSS_TOP = 0.030, 0.062
PRE = 0.006  # the stem enters 6 mm before it lands on its split ring
FULL = 0.022  # seated stem length in the bore
PLATE_T = 0.003
BASE_MASS = 2.0


def socket_xy(k):
    a = math.radians(ARMS_DEG[k])
    return (RS * math.cos(a), RS * math.sin(a))  # in the base frame


def _base_boxes():
    b = []
    w = (HUB - WELL) / 2
    b.append(mk.box((HUB, HUB, 0.015), at=(0, 0, 0.0075)))
    for sx, sy in ((1, 0), (-1, 0)):
        b.append(mk.box((w, HUB, HUB_H - 0.015), at=(sx * (WELL + w) / 2, 0, (0.015 + HUB_H) / 2)))
    for sy in (1, -1):
        b.append(mk.box((WELL, w, HUB_H - 0.015), at=(0, sy * (WELL + w) / 2, (0.015 + HUB_H) / 2)))
    r0, r1 = HUB / 2 - 0.002, RS - BOSS / 2 + 0.002
    for k, a in enumerate(ARMS_DEG):
        ar = math.radians(a)
        rc = (r0 + r1) / 2
        b.append(mk.box((r1 - r0, ARM_W, ARM_Z1 - ARM_Z0), at=(rc * math.cos(ar), rc * math.sin(ar),
                                                                 (ARM_Z0 + ARM_Z1) / 2), yaw=ar))
        x, y = socket_xy(k)
        t = (BOSS - BORE) / 2
        zf = BOSS_TOP - FULL - PLATE_T - 0.001  # bore floor
        b.append(mk.box((BOSS, BOSS, zf - BOSS_Z0), at=(x, y, (BOSS_Z0 + zf) / 2)))
        h = BOSS_TOP - zf
        for s in (1, -1):
            b.append(mk.box((t, BOSS, h), at=(x + s * (BORE + t) / 2, y, (zf + BOSS_TOP) / 2)))
            b.append(mk.box((BORE, t, h), at=(x, y + s * (BORE + t) / 2, (zf + BOSS_TOP) / 2)))
    return b


# -- casters: frame at the bottom of the stem, wheel up ---------------------------------------------------------
STEM, STEM_L = 0.011, 0.022
PLATE_W, PLATE_Z1 = 0.036, 0.027  # swivel plate, 5 mm thick, a 5 mm ledge each side of the wheels
WHEEL_X, WHEEL_Y, WHEEL_Z1 = 0.040, 0.026, 0.067
CASTER_MASS = 0.07


def _caster_boxes():
    return [mk.box((STEM, STEM, STEM_L), at=(0, 0, STEM_L / 2), color=STEEL),
            mk.box((PLATE_W, PLATE_W, PLATE_Z1 - STEM_L), at=(0, 0, (STEM_L + PLATE_Z1) / 2), color=STEEL),
            mk.box((WHEEL_X, WHEEL_Y, WHEEL_Z1 - PLATE_Z1), at=(0, 0, (PLATE_Z1 + WHEEL_Z1) / 2), color=WHEEL)]


RACK_XY = [(0.200, 0.330), (0.275, 0.330), (0.350, 0.330), (0.230, -0.330), (0.305, -0.330)]
RACK_H = 0.018

kit = mk.Kit()
BASE0 = (C[0], C[1], TOP)
kit.part("base", BASE0, _base_boxes(), mass=BASE_MASS, color=NYLON)

# Each socket's split ring is modelled as a sprung stop under the stem: it holds the stem 6 mm in, preloaded to
# 12 N; it gives 1 mm per 6 N, so 35 N is 3.8 mm of give. Once the ring passes, the stop drops away to the bore floor.
RING_K, RING_PRE = 6000.0, 0.002
PASS_N, BEND_N, CAM_N = 35.0, 30.0, 8.0
AXIS_DEG, BEND_DEG = 5.0, 12.0
for k in range(5):
    x, y = socket_xy(k)
    z = BOSS_TOP - PRE - PLATE_T
    kit.part(f"ring_{k}", (C[0] + x, C[1] + y, TOP + z), [mk.box((BORE - 0.002, BORE - 0.002, PLATE_T),
                                                                 at=(0, 0, PLATE_T / 2))], mass=0.2, color=STEEL)
    kit.slider(f"ring_{k}_give", "base", f"ring_{k}", axis=(0, 0, -1), lower=0.0, upper=FULL - PRE,
               stiffness=RING_K, rest=-RING_PRE, damping=30.0)
for i, (x, y) in enumerate(RACK_XY):
    kit.part(f"caster_{i}", (x, y, TOP), _caster_boxes(), mass=CASTER_MASS, color=STEEL)


def _seat_T(k):
    x, y = socket_xy(k)
    return mk._T(None, (C[0] + x, C[1] + y, TOP + BOSS_TOP - FULL))


BASE_T0 = mk._T(None, BASE0)
for i in range(5):
    for k in range(5):
        kit.catch(f"seat_{i}_{k}", f"caster_{i}", _seat_T(k), "base", BASE_T0)


def _rack(i):
    x, y = RACK_XY[i]
    o = 0.017
    t = 0.006
    return [Box(f"rack{i}_a", pos=(x + o, y, TOP + RACK_H / 2), size=(t, 2 * o + t, RACK_H), color=RACK),
            Box(f"rack{i}_b", pos=(x - o, y, TOP + RACK_H / 2), size=(t, 2 * o + t, RACK_H), color=RACK),
            Box(f"rack{i}_c", pos=(x, y + o, TOP + RACK_H / 2), size=(2 * o - t, t, RACK_H), color=RACK),
            Box(f"rack{i}_d", pos=(x, y - o, TOP + RACK_H / 2), size=(2 * o - t, t, RACK_H), color=RACK)]


FIXTURES = [b for i in range(5) for b in _rack(i)]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules ------------------------------------------------------------------------------------------------------


def _rel(objects, part):
    """Pose of ``part`` in the base frame (4x4)."""
    return mk._inv(kit._pose_T(objects, "base")) @ kit._pose_T(objects, part)


def _in_socket(objects, i):
    """Socket k whose bore caster i's stem tip is in (within 6 mm of the axis, below the boss top), else None."""
    r = _rel(objects, f"caster_{i}")
    p = r[:3, 3]
    for k in range(5):
        x, y = socket_xy(k)
        if math.hypot(p[0] - x, p[1] - y) < 0.006 and p[2] < BOSS_TOP + 0.001:
            return k
    return None


def off_axis_deg(objects, i):
    r = _rel(objects, f"caster_{i}")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def base_tilt_deg(objects):
    z = kit._pose_T(objects, "base")[:3, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def ring_force(objects, k):
    """Axial load on ring k (N): the sprung stop moves only above its 12 N preload."""
    q = kit.q(objects, f"ring_{k}_give")
    return RING_K * (q + RING_PRE) if q > 0.0002 else 0.0


def _ring_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    passed = m.setdefault("passed", {})  # socket -> caster
    bent = m.setdefault("bent", set())
    cammed = m.setdefault("cammed", set())
    for i in range(5):
        k = _in_socket(objects, i)
        if k is None:
            continue
        f, tilt = ring_force(objects, k), off_axis_deg(objects, i)
        # the push is taken as straight down: a socket rocked off vertical loads the ring off its axis too
        load = max(tilt, base_tilt_deg(objects))
        if k in passed:
            if passed[k] == i and not m.get(f"seat_{k}"):
                d = float(_rel(objects, f"caster_{i}")[2, 3])
                if d <= BOSS_TOP - FULL + 0.002 and tilt <= 3.0:
                    kit.engage(f"seat_{i}_{k}")
                    m[f"seat_{k}"] = i
            continue
        if f > BEND_N and tilt > BEND_DEG:
            bent.add(i)
        if f * math.sin(math.radians(load)) > CAM_N:
            cammed.add(k)  # the ring has ridden out of its groove: the push has to be taken off and made again
        if k in cammed and f <= 0.0:
            cammed.discard(k)
        if i not in bent and k not in cammed and f >= PASS_N and load <= AXIS_DEG:
            passed[k] = i
            kit.drive(f"ring_{k}_give", FULL)


def _base_rule(objects):
    p, _ = kit.pose(objects, "base")
    z = kit._pose_T(objects, "base")[:3, 2]
    if float(p[2]) < TOP - 0.03 or math.degrees(math.acos(max(-1.0, min(1.0, float(z[2]))))) > 45.0 \
            or math.hypot(float(p[0]) - C[0], float(p[1]) - C[1]) > 0.10:
        mk.MEMORY["base_off"] = True
    for i in range(5):
        if float(kit.pose(objects, f"caster_{i}")[0][2]) < TOP - 0.05:
            mk.MEMORY.setdefault("floor", set()).add(i)


def _debug_rule(objects):
    if os.environ.get("CASTER_DEBUG") and mk.MEMORY.get("n", 0) % 10 == 0:
        m = mk.MEMORY
        z = kit._pose_T(objects, "base")[:3, 2]
        bt = math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))
        socks = " ".join(f"{k}:{ring_force(objects, k):.0f}N" for k in range(5))
        cs = " ".join(f"c{i}:{_in_socket(objects, i)}/{off_axis_deg(objects, i):.1f}d"
                      f"/{float(_rel(objects, f'caster_{i}')[2, 3]) * 1000:.1f}" for i in range(5))
        print(f"[caster] n={m['n']} base_tilt={bt:.1f} {socks} {cs} passed={m.get('passed')} bent={m.get('bent')} "
              f"cam={m.get('cammed')} score={score(objects)}", flush=True)


kit.on_step(_ring_rule)
kit.on_step(_base_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None


def seated(objects, i, k) -> bool:
    """Caster i went past socket k's ring (35 N within 5 deg) and its stem is home: within 2 mm of full depth,
    within 2 mm of the socket axis and 3 deg of it."""
    m = mk.MEMORY
    if m.get("passed", {}).get(k) != i or i in m.get("bent", set()):
        return False
    r = _rel(objects, f"caster_{i}")
    x, y = socket_xy(k)
    return (abs(float(r[2, 3]) - (BOSS_TOP - FULL)) <= 0.002 and math.hypot(r[0, 3] - x, r[1, 3] - y) <= 0.002
            and off_axis_deg(objects, i) <= 3.0)


def _released(st, margin=0.015) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def fitted(objects):
    return [k for k in range(5) if any(seated(objects, i, k) for i in range(5))]


def failed(objects) -> bool:
    return bool(mk.MEMORY.get("base_off"))


def settled(objects) -> bool:
    return (all(_released(objects[f"caster_{i}"]) for i in range(5))
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("base", *[f"caster_{i}" for i in
                                                                                       range(5)])))


def score(objects) -> float:
    """0 if the base came off the bench (or tipped over). Otherwise 0.2 for each socket holding a caster whose stem
    went past the ring and is home, unbent; counted only with every caster let go and everything still."""
    if failed(objects) or not settled(objects):
        return 0.0
    return round(0.2 * len(fitted(objects)), 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    d = 0.0
    for i in range(5):
        p = _rel(objects, f"caster_{i}")[:3, 3]
        d += min(math.dist([float(v) for v in p], [*socket_xy(k), BOSS_TOP - FULL]) for k in range(5))
    return d


# -- the scripted demo ------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = 0.880
F_OPEN, F_PINCH = -0.024, -0.006  # wheels 26 mm wide; the pinch squeezes to the gripper's force cap
GRIP_DZ = PLATE_Z1 + 0.002 + TIP  # TCP above the stem tip: pad tips 2 mm over the swivel plate's ledge
CARRY = TOP + GRIP_DZ + 0.070  # a carried caster clears the wheels of fitted ones
PUSH = float(os.environ.get("CASTER_PUSH", "0.075"))  # TCP commanded this far below the touch point: >35 N
BRACE = float(os.environ.get("CASTER_BRACE", "0.070"))  # the bracing fingertips commanded this far into the arm
LEFT_SOCKETS = (0, 4, 3)
RIGHT_SOCKETS = (1, 2)
LEFT_CASTERS = (0, 1, 2)
RIGHT_CASTERS = (3, 4)
# brace points: on an arm across the hub from the pushed tip, 110 mm out from the centre
BRACE_ON = {4: 2, 3: 1, 0: 2, 1: 4, 2: 4}
BRACE_R = 0.150


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


def _sock_w(k):
    x, y = socket_xy(k)
    return (C[0] + x, C[1] + y)


def _brace_w(k):
    a = math.radians(ARMS_DEG[BRACE_ON[k]])
    return (C[0] + BRACE_R * math.cos(a), C[1] + BRACE_R * math.sin(a))


def _fetch(s, i, k, park, push=True, target=None):
    """Pick caster i from its rack and set it in socket k (or over ``target``: world xy, bottom z); returns the time
    the push starts, or None."""
    rx, ry = RACK_XY[i]
    s.go(2.0, (rx, ry, TRAVEL), F_OPEN, f"over caster {i}")
    s.go(1.5, (rx, ry, TOP + GRIP_DZ), F_OPEN, "pads down round the wheels")
    s.go(0.8, (rx, ry, TOP + GRIP_DZ), F_PINCH, "pinch")
    s.go(1.2, (rx, ry, TOP + GRIP_DZ + 0.060), F_PINCH, "lift it out of the rack")
    s.go(1.0, (rx, ry, CARRY), F_PINCH, "up, clear of the fitted casters")
    if target is not None:
        tx, ty, tz = target
        s.go(2.0, (tx, ty, CARRY), F_PINCH, "over the hub's bore")
        s.go(1.5, (tx, ty, tz + GRIP_DZ + 0.040), F_PINCH, "down to it")
        s.go(1.0, (tx, ty, tz + GRIP_DZ + 0.005), F_PINCH, "down into it")
        s.go(0.6, (tx, ty, tz + GRIP_DZ + 0.005), F_OPEN, "let go")
        s.go(1.0, (tx, ty, TRAVEL), F_OPEN, "up")
        s.go(1.5, (*park, TRAVEL), F_OPEN, "out of the way")
        return None
    sx, sy = _sock_w(k)
    z0 = TOP + BOSS_TOP - PRE + GRIP_DZ
    s.go(2.2, (sx, sy, CARRY), F_PINCH, f"over socket {k}")
    s.go(1.5, (sx, sy, z0 + 0.040), F_PINCH, "down to it")
    s.go(2.0, (sx, sy, z0 + 0.008), F_PINCH, "settle over it")
    s.go(1.2, (sx, sy, z0 + 0.001), F_PINCH, "stem into the bore, onto the ring")
    t = s.t
    if not push:
        s.go(0.6, (sx, sy, z0 + 0.001), F_OPEN, "let go")
        s.go(1.0, (sx, sy, TRAVEL), F_OPEN, "up")
        s.go(1.5, (*park, TRAVEL), F_OPEN, "out of the way")
        return t
    return t


def _push(s, k, park):
    sx, sy = _sock_w(k)
    z0 = TOP + BOSS_TOP - PRE + GRIP_DZ
    s.go(2.5, (sx, sy, z0 - PUSH), F_PINCH, "push it home along the socket")
    s.go(1.5, (sx, sy, z0 - PUSH), F_PINCH, "hold")
    s.go(0.4, (sx, sy, z0 - 0.012), F_PINCH, "ease off")
    s.go(0.6, (sx, sy, z0 - 0.012), F_OPEN, "let go")
    s.go(1.0, (sx, sy, TRAVEL), F_OPEN, "up")
    s.go(1.5, (*park, TRAVEL), F_OPEN, "out of the way")


def _brace(s, k, t_from, t_to, park):
    """Closed fingertips on the arm across the hub from socket k; press it down as the other hand pushes."""
    bx, by = _brace_w(k)
    z = TOP + ARM_Z1 + TIP
    s.until(t_from - 3.0)
    s.go(2.0, (bx, by, z + 0.040), 0.0, f"over arm {BRACE_ON[k]}")
    s.go(1.0, (bx, by, z + 0.001), 0.0, "fingertips on the arm")
    s.until(t_from, "wait for the push")
    s.go(2.5, (bx, by, z - BRACE), 0.0, "hold the base down as the push builds")
    s.until(t_to, "hold it")
    s.go(0.6, (bx, by, z + 0.010), 0.0, "ease off")
    s.go(1.2, (*park, TRAVEL), 0.0, "out of the way")


L_PARK, R_PARK = (0.12, 0.30), (0.12, -0.30)
HUB_TARGET = (C[0], C[1], TOP + 0.015)


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    L.go(1.0, (*L_PARK, TRAVEL), F_OPEN, "start")
    R.go(1.0, (*R_PARK, TRAVEL), F_OPEN, "start")
    if variant == "hub_hole":  # every caster dropped into the hub's column bore instead of an arm socket
        for i in LEFT_CASTERS:
            _fetch(L, i, None, L_PARK, target=HUB_TARGET)
        R.until(L.t)
        for i in RIGHT_CASTERS:
            _fetch(R, i, None, R_PARK, target=HUB_TARGET)
        L.until(R.t + 3.0)
        R.until(R.t + 3.0)
        return {"left": L.w, "right": R.w}
    loose = variant == "loose"
    brace = variant != "no_brace"
    for i, k in zip(LEFT_CASTERS, LEFT_SOCKETS):
        L.until(R.t)
        t = _fetch(L, i, k, L_PARK, push=not loose)
        if loose:
            continue
        if brace:
            _brace(R, k, t + 0.2, t + 0.2 + 2.5 + 1.5 + 0.2, R_PARK)
            L.until(t + 0.2)
        _push(L, k, L_PARK)
    for i, k in zip(RIGHT_CASTERS, RIGHT_SOCKETS):
        R.until(L.t)
        t = _fetch(R, i, k, R_PARK, push=not loose)
        if loose:
            continue
        if brace:
            _brace(L, k, t + 0.2, t + 0.2 + 2.5 + 1.5 + 0.2, L_PARK)
            R.until(t + 0.2)
        _push(R, k, R_PARK)
    end = max(L.t, R.t) + 3.0
    L.until(end, "rest")
    R.until(end, "rest")
    return {"left": L.w, "right": R.w}


def demo():
    # DEMO_VARIANT: loose | hub_hole | no_brace (degenerate checks; all score 0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])

    def seat(k, dz=0.0, dxy=0.0):
        x, y = socket_xy(k)
        return (np.array([C[0] + x + dxy, C[1] + y, TOP + BOSS_TOP - FULL + dz]), I)

    def st(**moved):
        return kit.fake_states(moved=moved)

    all5 = {f"caster_{i}": seat(k) for i, k in enumerate((0, 1, 2, 3, 4))}
    P = {"passed": {k: k for k in range(5)}}
    tilt = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(5)))
    cases = {
        "start": (st(), [], {}, 0.0),
        "all home": (st(**all5), [], P, 1.0),
        "all home, a hand still on one": (st(**all5), [kit.point(st(**all5), "caster_0", (0, 0, 0.04))], P, 0.0),
        "two home": (st(caster_0=seat(0), caster_1=seat(1)), [], {"passed": {0: 0, 1: 1}}, 0.4),
        "all at the right depth, rings never passed": (st(**all5), [], {}, 0.0),
        "loose at the ring": (st(**{f"caster_{i}": seat(i, 0.016) for i in range(5)}), [], {}, 0.0),
        "passed but 4 mm short": (st(**{f"caster_{i}": seat(i, 0.004) for i in range(5)}), [], P, 0.0),
        "one bent": (st(**all5), [], {**P, "bent": {2}}, 0.8),
        "one tilted 5 deg": (st(**{**all5, "caster_3": (seat(3)[0], tilt)}), [], P, 0.8),
        "one 3 mm off axis": (st(**{**all5, "caster_4": seat(4, 0, 0.003)}), [], P, 0.8),
        "base slid off": (st(**all5), [], {**P, "base_off": True}, 0.0),
        "caster credited to another socket": (st(**all5), [], {"passed": {0: 1, 1: 0, 2: 2, 3: 3, 4: 4}}, 0.6),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        mk.new_episode()
        assert all(_in_socket(st(**all5), i) == i for i in range(5))
        assert _in_socket(st(), 0) is None
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
    for v in ("", "loose", "hub_hole", "no_brace"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
