"""A twin jumbo-roll toilet-tissue dispenser lies on its back on the bench, front up, its cover swung open on its
bottom-edge hinge (toward the right arm) and resting past upright on its stop. Inside are two spindles with sprung
split hubs: the primary (left) still carries an almost used-up stub; the reserve (right) is empty and blocked by a
slide gate lever. A new 230 mm jumbo roll lies on the bench to the left, a waste bin stands in front of it.

The job: the stub goes onto the reserve spindle (so it is used up first), the new roll onto the primary, and the
cover is lowered shut until its latch clicks. Each hub only lets a core on or off while its release tab beside it
is held down (the sprung hub half retracts), so one hand holds the tab while the other slides the core along the
spindle. The cover (0.5 kg) has a soft-close damper: once pushed over it lowers itself; slammed, it counts as dropped.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 2000 \\
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

WHITE = (0.92, 0.92, 0.90)
SMOKE = (0.30, 0.32, 0.36)  # smoked cover
GREY = (0.55, 0.56, 0.58)
BLUE = (0.15, 0.35, 0.75)
RED = (0.85, 0.15, 0.12)
PAPER = (0.96, 0.95, 0.90)
CORE = (0.62, 0.48, 0.32)
BIN = (0.20, 0.22, 0.24)

# -- housing (back plate and a 30 mm rim) -------------------------------------------------------------------------
X0, X1, YH = 0.135, 0.435, 0.245
ZP = TOP + 0.006  # back plate face
RIM = ZP + 0.030  # rim top: the cover's hinge line and the hub tops
WALL = 0.004

# -- rolls ----------------------------------------------------------------------------------------------------------
R_I = 0.0375  # 75 mm core
ROLL_RO, STUB_RO, ROLL_H = 0.115, 0.0415, 0.090
PRIM, RES = (0.275, 0.120), (0.275, -0.120)  # hub axes: primary (left), reserve (right)
SEAT_DX = 0.0215  # a seated core sits pushed this far +x by the sprung hub half against the fixed half


def _ring(ro, h, color_out, n=12):
    """A 12-sided ring (core + paper) round a 75 mm core hole, frame at the bottom centre."""
    t = ro - R_I
    w = 2 * ro * math.tan(math.pi / n)
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = (ro + R_I) / 2
        out.append(mk.box((t, w, h), at=(rc * math.cos(a), rc * math.sin(a), h / 2), yaw=a, color=color_out))
    return out


kit = mk.Kit()

# hubs: a fixed half and a sprung half on a slider (+x side), each with a release tab beside it near the middle
JAW_TRAVEL, JAW_OPEN_AT = 0.048, 0.036
TAB_TRAVEL, TAB_PRESSED = 0.006, 0.004
HUBS = {"p": PRIM, "r": RES}
TABS = {"p": (0.150, 0.015), "r": (0.150, -0.015)}
for k, (hx, hy) in HUBS.items():
    kit.part(f"hub_fix_{k}", (hx, hy, ZP), [mk.box((0.015, 0.024, 0.030), at=(-0.0085, 0, 0.015))], mass=None,
             color=GREY)
    kit.part(f"jaw_{k}", (hx, hy, ZP), [mk.box((0.011, 0.024, 0.029), at=(0.0565, 0, 0.0155))], mass=0.020, color=GREY)
    kit.slider(f"jaw_{k}_slide", f"hub_fix_{k}", f"jaw_{k}", axis=(-1, 0, 0), lower=0.0, upper=JAW_TRAVEL,
               start=0.003 if k == "p" else 0.0, stiffness=500.0, rest=-0.004, damping=8.0)
    tx, ty = TABS[k]
    kit.part(f"tab_base_{k}", (tx, ty, ZP), [mk.box((0.018, 0.018, 0.010), at=(0, 0, 0.005))], mass=None, color=GREY)
    kit.part(f"tab_{k}", (tx, ty, ZP + 0.010), [mk.box((0.014, 0.014, 0.012), at=(0, 0, 0.006))], mass=0.020,
             color=BLUE)
    kit.slider(f"tab_{k}_push", f"tab_base_{k}", f"tab_{k}", axis=(0, 0, -1), lower=0.0, upper=TAB_TRAVEL,
               stiffness=400.0, rest=-0.002, damping=2.0)

# slide gate over the reserve spindle: a lever on a post, detents at blocking (0) and clear (90 deg, clockwise)
GATE = (0.290, -0.215)
GATE_Z = 0.787
kit.part("gate_post", (GATE[0], GATE[1], ZP), [mk.box((0.012, 0.012, GATE_Z - ZP), at=(0, 0, (GATE_Z - ZP) / 2))],
         mass=None, color=GREY)
kit.part("gate", (GATE[0], GATE[1], GATE_Z),
         [mk.box((0.014, 0.014, 0.008), at=(0, 0, 0.004)), mk.box((0.012, 0.080, 0.008), at=(0, 0.040, 0.004))],
         mass=0.030, color=RED)
kit.hinge("gate_turn", "gate_post", "gate", anchor=(GATE[0], GATE[1], GATE_Z), axis=(0, 0, -1), lower_deg=0.0,
          upper_deg=95.0, stiffness=0.05, damping=0.002)

# the stub on the primary hub (held by the hub until its tab is pressed) and the new roll on the bench
STUB0 = (PRIM[0] + SEAT_DX, PRIM[1], ZP)
NEW0 = (0.300, 0.400, TOP)
kit.part("stub", STUB0, _ring(STUB_RO, ROLL_H, PAPER), mass=0.15, color=PAPER)
kit.bond("stub_hold", "hub_fix_p", "stub", anchor=(STUB0[0], STUB0[1], ZP + 0.015))
kit.part("roll", NEW0, _ring(ROLL_RO, ROLL_H, PAPER), mass=0.90, color=PAPER)

# cover: a 300 x 490 mm shell 74 mm deep, hinged along the far rim; held open at 100 deg by a stay until its
# release button is pressed; then its closing spring and a rotary damper lower it onto its latch
HINGE = (X1, -YH, RIM)
CW, CL, CD = X1 - X0, 2 * YH, 0.074
OPEN_DEG = 100.0
_cover = [mk.box((CW, CL, 0.004), at=(-CW / 2, CL / 2, CD - 0.002)),
          mk.box((WALL, CL, CD - 0.005), at=(-WALL / 2, CL / 2, 0.001 + (CD - 0.005) / 2)),
          mk.box((WALL, CL, CD - 0.005), at=(-CW + WALL / 2, CL / 2, 0.001 + (CD - 0.005) / 2)),
          mk.box((CW, WALL, CD - 0.005), at=(-CW / 2, WALL / 2, 0.001 + (CD - 0.005) / 2)),
          mk.box((CW, WALL, CD - 0.005), at=(-CW / 2, CL - WALL / 2, 0.001 + (CD - 0.005) / 2)),
          ]
kit.part("cover", HINGE, _cover, mass=0.50, color=SMOKE)
kit.hinge("cover_hinge", None, "cover", anchor=HINGE, axis=(0, 1, 0), lower_deg=0.0, upper_deg=OPEN_DEG,
          start_deg=OPEN_DEG, stiffness=0.5, rest_deg=-10.0, damping=3.0)
kit.catch("cover_latch", "cover", kit.parts["cover"].ref)


def _open_pose():
    a = math.radians(OPEN_DEG)
    R = np.eye(4)
    R[:3, :3] = [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
    T0, T1 = np.eye(4), np.eye(4)
    T0[:3, 3], T1[:3, 3] = -np.asarray(HINGE), np.asarray(HINGE)
    return T1 @ R @ T0 @ kit.parts["cover"].ref


kit.catch("cover_stay", "cover", _open_pose())
# stay release button on a bracket outside the right rim
STAY = (0.250, -YH - 0.035)
kit.part("stay_base", (*STAY, ZP), [mk.box((0.024, 0.024, 0.030), at=(0, 0, 0.015))], mass=None, color=GREY)
kit.part("stay_btn", (*STAY, ZP + 0.030), [mk.box((0.016, 0.016, 0.012), at=(0, 0, 0.006))], mass=0.020,
         color=BLUE)
kit.slider("stay_push", "stay_base", "stay_btn", axis=(0, 0, -1), lower=0.0, upper=TAB_TRAVEL, stiffness=400.0,
           rest=-0.002)

BIN_XY, BIN_W, BIN_H = (0.100, 0.400), 0.110, 0.050


def _bin():
    x, y = BIN_XY
    w, t, h = BIN_W, 0.006, BIN_H
    return [Box("bin_a", pos=(x + (w - t) / 2, y, TOP + h / 2), size=(t, w, h), color=BIN),
            Box("bin_b", pos=(x - (w - t) / 2, y, TOP + h / 2), size=(t, w, h), color=BIN),
            Box("bin_c", pos=(x, y + (w - t) / 2, TOP + h / 2), size=(w - 2 * t, t, h), color=BIN),
            Box("bin_d", pos=(x, y - (w - t) / 2, TOP + h / 2), size=(w - 2 * t, t, h), color=BIN)]


FIXTURES = [
    Box("back_plate", pos=((X0 + X1) / 2, 0, TOP + 0.003), size=(X1 - X0, 2 * YH, 0.006), color=WHITE),
    Box("rim_near", pos=(X0 + WALL / 2, 0, (ZP + RIM) / 2), size=(WALL, 2 * YH, RIM - ZP), color=WHITE),
    Box("rim_far", pos=(X1 - WALL / 2, 0, (ZP + RIM) / 2), size=(WALL, 2 * YH, RIM - ZP), color=WHITE),
    Box("rim_left", pos=((X0 + X1) / 2, YH - WALL / 2, (ZP + RIM) / 2), size=(X1 - X0 - 2 * WALL, WALL, RIM - ZP),
        color=WHITE),
    Box("rim_right", pos=((X0 + X1) / 2, -YH + WALL / 2, (ZP + RIM) / 2), size=(X1 - X0 - 2 * WALL, WALL, RIM - ZP),
        color=WHITE),
    *_bin(),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules ------------------------------------------------------------------------------------------------------
DROP_RAD_S = 0.5
DT = 0.05  # one control step at 20 Hz


def _hub_rule(objects):
    for k in HUBS:
        pressed = kit.q(objects, f"tab_{k}_push") >= TAB_PRESSED
        if mk.MEMORY.get(f"pressed_{k}") != pressed:
            mk.MEMORY[f"pressed_{k}"] = pressed
            kit.drive(f"jaw_{k}_slide", JAW_TRAVEL + 0.004 if pressed else -0.004)
        if pressed and k == "p" and "stub_hold" not in kit._released:
            kit.release("stub_hold")


def _gate_rule(objects):
    det = 90.0 if kit.q_deg(objects, "gate_turn") >= 45.0 else 0.0
    if mk.MEMORY.get("gate_det") != det:
        mk.MEMORY["gate_det"] = det
        kit.drive("gate_turn", det)


def _cover_under_z(q_deg, x):
    """Height of the cover's inside face over world x, with the cover at q_deg."""
    q = math.radians(q_deg)
    return RIM + (X1 - x) * math.sin(q) + (CD - 0.004) * math.cos(q)


def _disengage(name):
    """Open catch ``name`` now (sim); every catch is disarmed again at reset."""
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _cover_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    q = kit.q_deg(objects, "cover_hinge")
    if m["n"] == 1:
        kit.engage("cover_stay")
    if not m.get("stay_off") and kit.q(objects, "stay_push") >= TAB_PRESSED:
        _disengage("cover_stay")
        m["stay_off"] = True
    if m.get("latched"):
        return
    hist = m.setdefault("q_hist", [])
    hist.append(q)
    del hist[:-4]
    w = math.radians(abs(hist[-1] - hist[0])) / (DT * (len(hist) - 1)) if len(hist) > 1 else 0.0
    if m["n"] > 10 and 2.0 < q < OPEN_DEG - 3.0 and w > DROP_RAD_S:
        m["dropped"] = True
    if q < 1.5:
        kit.engage("cover_latch")
        m["latched"] = True
        return
    if q < 45.0:  # cover down on a core that is not on its hub: forced over a mis-seated roll
        for n, ro in (("stub", STUB_RO), ("roll", ROLL_RO)):
            c = kit.point(objects, n, (0, 0, 0))
            inside = X0 < float(c[0]) < X1 and -YH < float(c[1]) < YH
            if inside and not (seated(objects, n, "p") or seated(objects, n, "r")):
                top = float(kit.point(objects, n, (0, 0, ROLL_H))[2])
                if top > _cover_under_z(q, float(c[0]) + ro) - 0.001:
                    m["crushed"] = True


def _latch_rule(objects):
    for n in ("stub", "roll"):
        c = kit.point(objects, n, (0, 0, 0))
        if float(c[2]) < TOP - 0.05:
            mk.MEMORY["floor"] = True
    c = kit.point(objects, "stub", (0, 0, 0))
    if abs(float(c[0]) - BIN_XY[0]) < BIN_W / 2 and abs(float(c[1]) - BIN_XY[1]) < BIN_W / 2 \
            and float(c[2]) < TOP + BIN_H:
        mk.MEMORY["trashed"] = True


kit.on_step(_hub_rule)
kit.on_step(_gate_rule)
kit.on_step(_cover_rule)
kit.on_step(_latch_rule)


def _debug_rule(objects):
    if os.environ.get("JUMBO_DEBUG") and mk.MEMORY.get("n", 0) % 20 == 0:
        f = lambda v: "(" + ",".join(f"{float(c):.4f}" for c in v) + ")"
        print(f"[jumbo] n={mk.MEMORY.get('n')} stub={f(kit.point(objects, 'stub', (0, 0, 0)))} "
              f"roll={f(kit.point(objects, 'roll', (0, 0, 0)))} gate={kit.q_deg(objects, 'gate_turn'):.1f} "
              f"cover={kit.q_deg(objects, 'cover_hinge'):.1f} jaw_p={kit.q(objects, 'jaw_p_slide'):.4f} "
              f"jaw_r={kit.q(objects, 'jaw_r_slide'):.4f} tab_p={kit.q(objects, 'tab_p_push'):.4f} "
              f"tab_r={kit.q(objects, 'tab_r_push'):.4f} stay={kit.q(objects, 'stay_push'):.4f} mem={ {k: v for k, v in mk.MEMORY.items() if k != 'n'} } "
              f"tcps={[f(t) for t in mk.tcps()]} score={score(objects)} sv={[round(float(v), 4) for v in (*objects['stub'].lin_vel, *objects['stub'].ang_vel)]} tilt={_tilt(kit.pose(objects, 'stub')[1]):.2f} rest={[(n, _released(objects[n]), objects[n].resting(max_speed=0.01, max_spin=0.1)) for n in ('stub', 'roll', 'cover', 'stay_btn')]}", flush=True)


kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None
# a core flat on the plate with its axis within 25 mm of the spindle's middle has the spindle through its 75 mm bore
# (any further off, its wall would sit up on the spindle)
SEAT_AXIAL, SEAT_LAT, SEAT_DEG, HUB_DX = 0.003, 0.025, 3.0, -0.001


def _tilt(q) -> float:
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def seated(objects, part, hub) -> bool:
    """Core bottom within 3 mm of the plate, its axis within 4 mm of the hub's seat and within 3 deg of the spindle:
    the core is all the way on and the roll turns freely clear of the housing and the other roll."""
    hx, hy = HUBS[hub]
    b = kit.point(objects, part, (0, 0, 0))
    _, q = kit.pose(objects, part)
    return (abs(float(b[2]) - ZP) <= SEAT_AXIAL and math.hypot(float(b[0]) - hx - HUB_DX, float(b[1]) - hy) <= SEAT_LAT
            and _tilt(q) <= SEAT_DEG)


def engaged(objects, hub) -> bool:
    """The sprung hub half is out against the core (its tab let go)."""
    return kit.q(objects, f"jaw_{hub}_slide") < JAW_OPEN_AT


def gate_clear(objects) -> bool:
    return kit.q_deg(objects, "gate_turn") >= 80.0


def cover_locked(objects) -> bool:
    return bool(mk.MEMORY.get("latched")) and kit.q_deg(objects, "cover_hinge") < 1.0


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def settled(objects) -> bool:
    return (all(_released(objects[n]) for n in ("stub", "roll", "tab_p", "tab_r", "stay_btn"))
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("stub", "roll", "cover")))


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("dropped") or m.get("floor") or m.get("trashed") or m.get("crushed"))


def inverted(objects) -> bool:
    return seated(objects, "roll", "r") and seated(objects, "stub", "p")


def score(objects) -> float:
    """0 if failed() or inverted(). Otherwise: stub seated on the reserve hub, engaged, gate clear 0.25; new roll
    seated on the primary hub, engaged 0.3; gate in its clear detent 0.05; with both seated: cover shut and latched
    0.3, grippers off and everything still 0.1."""
    if failed(objects) or inverted(objects):
        return 0.0
    gate = gate_clear(objects)
    stub_ok = seated(objects, "stub", "r") and engaged(objects, "r") and gate
    roll_ok = seated(objects, "roll", "p") and engaged(objects, "p")
    s = 0.25 * stub_ok + 0.30 * roll_ok + 0.05 * gate
    if stub_ok and roll_ok:
        s += 0.30 * cover_locked(objects) + 0.10 * settled(objects)
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    a = kit.point(objects, "stub", (0, 0, 0))
    b = kit.point(objects, "roll", (0, 0, 0))
    return float(math.dist([float(v) for v in a], [RES[0] + SEAT_DX, RES[1], ZP])
                 + math.dist([float(v) for v in b], [PRIM[0] + SEAT_DX, PRIM[1], ZP]))


# -- the scripted demo ------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = 0.900
F_IN, F_GRIP = -0.020, -0.040  # fingers inside a core; opened against its wall
GRIP_DZ = ROLL_H - 0.025 + TIP  # TCP above a core's bottom: pad tips 25 mm down inside the core
LIFT = 0.035  # core bottom this far above the plate clears the hub halves and the rim
TAB_Z = ZP + 0.022 - TAB_TRAVEL - 0.006 + TIP  # pushed onto the tab's stop and 6 mm on
TAB_UP = 0.800
AIM_X = 0.278  # cores are lowered over the spindle's middle, the sprung half drawn in (the hand lags a few mm)
GRIP_LIFT_DX = 0.0


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


def _gate_sweep(s, start=-5, stop=100, step=15, r=0.050, off=0.0145):
    def at(q):
        a = math.radians(90.0 - q)
        return (GATE[0] + r * math.cos(a) - off * math.sin(a), GATE[1] + r * math.sin(a) + off * math.cos(a))

    z = GATE_Z - 0.003 + TIP
    s.go(2.0, (*at(start), TRAVEL), 0.0, "over the gate lever")
    s.go(1.5, (*at(start), z), 0.0, "down beside it")
    for q in range(start + step, stop + 1, step):
        s.go(1.0, (*at(q), z), 0.0, f"push the gate to {q} deg")
    s.go(1.0, (*at(stop - 10), TRAVEL), 0.0, "up off the gate")


def _press_tab(s, k, hold_until):
    tx, ty = TABS[k]
    s.go(2.0, (tx, ty, TAB_UP), 0.0, f"over the {k} release tab")
    s.go(1.0, (tx, ty, TAB_Z), 0.0, "hold the tab down")
    s.until(hold_until, "hold it while the core goes along the spindle")
    s.go(0.8, (tx, ty, TAB_UP), 0.0, "let the tab up")


def _close_cover(s):
    """Press the cover stay's release button; the damper lowers the cover onto its latch."""
    bx, by = STAY
    s.go(2.0, (bx, by, ZP + 0.080 + TIP), 0.0, "over the cover stay's button")
    s.go(1.0, (bx, by, ZP + 0.030 + 0.012 - 0.012 + TIP), 0.0, "press it")
    s.go(0.6, (bx, by, ZP + 0.030 + 0.012 - 0.012 + TIP), 0.0, "hold")
    s.go(1.0, (bx, by, ZP + 0.080 + TIP), 0.0, "let it up")
    s.go(1.5, (0.20, -0.36, TRAVEL), 0.0, "clear")
    s.go(8.0, (0.20, -0.36, TRAVEL), 0.0, "wait for the latch")


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    sx, sy = PRIM[0] + SEAT_DX, PRIM[1]
    tmp = (0.290, 0.000)
    pick = (0.295, 0.0075)  # where the stub ends up after it is set down at tmp (the hand lags it a few mm)
    rx, ry = AIM_X, RES[1]
    if variant == "close_only":
        L.go(2.0, (0.20, 0.35, 0.92), 0.0, "park")
        R.go(1.0, (0.20, -0.33, TRAVEL), 0.0, "start")
        _close_cover(R)
        return {"left": L.w, "right": R.w}
    # R: gate, then hold the primary tab while L takes the stub off
    _gate_sweep(R)
    t_tab_p = R.t + 3.0
    # L: into the stub's core
    L.go(2.5, (sx, sy, TRAVEL), F_IN, "over the stub")
    L.go(1.5, (sx, sy, ZP + GRIP_DZ), F_IN, "fingers into its core")
    L.until(t_tab_p, "wait for the hub to be released")
    L.go(1.0, (sx, sy, ZP + GRIP_DZ), F_GRIP, "open against the core")
    L.go(0.5, (sx, sy, ZP + GRIP_DZ), F_GRIP, "settle")
    L.go(1.5, (sx, sy, ZP + GRIP_DZ + LIFT), F_GRIP, "slide the stub up off the spindle")
    t_lifted = L.t
    _press_tab(R, "p", t_lifted + 0.3)
    if variant == "trash_stub":
        L.go(2.5, (BIN_XY[0], BIN_XY[1], ZP + GRIP_DZ + 0.06), F_GRIP, "over the bin")
        L.go(0.8, (BIN_XY[0], BIN_XY[1], ZP + GRIP_DZ + 0.06), F_IN, "drop the stub in")
        L.go(1.0, (BIN_XY[0], BIN_XY[1], TRAVEL), F_IN, "up")
    else:
        L.go(2.0, (*tmp, ZP + GRIP_DZ + LIFT), F_GRIP, "set it aside on the plate")
        L.go(1.0, (*tmp, ZP + GRIP_DZ + 0.001), F_GRIP, "down")
        L.go(0.8, (*tmp, ZP + GRIP_DZ + 0.001), F_IN, "let go")
        L.go(1.0, (*tmp, TRAVEL), F_IN, "up")
        t_l_clear = L.t
        L.go(1.5, (0.20, 0.25, TRAVEL), 0.0, "out of the way")
        # R: take the stub to the reserve while L holds the reserve tab
        R.go(1.0, (TABS["p"][0], TABS["p"][1] - 0.06, TRAVEL), F_IN, "up and back")
        R.until(t_l_clear + 1.2)
        R.go(2.0, (*pick, TRAVEL), F_IN, "over the stub")
        R.go(1.5, (*pick, ZP + GRIP_DZ), F_IN, "fingers into its core")
        R.go(1.0, (*pick, ZP + GRIP_DZ), F_GRIP, "open against the core")
        R.go(0.5, (*pick, ZP + GRIP_DZ), F_GRIP, "settle")
        R.go(1.5, (*pick, ZP + GRIP_DZ + LIFT), F_GRIP, "lift")
        R.go(2.0, (rx, ry, ZP + GRIP_DZ + LIFT), F_GRIP, "over the reserve spindle")
        R.go(1.0, (rx, ry, ZP + GRIP_DZ + LIFT), F_GRIP, "settle over it")
        t_res_tab = R.t
        R.go(1.5, (rx, ry, ZP + GRIP_DZ - 0.002), F_GRIP, "slide it down onto the spindle")
        R.go(0.5, (rx, ry, ZP + GRIP_DZ - 0.002), F_GRIP, "settle")
        R.go(0.8, (rx, ry, ZP + GRIP_DZ - 0.002), F_IN, "let go")
        t_r_off = R.t
        R.go(1.0, (rx, ry, TRAVEL), F_IN, "up")
        L.until(t_res_tab - 5.0)
        _press_tab(L, "r", t_r_off + 0.2)
    # L: the new roll onto the primary while R holds the primary tab (loose_roll: without the tab)
    nx, ny = NEW0[0], NEW0[1]
    lz = NEW0[2] + GRIP_DZ
    L.go(2.5, (nx, ny, TRAVEL), F_IN, "over the new roll")
    L.go(1.5, (nx, ny, lz), F_IN, "fingers into its core")
    L.go(1.0, (nx, ny, lz), F_GRIP, "open against the core")
    L.go(0.5, (nx, ny, lz), F_GRIP, "settle")
    L.go(1.5, (nx, ny, lz + 0.065), F_GRIP, "lift")
    L.go(2.5, (AIM_X + 0.004, sy, ZP + GRIP_DZ + 0.060), F_GRIP, "over the primary spindle")
    L.go(1.5, (AIM_X + 0.004, sy, ZP + GRIP_DZ + 0.045), F_GRIP, "let it settle over the spindle")
    t_prim = L.t
    L.go(2.0, (AIM_X + 0.004, sy, ZP + GRIP_DZ - 0.003), F_GRIP, "slide it down onto the spindle")
    L.go(0.5, (AIM_X + 0.004, sy, ZP + GRIP_DZ - 0.003), F_GRIP, "settle")
    L.go(0.8, (AIM_X + 0.004, sy, ZP + GRIP_DZ - 0.003), F_IN, "let go")
    t_l_off = L.t
    L.go(1.2, (sx, sy, TRAVEL + 0.02), F_IN, "up")
    L.go(2.0, (0.20, 0.35, 0.92), F_IN, "park")
    if variant != "loose_roll":
        R.until(t_prim - 5.0)
        _press_tab(R, "p", t_l_off + 0.2)
    else:
        R.until(t_l_off)
    R.until(L.t)
    _close_cover(R)
    return {"left": L.w, "right": R.w}


def demo():
    # DEMO_VARIANT: close_only | trash_stub | loose_roll (degenerate checks; all score 0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])

    def at(hub, z=0.0):
        hx, hy = HUBS[hub]
        return (np.array([hx + SEAT_DX, hy, ZP + z]), I)

    def st(q=None, **moved):
        return kit.fake_states(q=q, moved=moved)

    closed = {"cover_hinge": 0.0, "gate_turn": math.radians(90)}
    done = st(closed, stub=at("r"), roll=at("p"))
    L = {"latched": True}
    tilt = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(5)))
    cases = {
        "start": (st(), [], {}, 0.0),
        "gate only": (st({"gate_turn": math.radians(90)}), [], {}, 0.05),
        "closed untouched": (st({"cover_hinge": 0.0}), [], L, 0.0),
        "stub on reserve, gate clear": (st({"gate_turn": math.radians(90)}, stub=at("r")), [], {}, 0.30),
        "stub on reserve, gate shut": (st(stub=at("r")), [], {}, 0.0),
        "both seated, cover open": (st({"gate_turn": math.radians(90)}, stub=at("r"), roll=at("p")), [], {}, 0.7),
        "both seated, latched, held": (done, [kit.point(done, "roll", (0, 0, 0.08))], L, 0.9),
        "everything done": (done, [], L, 1.0),
        "roll 4 mm high": (st(closed, stub=at("r"), roll=at("p", 0.004)), [], L, 0.30),
        "roll tilted 5 deg": (st(closed, stub=at("r"), roll=(at("p")[0], tilt)), [], L, 0.30),
        "roll hub still squeezed": (st({**closed, "jaw_p_slide": 0.040}, stub=at("r"), roll=at("p")), [], L, 0.3),
        "inverted": (st(closed, stub=at("p"), roll=at("r")), [], L, 0.0),
        "dropped cover": (done, [], {**L, "dropped": True}, 0.0),
        "stub in the bin": (done, [], {**L, "trashed": True}, 0.0),
        "cover forced on a loose roll": (done, [], {**L, "crushed": True}, 0.0),
        "roll on the floor": (done, [], {**L, "floor": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert seated(st(), "stub", "p")
        assert _cover_under_z(0.0, 0.0) > ZP + ROLL_H + 0.005
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
    for v in ("", "close_only", "trash_stub", "loose_roll"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
