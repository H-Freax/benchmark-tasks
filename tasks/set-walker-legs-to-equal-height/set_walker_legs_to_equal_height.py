"""A folding aluminium walker lies folded flat in a bench fixture, its four legs pointing away from the robot. Each
leg is an inner extension tube (with a rubber foot) sliding in the frame's outer tube. A sprung push pin on the
inner tube pokes up through a hole in the outer tube and locks it; the inner tube has five holes 25 mm apart. All
four legs start at hole 1. A red band across the fixture marks where the feet sit at hole 3.

The job: every leg out to hole 3 with its pin snapped back up. A leg only slides while its pin is held down, so one
hand holds the pin while the other pulls the foot. Let go of the pin between two holes and it rides on the inside
of the tube, the leg loose; it snaps into the next hole the leg is moved to. Pulled past hole 5 the extension
leaves its tube (dropped, scores 0 for good).

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/set-walker-legs-to-equal-height/set_walker_legs_to_equal_height.py \\
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

ALU = (0.72, 0.74, 0.77)
GREY = (0.45, 0.46, 0.48)
RUBBER = (0.12, 0.12, 0.13)
RED = (0.85, 0.12, 0.10)
PIN = (0.80, 0.80, 0.82)
WOOD = (0.62, 0.50, 0.36)

# -- geometry (world m) -----------------------------------------------------------------------------------------
# The legs point toward the right arm (-y); x is across the walker.
BOARD = 0.006  # fixture board
AZ = TOP + BOARD + 0.026  # leg axis height (tubes rest on 13 mm saddles)
OUT_W, IN_W = 0.0254, 0.019  # outer tube 1 in square; inner tube (shown smaller: it sits inside)
TUBE_Y1, MOUTH = 0.200, 0.040  # outer tube from the frame bend (+y) to its mouth
PIN_Y = 0.070  # pin hole in the outer tube, 30 mm in from the mouth
IN_LEN = 0.270
FOOT_L, FOOT_W = 0.035, 0.040
LEGS = {"a": 0.180, "b": 0.245, "c": 0.310, "d": 0.375}  # x of each leg: front legs outside, rear folded in
EXPOSED0 = 0.120  # inner tube showing past the mouth at hole 1
FOOT0 = MOUTH - EXPOSED0 - FOOT_L / 2  # foot centre (y) at hole 1
PITCH, HOLES, TARGET = 0.025, 5, 2  # holes 1..5 at 0, 25 ... 100 mm out; hole 3 is index 2
OUT_AT = 0.118  # past this the inner tube's end has left the pin and the tube: the leg comes out
BAND_Y = MOUTH - EXPOSED0 - TARGET * PITCH  # the foot's back edge at hole 3
PIN_W, PIN_H, PIN_TRAVEL = 0.006, 0.006, 0.003
PIN_Z0 = AZ + OUT_W / 2 + 0.0003  # pin bottom when up
PIN_PRESSED, PIN_UP = 0.002, 0.001
NEAR_HOLE = 0.002
CATCH = 0.003  # the pin's rounded tip finds a hole within 3 mm
HAND_ON_PIN = 0.025

kit = mk.Kit()

_frame = []
for x in LEGS.values():
    _frame.append(mk.box((OUT_W, TUBE_Y1 - MOUTH, OUT_W), at=(x, (TUBE_Y1 + MOUTH) / 2, AZ)))
    for sy in (0.120, 0.170):
        _frame.append(mk.box((0.030, 0.020, AZ - OUT_W / 2 - TOP - BOARD), at=(x, sy, (AZ - OUT_W / 2 + TOP + BOARD) / 2),
                             color=GREY))
# frame bends and cross bars at the far end (folded front frame, side frames, hand grips)
_frame += [mk.box((0.220, 0.025, 0.025), at=(0.2775, TUBE_Y1 + 0.0125, AZ)),
           mk.box((0.090, 0.025, 0.025), at=(0.2775, 0.150, AZ + 0.026)),
           mk.box((0.030, 0.030, 0.026), at=(0.245, 0.150, AZ + 0.013), color=GREY),
           mk.box((0.030, 0.030, 0.026), at=(0.310, 0.150, AZ + 0.013), color=GREY)]
kit.part("frame", (0, 0, 0), _frame, mass=None, color=ALU)

for k, x in LEGS.items():
    kit.part(f"leg_{k}", (x, FOOT0, AZ),
             [mk.box((IN_W, IN_LEN, IN_W), at=(0, FOOT_L / 2 + IN_LEN / 2, 0), color=ALU),
              mk.box((FOOT_W, FOOT_L, FOOT_W), at=(0, 0, 0), color=RUBBER)],
             mass=0.30, color=ALU)
    kit.slider(f"slide_{k}", "frame", f"leg_{k}", axis=(0, -1, 0), lower=-0.002, upper=0.140, friction=1.5,
               damping=4.0)
    kit.part(f"pin_{k}", (x, PIN_Y, PIN_Z0), [mk.box((PIN_W, PIN_W, PIN_H), at=(0, 0, PIN_H / 2))], mass=0.02,
             color=PIN)
    kit.slider(f"pin_{k}_push", "frame", f"pin_{k}", axis=(0, 0, -1), lower=0.0, upper=PIN_TRAVEL,
               stiffness=600.0, rest=-0.002, damping=1.0)
    for h in range(HOLES):
        T = kit.parts[f"leg_{k}"].ref.copy()
        T[1, 3] -= h * PITCH
        kit.catch(f"lock_{k}_{h}", f"leg_{k}", T)

FIXTURES = [
    Box("board", pos=(0.2775, 0.035, TOP + BOARD / 2), size=(0.320, 0.390, BOARD), color=WOOD),
    Box("band", pos=(0.2775, BAND_Y - 0.004, TOP + BOARD + 0.0005), size=(0.270, 0.008, 0.001), color=RED),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules ------------------------------------------------------------------------------------------------------


def _disengage(name):
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _hand_near(objects, k, margin=HAND_ON_PIN) -> bool:
    top = kit.point(objects, f"pin_{k}", (0, 0, PIN_H))
    return any(float(np.linalg.norm(np.asarray(t) - top)) < margin for t in mk.tcps())


def _pin_rule(objects):
    """Per leg: a held-down pin unlocks it; a pin let go at a hole snaps up and locks it; let go between holes it
    rides down on the tube wall (held down here) and the leg stays loose until it reaches a hole."""
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    for k in LEGS:
        if m["n"] == 1:
            m[f"lock_{k}"] = 0
            kit.engage(f"lock_{k}_0")
            continue
        if m.get(f"out_{k}"):
            continue
        q = kit.q(objects, f"slide_{k}")
        if q > OUT_AT:
            kit.release(f"slide_{k}")
            m[f"out_{k}"] = True
            m["dropped"] = True
            continue
        pin = kit.q(objects, f"pin_{k}_push")
        lock = m.get(f"lock_{k}")
        if lock is not None:
            if pin >= PIN_PRESSED:
                _disengage(f"lock_{k}_{lock}")
                m[f"lock_{k}"] = None
                kit.drive(f"pin_{k}_push", PIN_TRAVEL + 0.002)  # rides the wall once the hand lets go
            continue
        h = int(round(q / PITCH))
        if 0 <= h < HOLES and abs(q - h * PITCH) <= CATCH and not _hand_near(objects, k):
            kit.engage(f"lock_{k}_{h}")
            m[f"lock_{k}"] = h
            kit.drive(f"pin_{k}_push", -0.002)


def _debug_rule(objects):
    if os.environ.get("WALKER_DEBUG") and mk.MEMORY.get("n", 0) % 10 == 0:
        print("[walker] tcp", [tuple(round(float(v), 4) for v in t) for t in mk.tcps()],
              "foot_a", tuple(round(float(v), 4) for v in kit.point(objects, "leg_a", (0, 0, 0))), flush=True)
        legs = " ".join(f"{k}:q={kit.q(objects, f'slide_{k}') * 1000:.1f}/pin={kit.q(objects, f'pin_{k}_push') * 1000:.1f}"
                        f"/lock={mk.MEMORY.get(f'lock_{k}')}" for k in LEGS)
        print(f"[walker] n={mk.MEMORY.get('n')} {legs} dropped={mk.MEMORY.get('dropped')} score={score(objects)}",
              flush=True)


kit.on_step(_pin_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ----------------------------------------------------------------------------------------------------
_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def leg_set(objects, k) -> bool:
    """Leg k locked in hole 3: its pin snapped up into that hole (latched lock state and the pin up), the leg
    within 2 mm of the hole, no gripper within 2 cm of its foot or pin, and the leg still."""
    m = mk.MEMORY
    if m.get(f"lock_{k}") != TARGET or m.get(f"out_{k}"):
        return False
    if abs(kit.q(objects, f"slide_{k}") - TARGET * PITCH) > NEAR_HOLE or kit.q(objects, f"pin_{k}_push") > PIN_UP:
        return False
    for part in (f"leg_{k}", f"pin_{k}"):
        st = objects[part]
        lo, hi = np.asarray(st.low) - 0.02, np.asarray(st.high) + 0.02
        if part == f"leg_{k}":  # the foot end only (the inner tube is inside the frame)
            hi[1] = min(hi[1], float(kit.point(objects, part, (0, FOOT_L / 2, 0))[1]) + 0.02)
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps()):
            return False
    return objects[f"leg_{k}"].resting(max_speed=0.01, max_spin=0.25)


def failed(objects) -> bool:
    return bool(mk.MEMORY.get("dropped"))


def score(objects) -> float:
    """0 if a leg came out of its tube. Otherwise 0.25 per leg locked in hole 3, released and still."""
    if failed(objects):
        return 0.0
    return round(0.25 * sum(leg_set(objects, k) for k in LEGS), 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(sum(abs(kit.q(objects, f"slide_{k}") - TARGET * PITCH) for k in LEGS))


# -- the scripted demo ------------------------------------------------------------------------------------------
TIP = 0.013
HOVER = TOP + 0.075
OPEN = -0.030
PRESS_Z = PIN_Z0 + PIN_H - PIN_TRAVEL - 0.009 + TIP  # closed pads pushed onto the pin's stop and 4 mm on
GRIP_Z = AZ + IN_W / 2 + 0.002 + TIP  # pad tips just above the inner tube, on the foot's two end faces
PARK = {"left": (0.12, 0.25, HOVER), "right": (0.12, -0.25, HOVER)}


class _Seq:
    def __init__(self, arm):
        self.w, self.t = [], 0.0
        self.last = (PARK[arm], OPEN, 0.0)

    def go(self, d, pos, f, label, yaw=0.0):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, yaw))
        self.t += d
        self.last = (tuple(pos), f, yaw)
        return self

    def until(self, t, label="wait"):
        pos, f, yaw = self.last
        return self.go(max(0.1, t - self.t), pos, f, label, yaw)


def _leg(arms, k, out=TARGET * PITCH, press=True):
    """One leg: the left arm holds the pin down, the right arm grips the foot by its two end faces and pulls it
    ``out`` (toward -y)."""
    x = LEGS[k]
    puller, presser = arms["right"], arms["left"]
    t0 = max(puller.t, presser.t)
    puller.until(t0)
    presser.until(t0)
    puller.go(2.0, (x, FOOT0, HOVER), OPEN, f"over the {k} foot")
    puller.go(1.5, (x, FOOT0, GRIP_Z), OPEN, "down round the foot")
    puller.go(1.0, (x, FOOT0, GRIP_Z), FINGER_CLOSED, "grip the foot")
    if press:
        py = PIN_Y - 0.004  # the arms press each other apart a few mm here
        presser.go(2.0, (x, py, HOVER), FINGER_CLOSED, f"over the {k} pin")
        presser.go(1.5, (x, py, PRESS_Z + 0.010), FINGER_CLOSED, "down onto the pin")
        presser.go(1.0, (x, py, PRESS_Z), FINGER_CLOSED, "hold the pin down")
    t1 = max(puller.t, presser.t) + 0.3
    puller.until(t1)
    y1 = FOOT0 - out + 0.002  # the arm lags then overshoots 2-3 mm
    puller.go(3.5, (x, y1, GRIP_Z), FINGER_CLOSED, "pull the leg out")
    puller.go(1.0, (x, y1, GRIP_Z), FINGER_CLOSED, "hold it there")
    if press:
        presser.until(puller.t)
        presser.go(1.0, (x, py, HOVER), FINGER_CLOSED, "let the pin up")
        presser.go(0.8, (x - 0.030, py + 0.030, HOVER), FINGER_CLOSED, "hand off the pin")
        puller.until(presser.t)
        puller.go(0.6, (x, y1 - 0.004, GRIP_Z), FINGER_CLOSED, "work the leg a little out")
        puller.go(1.0, (x, y1 + 0.001, GRIP_Z), FINGER_CLOSED, "and in, till the pin clicks into the hole")
        puller.go(0.6, (x, y1, GRIP_Z), FINGER_CLOSED, "settle")
    puller.go(0.8, (x, y1, GRIP_Z), OPEN, "let go of the foot")
    puller.go(1.0, (x, y1, HOVER), OPEN, "up")


def _plans(variant=""):
    arms = {"left": _Seq("left"), "right": _Seq("right")}
    order = ("a", "b", "c", "d")
    for i, k in enumerate(order):
        out = TARGET * PITCH
        if variant == "midway":
            out = TARGET * PITCH - 0.013  # between holes 2 and 3
        elif variant == "overshoot":
            out = (TARGET + 1) * PITCH  # hole 4
        elif variant == "mixed" and i % 2:
            out = (TARGET + 1) * PITCH
        elif variant == "pull_out" and i == 0:
            out = OUT_AT + 0.010
        _leg(arms, k, out, press=variant != "no_press")
    for a in ("left", "right"):
        arms[a].go(1.5, PARK[a], OPEN, "park")
    t = max(s.t for s in arms.values()) + 2.0
    for s in arms.values():
        s.until(t, "wait")
    return {a: s.w for a, s in arms.items()}


VARIANTS = ("no_press", "midway", "overshoot", "pull_out", "mixed")


def demo():
    # DEMO_VARIANT: no_press | midway | overshoot | pull_out (score 0); mixed (two legs right: 0.5)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS

    def st(**q):
        return kit.fake_states(q=q)

    at3 = {f"slide_{k}": TARGET * PITCH for k in LEGS}
    locked3 = {f"lock_{k}": TARGET for k in LEGS}
    foot = kit.point(st(**at3), "leg_a", (0, 0, 0))
    pin = kit.point(st(**at3), "pin_c", (0, 0, PIN_H))
    cases = {
        "start": (st(), [], {f"lock_{k}": 0 for k in LEGS}, 0.0),
        "all at hole 3, locked": (st(**at3), [], locked3, 1.0),
        "all at hole 3, one pin ridden down": (st(**at3, pin_a_push=PIN_TRAVEL), [], {**locked3, "lock_a": None}, 0.75),
        "all at hole 3, one still held at the foot": (st(**at3), [foot], locked3, 0.75),
        "all at hole 3, a hand on one pin": (st(**at3), [pin], locked3, 0.75),
        "two at hole 3, two at hole 4": (st(**{**at3, "slide_a": 0.075, "slide_c": 0.075}), [],
                                         {**locked3, "lock_a": 3, "lock_c": 3}, 0.5),
        "all at hole 4": (st(**{k: 0.075 for k in at3}), [], {f"lock_{k}": 3 for k in LEGS}, 0.0),
        "all between holes, loose": (st(**{k: 0.037 for k in at3}), [], {f"lock_{k}": None for k in LEGS}, 0.0),
        "lock state 3 but leg 4 mm off": (st(**{**at3, "slide_b": 0.054}), [], locked3, 0.75),
        "one leg came out": (st(**at3), [], {**locked3, "dropped": True, "out_d": True}, 0.0),
        "went to 4, back to 3": (st(**at3), [], locked3, 1.0),
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
    for v in ("", *VARIANTS):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
