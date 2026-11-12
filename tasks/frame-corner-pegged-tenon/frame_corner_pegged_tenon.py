"""A joiner's bench, dry-fitting one corner of a pine face frame. A 280 mm stile (50 x 20 mm pine) is clamped flat
on the bench with its long edge toward the robot; near its right end it has a through mortise (32 x 10 mm, through
the full 50 mm width) and a 12 mm square peg hole bored down through the stile's face, crossing the mortise. On the
right, lying flat on the bench parallel to the arms, is the matching 200 mm rail (40 x 20 mm pine, 73 g) with a
through tenon (30 x 8 mm, 54 mm long, its end chamfered) that has the same 12 mm hole through it. On the left a
square oak drawbore peg (10 x 10 x 50 mm) stands in a bench block.

The corner is assembled when the tenon is driven home (rail shoulder tight against the stile) and the peg is
through the stile and the tenon, locking them. Order matters: the peg only passes once the tenon's hole lines up
under the stile's, and a peg put in first fills the mortise so the tenon can no longer go home. The tenon only moves
along the mortise (1 mm clearance on each side), so the rail has to be lined up with it before it is pushed.

Scored from the parts' poses at the end, after both grippers let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/frame-corner-pegged-tenon/frame_corner_pegged_tenon.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box parts)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PINE = (0.86, 0.72, 0.50)
END_GRAIN = (0.74, 0.58, 0.36)
OAK = (0.55, 0.38, 0.20)
BLOCK = (0.45, 0.42, 0.40)

# Frame: x forward from the arm bases, y left, z up, metres.
T = 0.020  # stock thickness (stile and rail)
FACE_X = 0.330  # the stile's near edge (the face the rail's shoulder meets)
STILE_W = 0.050  # stile width (x): the mortise runs through it
STILE_Y = (-0.140, 0.140)
MY = 0.000  # mortise centre line (y)
MORT_W, MORT_H = 0.032, 0.010  # mortise opening: 32 mm (y) x 10 mm (z), centred in the thickness
HOLE = 0.012  # peg hole, square
TEN_HOLE_X = 0.013  # the tenon's hole: 13 mm along the tenon, 12 mm across (1 mm of play for the shoulder)
HOLE_X = 0.025  # peg hole centre, back from the stile's near edge (centre of the width)
TEN_W, TEN_H, TEN_L = 0.030, 0.008, 0.054  # tenon: 30 x 8 mm, 54 mm long (4 mm proud of the far edge)
CHAMFER = 0.006  # the last 6 mm of the tenon stepped in 2 mm a side (a chamfered end)
RAIL_L, RAIL_W = 0.200, 0.040
RAIL_KG = 0.073  # 200 x 40 x 20 mm pine at 450 kg/m^3, plus the tenon
PEG = (0.010, 0.010, 0.050)
PEG_KG = 0.0035  # oak, 700 kg/m^3
PEG_XY = (0.300, 0.100)  # the peg block's hole
RAIL_START = (0.270, -0.130)  # the rail's shoulder (centre of its tenon end, on the bench) at the start


def _stile_boxes():
    """Stile frame: origin at the mortise mouth on the near edge, at bench level; x into the stile."""
    zc = T / 2
    lo_y, hi_y = STILE_Y[0] - MY, STILE_Y[1] - MY
    h, w, hm = MORT_W / 2, STILE_W, MORT_H / 2
    wall = (T - MORT_H) / 2  # 5 mm above and below the mortise
    hx0, hx1, hy = HOLE_X - HOLE / 2, HOLE_X + HOLE / 2, HOLE / 2
    out = [
        mk.box((w, hi_y - h, T), at=(w / 2, (hi_y + h) / 2, zc)),  # beside the mortise (left)
        mk.box((w, -h - lo_y, T), at=(w / 2, (lo_y - h) / 2, zc)),  # beside the mortise (right)
    ]
    for z in (wall / 2, T - wall / 2):  # the cheeks below and above the mortise, bored through for the peg
        out += [
            mk.box((hx0, MORT_W, wall), at=(hx0 / 2, 0, z)),
            mk.box((w - hx1, MORT_W, wall), at=((w + hx1) / 2, 0, z)),
            mk.box((HOLE, h - hy, wall), at=(HOLE_X, (h + hy) / 2, z)),
            mk.box((HOLE, h - hy, wall), at=(HOLE_X, -(h + hy) / 2, z)),
        ]
    return out


def _rail_boxes():
    """Rail frame: origin at the shoulder, on the rail's underside, on its centre line; the tenon along +x."""
    zc = T / 2
    t0 = HOLE_X - TEN_HOLE_X / 2  # the tenon's hole lines up with the stile's when the shoulder is home
    t1 = HOLE_X + TEN_HOLE_X / 2
    hw, hy = TEN_W / 2, HOLE / 2
    tip = TEN_L - CHAMFER
    return [
        mk.box((RAIL_L, RAIL_W, T), at=(-RAIL_L / 2, 0, zc)),
        mk.box((0.002, RAIL_W, T), at=(-0.001, 0, zc), color=END_GRAIN),  # shoulder line (cosmetic, inside rail)
        mk.box((t0, TEN_W, TEN_H), at=(t0 / 2, 0, zc)),
        mk.box((tip - t1, TEN_W, TEN_H), at=((tip + t1) / 2, 0, zc)),
        mk.box((TEN_HOLE_X, hw - hy, TEN_H), at=(HOLE_X, (hw + hy) / 2, zc)),
        mk.box((TEN_HOLE_X, hw - hy, TEN_H), at=(HOLE_X, -(hw + hy) / 2, zc)),
        mk.box((CHAMFER, TEN_W - 0.004, TEN_H - 0.004), at=(tip + CHAMFER / 2, 0, zc), color=END_GRAIN),
    ]


def _peg_block_boxes():
    s, h, g = 0.040, 0.015, HOLE / 2
    side = (s - HOLE) / 2
    return [
        mk.box((s, side, h), at=(0, (g + s / 2) / 2, h / 2)),
        mk.box((s, side, h), at=(0, -(g + s / 2) / 2, h / 2)),
        mk.box((side, HOLE, h), at=((g + s / 2) / 2, 0, h / 2)),
        mk.box((side, HOLE, h), at=(-(g + s / 2) / 2, 0, h / 2)),
    ]


kit = mk.Kit()
kit.part("stile", (FACE_X, MY, TOP), _stile_boxes(), mass=None, color=PINE)  # clamped to the bench
kit.part("peg_block", (PEG_XY[0], PEG_XY[1], TOP), _peg_block_boxes(), mass=None, color=BLOCK)
kit.part("rail", (RAIL_START[0], RAIL_START[1], TOP), _rail_boxes(), mass=RAIL_KG, color=PINE)
kit.part("peg", (PEG_XY[0], PEG_XY[1], TOP), [mk.box(PEG, at=(0, 0, PEG[2] / 2))], mass=PEG_KG, color=OAK)
OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

HOME_GAP = 0.002  # shoulder within 2 mm of the stile's edge: home
LINE_TOL = 0.003  # rail centre line within 3 mm of the mortise's
PART_IN = 0.025  # tenon at least 25 mm into the mortise (half the stile)
PEG_DOWN = 0.004  # peg's lowest point within 4 mm of the bench: through the tenon (it rests on the tenon at 14 mm)
PEG_XY_TOL = 0.004
TILT_MAX = 10.0
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(st) -> float:
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _shoulder(objects):
    return kit.point(objects, "rail", (0, 0, 0))


def tenon_depth(objects) -> float:
    """How far the tenon's end is past the stile's near edge (m); 54 mm when home."""
    return float(kit.point(objects, "rail", (TEN_L, 0, 0))[0] - FACE_X)


def _rail_home(objects) -> bool:
    s = _shoulder(objects)
    return bool(FACE_X - s[0] <= HOME_GAP and abs(s[1] - MY) <= LINE_TOL)


def _pegged(objects) -> bool:
    st = objects["peg"]
    c = np.asarray(st.pos, float)
    return bool(float(st.low[2]) <= TOP + PEG_DOWN and math.dist(c[:2], (FACE_X + HOLE_X, MY)) <= PEG_XY_TOL
                and _tilt(st) <= TILT_MAX and st.resting() and _released(st))


def score(objects) -> float:
    """1.0 tenon home (shoulder within 2 mm of the stile, on the mortise's line within 3 mm) and the peg through
    stile and tenon (its foot within 4 mm of the bench, upright within 10 deg, in the hole), everything let go and
    still; 0.5 tenon home, peg not through; 0.2 tenon at least 25 mm into the mortise but not home; else 0. The rail
    must be let go and still for any credit."""
    rail = objects["rail"]
    if not (rail.resting() and _released(rail)):
        return 0.0
    if _rail_home(objects):
        return 1.0 if _pegged(objects) else 0.5
    return 0.2 if tenon_depth(objects) >= PART_IN else 0.0


def success(objects) -> bool:
    """Tenon home and pegged, let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres the shoulder still has to travel to the stile's edge, plus the peg foot's distance to the bench
    in the hole."""
    s = _shoulder(objects)
    p = objects["peg"]
    return float(math.dist(s[:2], (FACE_X, MY)) + math.dist([float(p.pos[0]), float(p.pos[1]), float(p.low[2])],
                                                             [FACE_X + HOLE_X, MY, TOP]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the fixed layout. Right arm: grip the rail across its width 60 mm behind the shoulder, lift it
# a few mm, draw it back so the tenon clears the stile, bring it onto the mortise's line, lower it to just above the
# bench and push it straight in along the mortise, commanding a few mm past home and holding so the shoulder seats;
# let go and leave. Left arm, after that: lift the peg out of its block, carry it over the stile's peg hole, lower
# it until its foot is 8 mm into the stile, let it drop, then tap it down with the closed fingers.

TIP = 0.013
GRIP_BACK = 0.060  # grip the rail this far behind the shoulder
RAIL_TIPS = TOP + 0.005  # pad tips 5 mm above the bench beside the rail
PEG_TIPS = TOP + 0.020  # pad tips on the peg 20 mm above its foot (5 mm above the block)
RAIL_OPEN = -0.032
PEG_OPEN = -0.016
TRAVEL_R = TOP + 0.090
TRAVEL_L = TOP + 0.085
PUSH_PAST = 0.006
Y_TRIM = 0.0155  # measured in sim: closing on the rail shifts it 15.5 mm toward -y in the fingers
PARK_L = (0.150, 0.280, TOP + 0.120)


def _right():
    sx, sy = RAIL_START
    gx0 = sx - GRIP_BACK
    back = FACE_X - TEN_L - 0.010 - GRIP_BACK  # tenon end 10 mm short of the stile
    home = FACE_X - GRIP_BACK
    z0, lift, carry = RAIL_TIPS + TIP, RAIL_TIPS + TIP + 0.012, RAIL_TIPS + TIP + 0.0015
    o, c = RAIL_OPEN, FINGER_CLOSED
    return [
        Waypoint(2.0, (gx0, sy, TRAVEL_R), o, "over the rail"),
        Waypoint(1.5, (gx0, sy, z0 + 0.02), o, "descend"),
        Waypoint(1.0, (gx0, sy, z0), o, "fingers either side of the rail"),
        Waypoint(1.0, (gx0, sy, z0), c, "grip"),
        Waypoint(1.0, (gx0, sy, lift), c, "lift a little"),
        Waypoint(1.0, (back, sy, lift), c, "draw back"),
        Waypoint(2.0, (back, MY + Y_TRIM, lift), c, "onto the mortise's line"),
        Waypoint(1.0, (back, MY + Y_TRIM, carry), c, "lower to just above the bench"),
        Waypoint(0.5, (back, MY + Y_TRIM, carry), c, "settle"),
        Waypoint(3.0, (home + PUSH_PAST, MY + Y_TRIM, carry), c, "push straight in along the mortise"),
        Waypoint(1.0, (home + PUSH_PAST, MY + Y_TRIM, carry), c, "hold: shoulder home"),
        Waypoint(1.0, (home + PUSH_PAST, MY + Y_TRIM, z0 - 0.001), c, "set down"),
        Waypoint(1.0, (home + PUSH_PAST, MY + Y_TRIM, z0 - 0.001), o, "let go"),
        Waypoint(1.2, (home - 0.010, MY + Y_TRIM, TRAVEL_R), o, "up"),
        Waypoint(1.5, (0.180, -0.250, TRAVEL_R), o, "clear"),
    ]


def _left(wait_s):
    px, py = PEG_XY
    hx, hy = FACE_X + HOLE_X, MY
    z0 = PEG_TIPS + TIP
    drop = TOP + 0.008 + 0.020 + TIP  # peg foot 8 mm above the bench (inside the stile's hole)
    o, c = PEG_OPEN, FINGER_CLOSED
    return [
        Waypoint(2.0, PARK_L, o, "park out of the right arm's way"),
        Waypoint(max(0.5, wait_s), PARK_L, o, "wait for the rail"),
        Waypoint(2.0, (px, py, TRAVEL_L), o, "over the peg"),
        Waypoint(1.5, (px, py, z0 + 0.02), o, "descend"),
        Waypoint(1.0, (px, py, z0), o, "fingers either side of the peg"),
        Waypoint(1.0, (px, py, z0), c, "grip"),
        Waypoint(1.5, (px, py, TRAVEL_L), c, "lift the peg out"),
        Waypoint(2.5, (hx, hy, TRAVEL_L), c, "over the peg hole"),
        Waypoint(0.8, (hx, hy, TRAVEL_L), c, "settle"),
        Waypoint(2.0, (hx, hy, drop), c, "lower the peg into the hole"),
        Waypoint(0.5, (hx, hy, drop), c, "settle"),
        Waypoint(1.0, (hx, hy, drop), o, "let go"),
        Waypoint(1.0, (hx, hy, TOP + 0.075 + TIP), o, "up"),
        Waypoint(0.8, (hx, hy, TOP + 0.075 + TIP), c, "close the fingers"),
        Waypoint(1.2, (hx, hy, TOP + PEG[2] - 0.002 + TIP), c, "tap the peg down"),
        Waypoint(1.0, (hx, hy, TRAVEL_L), c, "up"),
        Waypoint(1.5, (0.180, 0.250, TRAVEL_L), c, "clear"),
    ]


def demo():
    """Right arm: the tenon home and let go (about 19 s). Left arm: waits over the peg until the right arm is clear,
    then pegs the joint. About 37 s."""
    right = _right()
    t_clear = sum(w.duration for w in right)
    return {"left": _left(t_clear - 2.0 - 2.0), "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def _states(rail_xy=None, peg=None, peg_tilt=0.0):
    """Kit states with the rail's shoulder at ``rail_xy`` (on the bench) and the peg's foot at ``peg`` (x, y, z)."""
    moved = {}
    if rail_xy is not None:
        moved["rail"] = (np.array([rail_xy[0], rail_xy[1], TOP]), np.array([0.0, 0.0, 0.0, 1.0]))
    if peg is not None:
        q = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(peg_tilt)))
        moved["peg"] = (np.array(peg, float), q)
    return kit.fake_states(moved=moved)


def _geometry_check() -> None:
    """Clearances: tenon in mortise 1 mm a side, peg in holes 1 mm a side; the holes line up when home; a peg in
    the empty mortise stops the tenon short of 25 mm; a peg on an unseated tenon stands 14 mm off the bench."""
    assert abs((MORT_W - TEN_W) / 2 - 0.001) < 1e-9 and abs((MORT_H - TEN_H) / 2 - 0.001) < 1e-9
    assert abs((HOLE - PEG[0]) / 2 - 0.001) < 1e-9
    blocked = HOLE_X - PEG[0] / 2  # the tenon's end meets a peg standing in the empty mortise
    assert blocked < PART_IN, blocked
    assert (T + TEN_H) / 2 > PEG_DOWN
    # nothing overlaps at the start: the tenon's end is clear of the stile's end
    st = kit.fake_states()
    assert st["rail"].high[0] <= FACE_X or st["rail"].high[1] < STILE_Y[0], (st["rail"].high,)
    print(f"tenon blocked by an early peg at {blocked * 1000:.0f} mm (credit from {PART_IN * 1000:.0f} mm)",
          file=sys.stderr)


def self_test():
    """The scoring rules on hand-built poses."""
    global _TEST_TCPS
    import dataclasses

    _geometry_check()
    hole = (FACE_X + HOLE_X, MY)
    home = (FACE_X, MY)
    peg_down = (*hole, TOP)
    start = _states()
    done = _states(home, peg_down)
    moving = dict(done)
    moving["peg"] = dataclasses.replace(moving["peg"], lin_vel=np.array([0.0, 0.0, -0.05]))
    cases = {  # name: (state, tcps, score)
        "start": (start, [], 0.0),
        "home and pegged": (done, [], 1.0),
        "home, peg 1.5 mm short of home (still counts)": (_states((FACE_X - 0.0015, MY), peg_down), [], 1.0),
        "home, peg still in its block": (_states(home), [], 0.5),
        "home, peg resting on the tenon (holes not lined up)": (_states(home, (*hole, TOP + 0.014)), [], 0.5),
        "home, peg lying on the stile": (_states(home, (*hole, TOP + 0.020), peg_tilt=90.0), [], 0.5),
        "home, peg still falling": (moving, [], 0.5),
        "home, peg still held": (done, [(hole[0], hole[1], TOP + 0.060)], 0.5),
        "tenon 30 mm in": (_states((FACE_X - TEN_L + 0.030, MY)), [], 0.2),
        "peg first: tenon stopped 19 mm in by the peg": (_states((FACE_X - TEN_L + 0.019, MY), peg_down), [], 0.0),
        "rail off the mortise's line, against the stile": (_states((FACE_X - TEN_L, MY + 0.03), peg_down), [], 0.0),
        "home but still held": (done, [(FACE_X - GRIP_BACK, MY, TOP + 0.01)], 0.0),
    }
    try:
        for name, (st, tcps, want) in cases.items():
            _TEST_TCPS = tcps
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
        assert abs(tenon_depth(done) - TEN_L) < 1e-9
    finally:
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: geometry and the scoring rules on hand-built states, then both arms' plans replayed
    # through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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

