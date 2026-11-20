"""A lube bay bench. A driveshaft slip joint (a 50 mm steel barrel on two bench stands, strapped down) has one
grease fitting: a straight 1/4-28 SAE J534 fitting (zerk) screwed into the barrel's side, pointing at the robot's
right, 7 cm above the bench. On a rubber mat at the right lies a 3 oz pistol-grip grease gun on its side: barrel,
pump head, a 10 cm rigid pipe and a standard four-jaw coupler pointing at the fitting; its handle and pump lever
stick out toward the robot.

Shop practice ("lube, oil, filter"): the coupler is pushed square onto the fitting until its four sprung jaws snap
over the fitting's head, the gun is held there while the lever is squeezed for the number of shots on the service
sheet (three here), the coupler is pulled off and the gun goes back on its mat. A coupler that is not home pumps
grease past the fitting; too many shots blow the joint's seal.

Scored from the poses: the coupler home on the fitting with all four jaws closed behind the head; each full lever
stroke made while it is home; the coupler off the fitting again; the gun lying on its mat, let go and still.

Mechanism parts come from mechkit.py (in this folder): the four jaws are free bodies on sprung radial slides in the
coupler, the lever is a sprung hinge on the pump head, and the fitting's head is a convex collider; nothing latches
by proximity.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/grease-chassis-fitting-with-grease-gun/\\
           grease_chassis_fitting_with_grease_gun.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
           -E cam_height=176 --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box parts, sprung slides and hinges, per-episode hooks; installed in memory)

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the bench top

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
STEEL = (0.62, 0.63, 0.65)
DARK = (0.16, 0.16, 0.17)
RED = (0.72, 0.10, 0.08)
ZINC = (0.80, 0.70, 0.30)
RUBBER = (0.10, 0.10, 0.11)
SLICK = (0.12, 0.12)  # greased steel on steel (static, dynamic), min-combined


def _ry(deg: float) -> tuple:
    """Quaternion (xyzw) of a turn about y."""
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def _rx(deg: float) -> tuple:
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _octagon_y(across: float, length: float, y: float, color=None, friction=None) -> list:
    """An octagonal bar along the part's y: four boxes turned 45 deg apart about y."""
    side = across * math.tan(math.pi / 8) + 0.0002
    return [mk.box((across, length, side), at=(0.0, y, 0.0), quat=_ry(45.0 * k), color=color, friction=friction)
            for k in range(4)]


# -- the grease fitting: straight 1/4-28 SAE J534, pointing along -y ------------------------------------------------
FIT_X, FIT_Z = 0.300, TOP + 0.070  # the fitting's axis
HEX_FACE_Y = 0.100  # where the fitting's hex meets the barrel
FIT_AXIS = np.array([0.0, -1.0, 0.0])  # out of the barrel, toward the gun
# profile along the axis, mm from the barrel surface: hex 0 to 4.0 (8.0 across flats), neck 4.0 to 7.0 (4.8),
# head 7.0 to 11.5 (6.6 at its widest, a steep shoulder behind and a rounded nose in front)
HEX_L, NECK_END, TIP = 0.0040, 0.0070, 0.0115
HEAD_PROFILE = [(0.0070, 0.0024), (0.0075, 0.0033), (0.0093, 0.0033), (0.0107, 0.0020), (0.0115, 0.0012)]
FIT_TIP = np.array([FIT_X, HEX_FACE_Y - TIP, FIT_Z])
HOME_GAP_M = 0.0045  # mouth to barrel surface with the fitting's nose on the coupler's seat
HOME_MOUTH = np.array([FIT_X, HEX_FACE_Y - HOME_GAP_M, FIT_Z])  # the coupler's mouth when it is home


def _head_mesh():
    """The fitting's head: a convex solid of revolution (octagonal rings), in the fitting's frame (axis -y)."""
    verts, faces, n = [], [], 8
    for s, r in HEAD_PROFILE:
        rv = r / math.cos(math.pi / n)
        for k in range(n):
            a = 2 * math.pi * (k + 0.5) / n
            verts.append((rv * math.cos(a), -s, rv * math.sin(a)))
    rings = len(HEAD_PROFILE)
    for i in range(rings - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            faces.append((a, b, b + n, a + n))
    faces.append(tuple(range(n - 1, -1, -1)))
    faces.append(tuple(range((rings - 1) * n, rings * n)))
    return verts, faces


_hv, _hf = _head_mesh()
kit = mk.Kit()
kit.part("fitting", (FIT_X, HEX_FACE_Y, FIT_Z),
         [*_octagon_y(0.0080, HEX_L, -HEX_L / 2, friction=SLICK),
          *_octagon_y(0.0048, NECK_END - HEX_L + 0.0004, -(HEX_L + NECK_END) / 2, friction=SLICK),
          mk.convex(_hv, _hf, friction=SLICK)],
         color=ZINC, friction=SLICK, contact_offset=0.001)

# -- the slip joint: a 50 mm barrel along x on two stands, its yoke at the far end ---------------------------------
BARREL_R = 0.025
BARREL_Y = HEX_FACE_Y + BARREL_R
BARREL_X0, BARREL_X1 = 0.250, 0.500
_bl, _bx = BARREL_X1 - BARREL_X0, (BARREL_X0 + BARREL_X1) / 2
_side = 2 * BARREL_R * math.tan(math.pi / 8) + 0.0002
STAND_TOP = FIT_Z - BARREL_R
kit.part("slip_joint", (_bx, BARREL_Y, FIT_Z),
         [*[mk.box((_bl, 2 * BARREL_R, _side), quat=_rx(45.0 * k)) for k in range(4)],
          *[mk.box((0.022, 0.058, 0.058 * math.tan(math.pi / 8)), at=(BARREL_X0 - _bx + 0.011, 0, 0),
                   quat=_rx(45.0 * k), color=DARK) for k in range(4)],  # the dust cap over the splines
          mk.box((0.060, 0.012, 0.052), at=(BARREL_X1 - _bx + 0.030, 0.034, 0.0)),  # yoke ears
          mk.box((0.060, 0.012, 0.052), at=(BARREL_X1 - _bx + 0.030, -0.034, 0.0)),
          mk.box((0.012, 0.080, 0.052), at=(BARREL_X1 - _bx + 0.006, 0.0, 0.0))],
         color=STEEL)
STANDS = [Box(f"stand_{n}", pos=(x, BARREL_Y, (TOP + STAND_TOP) / 2), size=(0.040, 0.090, STAND_TOP - TOP), color=DARK)
          for n, x in (("near", 0.375), ("far", 0.465))]
STRAPS = [Box(f"strap_{n}", pos=(x, BARREL_Y, FIT_Z + BARREL_R + 0.0015), size=(0.025, 0.060, 0.003), color=RED)
          for n, x in (("near", 0.375), ("far", 0.465))]

# -- the grease gun, lying on its side. Part frame: origin on the axis at the barrel's middle, +y to the coupler,
#    handle and lever toward -x, z up -------------------------------------------------------------------------------
BARREL_D, BARREL_L = 0.042, 0.125
HEAD = (0.042, 0.055, 0.034)
HEAD_Y = BARREL_L / 2 + HEAD[1] / 2
PIPE_L = 0.100
PIPE_Y0 = BARREL_L / 2 + HEAD[1]
COUPLER_OD, COUPLER_L, BORE = 0.015, 0.038, 0.0072
MOUTH = PIPE_Y0 + PIPE_L + COUPLER_L  # the coupler's mouth, from the gun's origin: 0.2555
SEAT_DEPTH = 0.0070  # the fitting's nose meets the coupler's seat this far inside the mouth
HANDLE = (0.110, 0.014, 0.022)
HANDLE_Y = BARREL_L / 2 + 0.020
HANDLE_X = -(HEAD[0] / 2 + HANDLE[0] / 2)
LEVER = (0.100, 0.012, 0.016)
LEVER_PIVOT = np.array([-HEAD[0] / 2, HANDLE_Y + 0.030, 0.0])
LEVER_OPEN_DEG = 20.0  # splay of the released lever from the handle
LEVER_TRAVEL_DEG = 18.0
LEVER_K = 2.0  # N*m/rad
LEVER_REST_DEG = -10.0  # spring rest, behind the released stop: 0.35 N*m of preload
GUN_KG, LEVER_KG, JAW_KG = 0.50, 0.03, 0.02
_wall = (COUPLER_OD - BORE) / 2
_cy = MOUTH - COUPLER_L / 2
_wo = BORE / 2 + _wall / 2


def _gun_boxes():
    b = _octagon_y(BARREL_D, BARREL_L, 0.0, color=RED)
    b.append(mk.box((0.036, 0.010, 0.036), at=(0, -BARREL_L / 2 - 0.005, 0), color=DARK))  # end cap
    b.append(mk.box((0.006, 0.022, 0.006), at=(0, -BARREL_L / 2 - 0.021, 0), color=STEEL))  # follower rod
    b.append(mk.box((0.030, 0.006, 0.008), at=(0, -BARREL_L / 2 - 0.035, 0), color=STEEL))  # its T handle
    b.append(mk.box(HEAD, at=(0, HEAD_Y, 0), color=DARK))
    b.append(mk.box(HANDLE, at=(HANDLE_X, HANDLE_Y, 0), color=DARK))
    b += _octagon_y(0.008, PIPE_L, PIPE_Y0 + PIPE_L / 2, color=STEEL)
    b.append(mk.box((COUPLER_OD, COUPLER_L, _wall), at=(0, _cy, _wo), color=STEEL, friction=SLICK))
    b.append(mk.box((COUPLER_OD, COUPLER_L, _wall), at=(0, _cy, -_wo), color=STEEL, friction=SLICK))
    b.append(mk.box((_wall, COUPLER_L, BORE), at=(_wo, _cy, 0), color=STEEL, friction=SLICK))
    b.append(mk.box((_wall, COUPLER_L, BORE), at=(-_wo, _cy, 0), color=STEEL, friction=SLICK))
    b.append(mk.box((BORE, 0.0030, BORE), at=(0, MOUTH - SEAT_DEPTH - 0.0015, 0), color=STEEL, friction=SLICK))
    return b


# Start: on the mat, the coupler's mouth 14 cm short of the fitting's nose.
MAT_TOP = TOP + 0.003
GUN_X = FIT_X
GUN_Y0 = -0.045 - MOUTH  # the gun's origin at the start
GUN_Z0 = MAT_TOP + BARREL_D / 2
GUN_START = np.array([GUN_X, GUN_Y0, GUN_Z0])
kit.part("gun", tuple(GUN_START + (0, 0, 0.0005)), _gun_boxes(), mass=GUN_KG, color=RED, contact_offset=0.002)

# the lever: a sprung hinge on the pump head, squeezed toward the handle
_lu = np.array([-math.cos(math.radians(LEVER_OPEN_DEG)), math.sin(math.radians(LEVER_OPEN_DEG)), 0.0])
_lever_at = tuple(GUN_START + (0, 0, 0.0005) + LEVER_PIVOT)
kit.part("lever", _lever_at, [mk.box(LEVER, at=tuple((0.005 + LEVER[0] / 2) * _lu), yaw=-math.radians(LEVER_OPEN_DEG))],
         mass=LEVER_KG, color=STEEL)
kit.hinge("pump", "gun", "lever", anchor=_lever_at, axis=(0, 0, 1), lower_deg=0.0, upper_deg=LEVER_TRAVEL_DEG,
          stiffness=LEVER_K, rest_deg=LEVER_REST_DEG, damping=0.02)

# the four jaws: free bodies on sprung radial slides just inside the mouth
JAW = (0.0030, 0.0020, 0.0030)  # radial, along the axis, across
JAW_DEPTH = 0.0013  # the jaws' middle, inside the mouth
JAW_REACH = 0.0010  # how far a closed jaw stands into the bore
JAW_TRAVEL = 0.0013
JAW_K, JAW_PRELOAD_M = 600.0, 0.0010  # 0.6 N at the closed stop
JAW_LEAD = 0.0008  # 45 degree lead-in on each jaw's mouth-side edge
JAWS = {"jaw_px": (1, 0, 0), "jaw_nx": (-1, 0, 0), "jaw_pz": (0, 0, 1), "jaw_nz": (0, 0, -1)}


def _jaw_mesh(d):
    """One jaw as a convex prism in its part frame (origin at the middle of its inner face, +y to the mouth):
    a flat inner face with a 45 degree lead-in toward the mouth and a square edge behind."""
    h, w = JAW[1] / 2, JAW[2] / 2
    profile = [(JAW_LEAD, h), (0.0, h - JAW_LEAD), (0.0, -h), (JAW[0], -h), (JAW[0], h)]  # (radial out, along y)
    t = np.cross(d, (0.0, 1.0, 0.0))  # across the jaw
    verts = [tuple(r * d + np.array([0.0, y, 0.0]) + side * w * t) for side in (-1, 1) for r, y in profile]
    n = len(profile)
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
    return verts, faces


for _name, _d in JAWS.items():
    _d = np.array(_d, float)
    _jv, _jf = _jaw_mesh(_d)
    kit.part(_name, tuple(GUN_START + (0, 0, 0.0005) + (BORE / 2 - JAW_REACH) * _d + (0, MOUTH - JAW_DEPTH, 0)),
             [mk.convex(_jv, _jf)], mass=JAW_KG, color=DARK, friction=SLICK, contact_offset=0.001)
    kit.slider(f"{_name}_slide", "gun", _name, axis=tuple(_d), lower=0.0, upper=JAW_TRAVEL, stiffness=JAW_K,
               rest=-JAW_PRELOAD_M, damping=0.6)

MAT = Box("mat", pos=(0.245, -0.225, TOP + 0.0015), size=(0.190, 0.400, 0.003), color=RUBBER)
OBJECTS = [*kit.objects(), *STANDS, *STRAPS, MAT]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring -------------------------------------------------------------------------------------------------------

SHOTS = 3  # the service sheet's count
BURST_SHOTS = 6  # this many shots into the joint blow its seal
HOME_GAP = (0.0036, 0.0052)  # mouth to barrel surface when home (4.5 mm nominal): jaws behind the head's shoulder
HOME_SIDE = 0.0012  # mouth centre off the fitting's axis
HOME_TILT_DEG = 6.0
JAW_CLOSED = 0.0004  # a jaw this close to its closed stop has dropped behind the head
HOME_STEPS = 5  # home for 0.25 s counts as coupled
STROKE_FULL_DEG, STROKE_BACK_DEG = 15.0, 4.0
OFF_CLEAR = 0.020  # the mouth this far from the fitting's nose: off the fitting
MAT_LO, MAT_HI = np.array([0.150, -0.425]), np.array([0.340, -0.025])
LYING_DEG = 12.0
ON_MAT_GAP = 0.006
CLEAR = 0.020  # every robot collision piece this far from the gun
_TEST_ROBOT = None  # self-test stand-in for the robot's collision boxes
_DEBUG = int(os.environ.get("GREASE_DEBUG", "0"))  # print the readouts every this many control steps


def _mouth(objects):
    """(mouth centre, coupler axis) in the world."""
    pos, quat = kit.pose(objects, "gun")
    r = mk.quat_to_mat(quat)
    return np.asarray(pos) + r @ np.array([0.0, MOUTH, 0.0]), r @ np.array([0.0, 1.0, 0.0])


def _gap(objects):
    """(mouth to barrel surface along the fitting, mouth centre off the axis, tilt deg)."""
    m, a = _mouth(objects)
    rel = m - np.array([FIT_X, HEX_FACE_Y, FIT_Z])
    along = float(rel @ FIT_AXIS)
    side = float(np.linalg.norm(rel - along * FIT_AXIS))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(-(a @ FIT_AXIS))))))
    return along, side, tilt


def _jaws_closed(objects) -> bool:
    return all(kit.q(objects, f"{n}_slide") <= JAW_CLOSED for n in JAWS)


def home(objects) -> bool:
    """The coupler fully on the fitting: mouth 3.6 to 5.2 mm from the barrel, square, all four jaws closed."""
    along, side, tilt = _gap(objects)
    return bool(HOME_GAP[0] <= along <= HOME_GAP[1] and side <= HOME_SIDE and tilt <= HOME_TILT_DEG
                and _jaws_closed(objects))


def off_fitting(objects) -> bool:
    m, _ = _mouth(objects)
    return bool(np.linalg.norm(m - FIT_TIP) >= OFF_CLEAR and _jaws_closed(objects))


def lever_deg(objects) -> float:
    return kit.q_deg(objects, "pump")


def _robot_clear(state) -> bool:
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT is None else _TEST_ROBOT
    lo, hi = np.asarray(state.low) - CLEAR, np.asarray(state.high) + CLEAR
    return not any(bool(np.all(np.asarray(b_hi) >= lo) and np.all(np.asarray(b_lo) <= hi)) for b_lo, b_hi in boxes)


def on_mat(objects) -> bool:
    """The gun lying on its mat, wholly over it, let go (every robot collision piece 2 cm clear) and still."""
    st = objects["gun"]
    _, a = _mouth(objects)
    lying = math.degrees(math.asin(min(1.0, abs(float(a[2]))))) <= LYING_DEG
    inside = bool(np.all(np.asarray(st.low)[:2] >= MAT_LO) and np.all(np.asarray(st.high)[:2] <= MAT_HI))
    down = float(st.low[2]) <= MAT_TOP + ON_MAT_GAP
    return bool(lying and inside and down and st.resting() and _robot_clear(st))


def _observe(objects) -> None:
    """Per control step: coupled latch, lever strokes (in the fitting or spilt), the two real failures."""
    m = mk.MEMORY
    pos, _ = kit.pose(objects, "gun")
    if float(pos[2]) < TOP - 0.05:
        m["dropped"] = True
    is_home = home(objects)
    m["home_run"] = m.get("home_run", 0) + 1 if is_home else 0
    if m["home_run"] >= HOME_STEPS:
        m["coupled"] = True
    q = lever_deg(objects)
    if m.get("pressing"):
        m["press_home"] = m.get("press_home", True) and is_home
        if q <= STROKE_BACK_DEG:
            m["pressing"] = False
    elif q >= STROKE_FULL_DEG and m.get("armed", True):
        m["armed"] = False
        if is_home and m.get("stroke_home", True):
            m["shots"] = m.get("shots", 0) + 1
        else:
            m["spilt"] = m.get("spilt", 0) + 1
    if q <= STROKE_BACK_DEG:
        m["armed"] = True
        m["stroke_home"] = True
    elif not is_home:
        m["stroke_home"] = False  # the coupler came off home during this stroke
    if m.get("shots", 0) >= BURST_SHOTS:
        m["burst"] = True
    if _DEBUG:
        m["step"] = m.get("step", 0) + 1
        if m["step"] % _DEBUG == 0:
            along, side, tilt = _gap(objects)
            jaws = [round(kit.q(objects, f"{n}_slide") * 1000, 2) for n in JAWS]
            print(f"[grease] {m['step']:4d} gap {along * 1000:7.2f} side {side * 1000:5.2f} tilt {tilt:5.2f} jaws {jaws} "
                  f"lever {q:5.1f} home {int(is_home)} coupled {int(bool(m.get('coupled')))} shots {m.get('shots', 0)} "
                  f"spilt {m.get('spilt', 0)} gun {np.round(pos, 4).tolist()} "
                  f"mouth-home {np.round((_mouth(objects)[0] - HOME_MOUTH) * 1000, 2).tolist()} "
                  f"tcps {[np.round(t, 3).tolist() for t in mk.tcps()]} score {score(objects)}", flush=True)


kit.on_step(_observe)


def score(objects) -> float:
    """0.10 coupled; +0.20 per shot into the fitting up to three; +0.15 coupler off the fitting again; +0.15 gun on
    its mat, let go and still. -0.20 per shot beyond three, -0.10 once for any stroke spilt beside the fitting.
    0 if the gun was dropped off the bench or six shots blew the seal."""
    m = mk.MEMORY
    if m.get("dropped") or m.get("burst"):
        return 0.0
    if not m.get("coupled"):
        return 0.0
    shots = m.get("shots", 0)
    s = 0.10 + 0.20 * min(shots, SHOTS) - 0.20 * max(0, shots - SHOTS) - (0.10 if m.get("spilt") else 0.0)
    if off_fitting(objects):
        s += 0.15
        if on_mat(objects):
            s += 0.15
    return round(max(0.0, min(1.0, s)), 4)


def success(objects) -> bool:
    """Coupled, exactly three shots into the fitting, none spilt, coupler off, gun on its mat, let go; 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from the mouth to home while shots are owed, then from the gun to its start on the mat."""
    m = mk.MEMORY
    owed = max(0, SHOTS - m.get("shots", 0))
    pos, _ = kit.pose(objects, "gun")
    if owed:
        mouth, _ = _mouth(objects)
        return float(0.05 * owed + np.linalg.norm(mouth - HOME_MOUTH))
    return float(np.linalg.norm(np.asarray(pos) - GUN_START))


# -- the scripted demo ----------------------------------------------------------------------------------------------
# Open loop from the fixed layout. Right arm: pinch the barrel across its flats, lift the gun level with the
# fitting, push the coupler on, hold it there, pull it off, lay the gun back on the mat. Left arm: straddle the
# handle and the lever from above and squeeze them together three times, following the handle so the gun is not
# pushed.

VARIANT = os.environ.get("DEMO_VARIANT", "full")  # full | air | short | five | left_on
# Fingers closing along x. The pinched gun settles turned about 2.4 degrees in the pads (measured in sim), so the
# hand is turned that much the other way.
YAW_X = math.pi / 2 - math.radians(float(os.environ.get("GREASE_YAW_DEG", "2.38")))
BARREL_GAP_OPEN, BARREL_GAP_SHUT = 0.070, 0.030
TRAVEL_Z = TOP + 0.150
# The arm gives under load: with the 0.61 kg gun in hand the grip settles this far (m) from where it is commanded
# (x, y, z), so the carry is commanded that much the other way; pushes and pulls are commanded well past the
# point they reach.
SAG = tuple(float(v) for v in os.environ.get("GREASE_SAG", "0.0,0.002,0.0164").split(","))
SAG_REST = tuple(float(v) for v in os.environ.get("GREASE_SAG_REST", "0.002,0.0,0.009").split(","))
PUSH_PAST = float(os.environ.get("GREASE_PUSH", "0.045"))  # commanded past home: about 9 N on the seat
PULL_BACK = float(os.environ.get("GREASE_PULL", "0.075"))  # commanded behind home to draw the jaws off the head


def _balance_y() -> float:
    """Where along its axis the loaded gun balances (gun body by its solids' volumes, lever and jaws by mass)."""
    vol = sum(float(np.prod(b.size)) for b in kit.parts["gun"].boxes)
    gun = sum(float(np.prod(b.size)) * b.at[1] for b in kit.parts["gun"].boxes) / vol
    lever = LEVER_PIVOT[1] + (0.005 + LEVER[0] / 2) * math.sin(math.radians(LEVER_OPEN_DEG))
    return (GUN_KG * gun + LEVER_KG * lever + 4 * JAW_KG * (MOUTH - JAW_DEPTH)) / (GUN_KG + LEVER_KG + 4 * JAW_KG)


GRIP_Y = float(os.environ.get("GREASE_GRIP_Y", f"{_balance_y():.4f}"))  # pinch the barrel where the gun balances
SQUEEZE_AT = float(os.environ.get("GREASE_SQUEEZE_AT", "0.090"))  # along the lever from its pivot
LEFT_YAW = float(os.environ.get("GREASE_LEFT_YAW", "0.0"))  # the pumping hand: fingers closing along y
SQUEEZE_GAP_OPEN, SQUEEZE_GAP_SHUT = 0.092, 0.038
HOME_Y = HEX_FACE_Y - HOME_GAP_M - MOUTH  # the gun's origin with the coupler home


def _finger(gap: float) -> float:
    return max(FINGER_OPEN, -gap / 2)


def _right(variant):
    o, c = _finger(BARREL_GAP_OPEN), _finger(BARREL_GAP_SHUT)
    x = GUN_X
    y0 = GUN_Y0 + GRIP_Y
    grip_z = GUN_Z0 + 0.004
    xc = x + SAG[0]
    carry_z = FIT_Z + 0.004 + SAG[2]
    home_y = HOME_Y + GRIP_Y + SAG[1]
    pre = home_y - 0.030
    if variant == "air":
        seat = pre
    elif variant == "short":
        seat = home_y - 0.0045
    else:
        seat = home_y + PUSH_PAST
    n = 5 if variant == "five" else SHOTS
    hold = 9.7 + n * 2.6  # until the pumping hand has finished and risen
    w = [
        Waypoint(2.0, (x, y0, TRAVEL_Z), o, "over the barrel", YAW_X),
        Waypoint(1.6, (x, y0, grip_z + 0.020), o, "down round the barrel", YAW_X),
        Waypoint(0.8, (x, y0, grip_z), o, "fingers beside the flats", YAW_X),
        Waypoint(1.2, (x, y0, grip_z), c, "pinch the barrel", YAW_X),
        Waypoint(0.4, (x, y0, grip_z), c, "settle", YAW_X),
        Waypoint(2.0, (xc, y0 + SAG[1], carry_z), c, "lift level with the fitting", YAW_X),
        Waypoint(2.5, (xc, pre, carry_z), c, "bring the coupler up to the fitting", YAW_X),
        Waypoint(1.5, (xc, pre, carry_z), c, "settle in line", YAW_X),
        Waypoint(3.0, (xc, seat, carry_z), c, "push the coupler on", YAW_X),
        Waypoint(hold, (xc, seat, carry_z), c, "hold it home while the other hand pumps", YAW_X),
    ]
    if variant == "left_on":
        w += [Waypoint(1.5, (xc, home_y, carry_z), c, "ease the push", YAW_X),
              Waypoint(1.2, (xc, home_y, carry_z), o, "let go with the gun on the fitting", YAW_X),
              Waypoint(1.5, (x, home_y, TRAVEL_Z), o, "rise", YAW_X),
              Waypoint(1.5, (0.200, -0.300, TRAVEL_Z), o, "clear", YAW_X)]
        return w
    back = home_y - PULL_BACK
    down = (x + SAG_REST[0], y0 + SAG_REST[1], grip_z + 0.001 + SAG_REST[2])
    w += [
        Waypoint(2.0, (xc, back, carry_z), c, "pull the coupler off", YAW_X),
        Waypoint(0.8, (xc, home_y - 0.040, carry_z), c, "settle", YAW_X),
        Waypoint(2.0, (xc, y0 + SAG[1], carry_z), c, "back over the mat", YAW_X),
        Waypoint(1.6, down, c, "lay the gun down", YAW_X),
        Waypoint(0.5, down, c, "settle", YAW_X),
        Waypoint(1.2, down, o, "let go", YAW_X),
        Waypoint(1.5, (x, y0, TRAVEL_Z), o, "rise", YAW_X),
        Waypoint(1.5, (0.200, -0.300, TRAVEL_Z), o, "clear", YAW_X),
    ]
    return w


def _left(variant):
    o, c = _finger(SQUEEZE_GAP_OPEN), _finger(SQUEEZE_GAP_SHUT)
    gy = (HOME_Y - 0.030) if variant == "air" else (HOME_Y - 0.0045 if variant == "short" else HOME_Y)
    x = GUN_X + LEVER_PIVOT[0] - SQUEEZE_AT
    dx = -SQUEEZE_AT
    s_open = -dx / math.cos(math.radians(LEVER_OPEN_DEG))
    lever_y_open = gy + LEVER_PIVOT[1] + s_open * math.sin(math.radians(LEVER_OPEN_DEG))
    handle_y = gy + HANDLE_Y
    handle_lo = handle_y - HANDLE[1] / 2
    lever_hi_open = lever_y_open + LEVER[1] / 2
    y_open = (handle_lo + lever_hi_open) / 2

    rem_deg = LEVER_OPEN_DEG - LEVER_TRAVEL_DEG
    s_closed = -dx / math.cos(math.radians(rem_deg))
    lever_y_closed = gy + LEVER_PIVOT[1] + s_closed * math.sin(math.radians(rem_deg))
    lever_hi_closed = lever_y_closed + LEVER[1] / 2
    y_shut = (handle_lo + lever_hi_closed) / 2

    z = FIT_Z + 0.001 - float(os.environ.get("GREASE_SAG_LEFT", "0.040"))
    park = (0.200, 0.250, TRAVEL_Z)
    n = 5 if variant == "five" else SHOTS
    w = [
        Waypoint(2.0, park, o, "wait clear", LEFT_YAW),
        Waypoint(14.5, park, o, "wait for the coupler", LEFT_YAW),
        Waypoint(2.0, (x, y_open, TRAVEL_Z), o, "over the handle and lever", LEFT_YAW),
        Waypoint(3.6, (x, y_open, z), o, "down astride them", LEFT_YAW),
        Waypoint(1.0, (x, y_open, z), o, "settle", LEFT_YAW),
    ]
    for k in range(n):
        w += [
            Waypoint(1.0, (x, y_shut, z), c, f"squeeze {k + 1}", LEFT_YAW),
            Waypoint(0.4, (x, y_shut, z), c, "hold", LEFT_YAW),
            Waypoint(0.8, (x, y_open, z), o, "let the lever back", LEFT_YAW),
            Waypoint(0.4, (x, y_open, z), o, "lever out", LEFT_YAW),
        ]
    w += [
        Waypoint(1.5, (x, y_open, TRAVEL_Z), o, "rise", LEFT_YAW),
        Waypoint(1.5, park, o, "clear", LEFT_YAW),
    ]
    return w


def demo():
    """Both arms: couple, three shots, uncouple, gun back on the mat. About 35 s. DEMO_VARIANT picks a wrong way:
    air (pumps with the coupler 3 cm short), short (coupler 4.5 mm short of home), five (five shots), left_on."""
    return {"right": _right(VARIANT), "left": _left(VARIANT)}


# -- offline self-test ----------------------------------------------------------------------------------------------


def _states(gun_y=None, gun_z=None, lever=0.0, jaws=0.0, tilt_deg=0.0, side=0.0, moving=False):
    import dataclasses

    y = GUN_Y0 if gun_y is None else gun_y
    z = GUN_Z0 if gun_z is None else gun_z
    r = mk.axis_angle((1, 0, 0), math.radians(tilt_deg))
    pos = np.array([GUN_X + side, y, z])
    if tilt_deg:  # tilt about the mouth, so the mouth stays put
        pos = pos + np.array([0.0, MOUTH, 0.0]) - r @ np.array([0.0, MOUTH, 0.0])
    q = {"pump": math.radians(lever), **{f"{n}_slide": jaws for n in JAWS}}
    st = kit.fake_states(q=q, moved={"gun": (pos, mk.mat_to_quat(r))})
    if moving:
        st["gun"] = dataclasses.replace(st["gun"], lin_vel=np.array([0.0, 0.05, 0.0]))
    return st


def self_test():
    """The scoring rules on hand-built poses: coupled latch, shot counting, spills, burst, mat and release."""
    global _TEST_ROBOT
    at_home = dict(gun_y=HOME_Y, gun_z=FIT_Z)
    rest = _states()
    seated = _states(**at_home)
    press = _states(**at_home, lever=17.0)
    shot = [press, seated]
    air = _states(gun_y=HOME_Y - 0.030, gun_z=FIT_Z)
    air_press = _states(gun_y=HOME_Y - 0.030, gun_z=FIT_Z, lever=17.0)
    off = _states(gun_y=HOME_Y - 0.050, gun_z=FIT_Z)
    cocked = dict(gun_y=HOME_Y - 0.0045, gun_z=FIT_Z, jaws=0.0007)
    hand = [(np.array([GUN_X - 0.03, GUN_Y0 - 0.02, GUN_Z0]), np.array([GUN_X + 0.03, GUN_Y0 + 0.02, GUN_Z0 + 0.12]))]
    cases = {  # name: ([states in time order], robot collision boxes at the end, score)
        "start: nothing done": ([rest], [], 0.0),
        "coupled only": ([seated] * 6, [], 0.10),
        "home for 0.2 s only": ([seated] * 4 + [off], [], 0.0),
        "one shot, still coupled": ([seated] * 6 + shot, [], 0.30),
        "three shots, still coupled": ([seated] * 6 + shot * 3, [], 0.70),
        "three shots, off, gun held in the air": ([seated] * 6 + shot * 3 + [off], [], 0.85),
        "three shots, gun back on the mat": ([seated] * 6 + shot * 3 + [off, rest], [], 1.0),
        "gun on the mat but a finger still on it": ([seated] * 6 + shot * 3 + [off, rest], hand, 0.85),
        "gun on the mat, still sliding": ([seated] * 6 + shot * 3 + [off, _states(moving=True)], [], 0.85),
        "two shots, gun back": ([seated] * 6 + shot * 2 + [off, rest], [], 0.80),
        "five shots, gun back": ([seated] * 6 + shot * 5 + [off, rest], [], 0.60),
        "six shots blow the seal": ([seated] * 6 + shot * 6 + [off, rest], [], 0.0),
        "pumped in the air, never coupled": ([air, air_press, air] * 3 + [rest], [], 0.0),
        "one stroke spilt, then the full job": ([air, air_press, air] + [seated] * 6 + shot * 3 + [off, rest], [], 0.90),
        "coupler 4.5 mm short, jaws on the head": ([_states(**cocked)] * 6 + [_states(**cocked, lever=17.0),
                                                   _states(**cocked)] * 3 + [off, rest], [], 0.0),
        "coupler cocked 10 deg": ([_states(**at_home, tilt_deg=10.0)] * 8 + [rest], [], 0.0),
        "coupler 2 mm off the axis": ([_states(**at_home, side=0.002)] * 8 + [rest], [], 0.0),
        "half strokes only": ([seated] * 6 + [_states(**at_home, lever=9.0), seated] * 3 + [off, rest], [], 0.40),
        "lever held down counts once": ([seated] * 6 + [press] * 20 + [seated, off, rest], [], 0.60),
        "coupler pulled off mid-stroke": ([seated] * 6 + [_states(**at_home, lever=8.0),
                                          _states(gun_y=HOME_Y - 0.004, gun_z=FIT_Z, lever=17.0, jaws=0.0007),
                                          off, rest], [], 0.30),
        "gun left coupled, let go": ([seated] * 6 + shot * 3, [], 0.70),
        "gun put down beside the mat": ([seated] * 6 + shot * 3 + [off, _states(gun_y=GUN_Y0 + 0.08)], [], 0.85),
        "gun dropped off the bench": ([seated] * 6 + shot * 3 + [off, _states(gun_z=TOP - 0.2), rest], [], 0.0),
    }
    try:
        for name, (seq, robot, want) in cases.items():
            mk.new_episode()
            _TEST_ROBOT = []
            for st in seq:
                _observe(st)
            _TEST_ROBOT = robot
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-6 and got_success == (want >= 1.0), (name, got, want)
    finally:
        mk.new_episode()
        _TEST_ROBOT = None
    # geometry and forces
    head_r = max(r for _, r in HEAD_PROFILE)
    assert BORE / 2 - head_r >= 0.0002  # the head passes the bore
    assert BORE / 2 - JAW_REACH < head_r and BORE / 2 - JAW_REACH >= 0.0024  # closed jaws catch the head, clear the neck
    assert head_r - (BORE / 2 - JAW_REACH) < JAW_TRAVEL  # the jaws can open far enough
    home_gap = TIP - SEAT_DEPTH  # mouth to barrel with the nose on the seat: 4.5 mm
    assert abs(home_gap - HOME_GAP_M) < 1e-9 and HOME_GAP[0] < home_gap < HOME_GAP[1] and home_gap > HEX_L
    assert home_gap + JAW_DEPTH + JAW[1] / 2 < HEAD_PROFILE[0][0]  # the jaws sit behind the shoulder when home
    assert all(p.mass is None or p.mass >= 0.02 for p in kit.parts.values()) and JAW_K <= 600.0
    assert abs(GUN_KG + LEVER_KG + 4 * JAW_KG - 0.61) < 1e-9
    assert SQUEEZE_GAP_OPEN <= -2 * FINGER_OPEN and BARREL_GAP_OPEN <= -2 * FINGER_OPEN
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then every demo plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    print(f"scoring: {self_test()} cases pass")
    hz = 20.0
    for variant in ("full", "air", "short", "five", "left_on"):
        for arm, waypoints in (("right", _right(variant)), ("left", _left(variant))):
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (variant, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (variant, arm)
            print(f"{variant} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
