"""On the bench is a hydraulic test stand's flat-face quick coupler (ISO 16028 type): the female half is fixed, mouth
up, fed from a small valve block; the male half is plugged into it from above, with a swivel elbow on its side and a
rigid steel line running from the elbow forward and to the right, its far end resting on a stand. Pressure is still
trapped in the line (the gauge reads 150 bar). The valve block carries a pressure-relief lever with two detents:
along the pipe (closed) and a quarter turn across it (relieving). A dust plug for the female stands in a parts tray
and a dust cup for the male's nose stands open side up in a cup holder, next to a rest for the line.

The job is to break the connection and cap both open ends. A flat-face coupler only lets go while its female
sleeve is held pushed back (6 to 8 mm, against its spring); with pressure trapped in the line the male is held in
(150 N on the nose here, beyond the gripper) until the line is relieved. So: relieve the line, hold the sleeve back
with one hand while the other draws the male out, then cap both ends.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/hydraulic-flat-face-coupler-disconnect/hydraulic_flat_face_coupler_disconnect.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1600 \\
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
ZINC = (0.82, 0.74, 0.38)  # yellow-zinc plated steel (coupler bodies)
STEEL = (0.72, 0.73, 0.76)
BLACK = (0.10, 0.10, 0.11)
RED = (0.85, 0.12, 0.10)
LINE = (0.12, 0.12, 0.13)  # black-oxide steel line
CAP_RED = (0.80, 0.15, 0.12)
CAP_BLUE = (0.15, 0.30, 0.75)
PAINT = (0.20, 0.36, 0.62)  # valve block

# -- female half (fixed, mouth up) -------------------------------------------------------------------------------
F = (0.210, 0.000)
BODY_W = 0.024  # female body across flats
BORE = 0.0175  # bore the male nose sits in
F_H = 0.024
MOUTH = TOP + F_H  # the female's mating face
SEAT_DEPTH = 0.010  # male nose inside the female
SLEEVE_OUT, SLEEVE_IN, SLEEVE_H = 0.034, 0.025, 0.014
SLEEVE_PROUD = 0.003  # sleeve top stands this far above the mating face at rest
SLEEVE_TRAVEL = 0.008  # stop
UNLOCK_AT = 0.006  # sleeve pushed back this far: the locking balls are free
RELOCK_BELOW = 0.003
SLEEVE_K = 500.0  # N/m, 2 mm preload: 5 N on the stop (600 N/m and up: the arm no longer holds it on the stop)


def _square_ring(outer, inner, h, z0, color=None, cx=0.0, cy=0.0):
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    return [
        mk.box((outer, w, h), at=(cx, cy + c, z0 + h / 2), color=color),
        mk.box((outer, w, h), at=(cx, cy - c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(cx + c, cy, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(cx - c, cy, z0 + h / 2), color=color),
    ]


kit = mk.Kit()
# female body: square tube round the bore with its flat-face valve (the floor) 14 mm down
# (the kit part is only the body's base, the bonded parent of the male; the bore walls and the face valve are fixed
# scene geometry (FIXTURES below), separate from it, so the male, which is jointed to the kit part, still collides
# with them)
kit.part("female", (F[0], F[1], TOP), [mk.box((BORE, BORE, 0.008), at=(0, 0, 0.004))], mass=None, color=ZINC)
# sleeve: slides down 8 mm against its spring; at rest its top is flush with the mating face
kit.part("sleeve", (F[0], F[1], MOUTH + SLEEVE_PROUD - SLEEVE_H), _square_ring(SLEEVE_OUT, SLEEVE_IN, SLEEVE_H, 0.0), mass=0.060,
         color=STEEL)
kit.slider("sleeve_travel", "female", "sleeve", axis=(0, 0, -1), lower=0.0, upper=SLEEVE_TRAVEL, stiffness=SLEEVE_K,
           rest=-0.002, damping=3.0)

# -- male half with its swivel elbow and steel line------------------------------------------------------------------
NOSE = 0.015
HEX = 0.022
NOSE_Z = MOUTH - SEAT_DEPTH  # the nose's end face, seated
ARM_YAW = -math.radians(40)  # the elbow points forward and to the right (away from the left hand)
DIR = (math.cos(ARM_YAW), math.sin(ARM_YAW))
ARM_L = 0.160  # zinc swivel elbow adapter
TUBE_L = 0.220  # then the steel line it is screwed into, rigid
ARM_Z = 0.017  # elbow / line centre above the nose end
_REACH = ARM_L + TUBE_L


def _along(r, z=0.0):
    return (r * DIR[0], r * DIR[1], z)


_MALE_BOXES = [mk.box((NOSE, NOSE, 0.012), at=(0, 0, 0.006), color=STEEL),
               mk.box((HEX, HEX, 0.010), at=(0, 0, 0.012 + 0.005)),
               mk.box((ARM_L + 0.006, 0.012, 0.010), at=_along(ARM_L / 2 - 0.003, ARM_Z), yaw=ARM_YAW),
               mk.box((TUBE_L, 0.010, 0.010), at=_along(ARM_L + TUBE_L / 2, ARM_Z), yaw=ARM_YAW, color=LINE)]
_vol = [b.size[0] * b.size[1] * b.size[2] for b in _MALE_BOXES]
_com = sum(v * np.asarray(b.at) for v, b in zip(_vol, _MALE_BOXES)) / sum(_vol)
GRIP_R = float(np.hypot(_com[0], _com[1]))  # the demo grips the line over the male's centre of mass
kit.part("male", (F[0], F[1], NOSE_Z), _MALE_BOXES, mass=0.120, color=ZINC)
kit.bond("coupler_lock", "female", "male", anchor=(F[0], F[1], NOSE_Z + 0.006))
LINE_Z = NOSE_Z + ARM_Z
LINE_D = 0.010
LINE_END = (F[0] + (_REACH - 0.020) * DIR[0], F[1] + (_REACH - 0.020) * DIR[1])

# -- relief valve block with a detented lever ---------------------------------------------------------------------
VALVE_XY = (0.230, 0.110)
VALVE = 0.025
VALVE_TOP = TOP + VALVE
LEVER_L = 0.057
RELIEF_DEG = 80.0  # lever this far from closed: the line vents
kit.part("valve_block", (VALVE_XY[0], VALVE_XY[1], TOP), [mk.box((VALVE, VALVE, VALVE), at=(0, 0, VALVE / 2))],
         mass=None, color=PAINT)
kit.part("relief_lever", (VALVE_XY[0], VALVE_XY[1], VALVE_TOP),
         [mk.box((0.006, 0.006, 0.006), at=(0, 0, 0.003)),
          mk.box((0.012, LEVER_L - 0.004, 0.005), at=(0, (LEVER_L - 0.004) / 2, 0.0085), color=RED)],
         mass=0.030, color=STEEL)
kit.hinge("relief_turn", "valve_block", "relief_lever", anchor=(VALVE_XY[0], VALVE_XY[1], VALVE_TOP), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=90.0, stiffness=0.06, damping=0.002)

# Line gauge, 0-250 bar over 270 deg; the needle is a sprung hinge whose rest follows the pressure.
GAUGE_XY = (0.160, 0.070)
GAUGE_Z = TOP + 0.032
P_LINE = 150.0  # bar trapped in the line
DEG_PER_BAR = 270.0 / 250.0
kit.part("gauge_dial", (GAUGE_XY[0], GAUGE_XY[1], TOP),
         [mk.box((0.010, 0.010, GAUGE_Z - TOP - 0.004), at=(0, 0, (GAUGE_Z - TOP - 0.004) / 2), color=STEEL),
          mk.box((0.036, 0.036, 0.004), at=(0, 0, GAUGE_Z - TOP - 0.002), color=(0.95, 0.95, 0.93)),
          mk.box((0.004, 0.003, 0.0012), at=(-0.016, 0, GAUGE_Z - TOP + 0.0006), color=BLACK)],
         mass=None, color=STEEL)
kit.part("gauge_needle", (GAUGE_XY[0], GAUGE_XY[1], GAUGE_Z),
         [mk.box((0.015, 0.003, 0.0015), at=(-0.0065, 0, 0.0015)), mk.box((0.004, 0.004, 0.002), at=(0, 0, 0.001))],
         mass=0.020, color=BLACK)
kit.hinge("gauge", "gauge_dial", "gauge_needle", anchor=(GAUGE_XY[0], GAUGE_XY[1], GAUGE_Z), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=270.0, start_deg=P_LINE * DEG_PER_BAR, stiffness=0.02,
          rest_deg=P_LINE * DEG_PER_BAR, damping=0.0005)

# -- dust caps ----------------------------------------------------------------------------------------------------
# female dust plug: 16 mm stem 10 mm long under a 24 mm head 6 mm tall; frame at the stem's end
PLUG_STEM, PLUG_HEAD, PLUG_HEAD_H = 0.010, 0.022, 0.006
TRAY_XY = (0.160, 0.170)
TRAY = (0.050, 0.050, 0.012)
TRAY_TOP = TOP + TRAY[2]
kit.part("female_cap", (TRAY_XY[0], TRAY_XY[1], TRAY_TOP - PLUG_STEM),
         [mk.box((0.0115, 0.0115, 0.002), at=(0, 0, 0.001)),  # lead-in chamfer at the stem's end
          mk.box((0.0145, 0.0145, PLUG_STEM - 0.002), at=(0, 0, 0.002 + (PLUG_STEM - 0.002) / 2)),
          mk.box((PLUG_HEAD, PLUG_HEAD, PLUG_HEAD_H), at=(0, 0, PLUG_STEM + PLUG_HEAD_H / 2))],
         mass=0.020, color=CAP_RED)
FCAP_SEAT_Z = MOUTH - PLUG_STEM  # stem end with the head on the mating face

# male dust cup: 17 mm square inside, 12 mm deep, 2 mm floor; frame at its bottom
CUP_IN, CUP_OUT, CUP_DEPTH = 0.0172, 0.021, 0.012
CUP_LEAD = 0.0190  # lead-in at the cup mouth (top 3 mm)
HOLDER_XY = (0.180, -0.110)
kit.part("male_cap", (HOLDER_XY[0], HOLDER_XY[1], TOP),
         [mk.box((CUP_OUT, CUP_OUT, 0.002), at=(0, 0, 0.001)), *_square_ring(CUP_OUT, CUP_IN, CUP_DEPTH - 0.003, 0.002),
          *_square_ring(CUP_OUT, CUP_LEAD, 0.003, CUP_DEPTH - 0.001)],
         mass=0.020, color=CAP_BLUE)
MCAP_GAP = 0.0005  # nose end above the cup floor, seated

# The caps' snap rings: welds authored disarmed, engaged by the snap rule.
_FCAP_SEATED = mk._T(None, (F[0], F[1], FCAP_SEAT_Z))
kit.catch("female_cap_snap", "female_cap", _FCAP_SEATED)
_MALE_REF = kit._fk()["male"]
_MCAP_ON_REF = mk._T(None, (F[0], F[1], NOSE_Z - 0.002 - MCAP_GAP))
kit.catch("male_cap_snap", "male_cap", _MCAP_ON_REF, "male", _MALE_REF)


def _tray():
    """Parts tray with a 17 mm hole the dust plug's stem stands in (head on the tray top)."""
    (x, y), (lx, ly, h) = TRAY_XY, TRAY
    g = 0.0085
    side = lx / 2 - g
    return [
        Box("tray_a", pos=(x + g + side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_b", pos=(x - g - side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_c", pos=(x, y + g + (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
        Box("tray_d", pos=(x, y - g - (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
    ]


def _holder():
    """Cup holder: a 23 mm square well 8 mm deep."""
    x, y = HOLDER_XY
    o, i, h = 0.033, 0.023, 0.008
    w, c = (o - i) / 2, (o + i) / 4
    return [
        Box("holder_a", pos=(x, y + c, TOP + h / 2), size=(o, w, h), color=BLACK),
        Box("holder_b", pos=(x, y - c, TOP + h / 2), size=(o, w, h), color=BLACK),
        Box("holder_c", pos=(x + c, y, TOP + h / 2), size=(w, i, h), color=BLACK),
        Box("holder_d", pos=(x - c, y, TOP + h / 2), size=(w, i, h), color=BLACK),
    ]


FIXTURES = [
    *[Box(f"female_wall_{i}", pos=(F[0] + w.at[0], F[1] + w.at[1], TOP + w.at[2]), size=w.size, color=ZINC)
      for i, w in enumerate(_square_ring(BODY_W, BORE, F_H, 0.0))],
    Box("face_valve", pos=(F[0], F[1], (TOP + 0.008 + NOSE_Z - 0.0003) / 2), size=(BORE, BORE, NOSE_Z - 0.0003 - TOP - 0.008),
        color=STEEL),
    # supply line from the valve block to the female's bulkhead foot (low, under the sleeve's travel)
    Box("supply_pipe", pos=(F[0] + 0.010, (VALVE_XY[1] - VALVE / 2 + 0.019) / 2, TOP + 0.005),
        size=(0.010, VALVE_XY[1] - VALVE / 2 - 0.019, 0.010), color=STEEL),
    Box("bulkhead_foot", pos=(F[0] + 0.030, F[1], TOP + 0.003), size=(0.016, 0.040, 0.006), color=PAINT),
    Box("gauge_line", pos=((GAUGE_XY[0] + VALVE_XY[0] - VALVE / 2) / 2, GAUGE_XY[1] + 0.000, TOP + 0.004),
        size=(VALVE_XY[0] - VALVE / 2 - GAUGE_XY[0], 0.006, 0.008), color=STEEL),
    # stand the line's far end rests on
    Box("line_stand", pos=(LINE_END[0], LINE_END[1], (TOP + LINE_Z - LINE_D / 2) / 2),
        size=(0.040, 0.050, LINE_Z - LINE_D / 2 - TOP), color=(0.35, 0.36, 0.38), yaw=ARM_YAW),
    # rest for the line's end beside the cup holder (the male stands capped in the holder, its line on this)
    Box("line_rest", pos=(HOLDER_XY[0] + (_REACH - 0.020) * DIR[0], HOLDER_XY[1] + (_REACH - 0.020) * DIR[1],
                          TOP + (0.002 + MCAP_GAP + ARM_Z - LINE_D / 2) / 2),
        size=(0.040, 0.040, 0.002 + MCAP_GAP + ARM_Z - LINE_D / 2), color=(0.35, 0.36, 0.38), yaw=ARM_YAW),
    *_tray(),
    *_holder(),
]
OBJECTS = [*kit.objects(), *FIXTURES]

DT = 0.05  # one control step at 20 Hz
RELIEF_TAU = 0.4  # s
P_ZERO = 1.0  # bar: relieved


def _relief_rule(objects):
    """Detents: the lever's spring pulls it to closed (0) below 45 deg and to relieving (90) above. Lever at
    80 deg or more: the trapped line pressure vents; the gauge follows it."""
    q = kit.q_deg(objects, "relief_turn")
    det = 90.0 if q >= 45.0 else 0.0
    if mk.MEMORY.get("detent") != det:
        mk.MEMORY["detent"] = det
        kit.drive("relief_turn", det)
    p = mk.MEMORY.get("pressure", P_LINE)
    if q >= RELIEF_DEG:
        p = p * math.exp(-DT / RELIEF_TAU)
        p = 0.0 if p < 0.05 else p
    mk.MEMORY["pressure"] = p
    if p < P_ZERO:
        mk.MEMORY["relieved"] = True
    kit.drive("gauge", p * DEG_PER_BAR)


def _male_seated_offset(objects) -> float:
    nose = kit.point(objects, "male", (0, 0, 0))
    return float(math.dist([float(v) for v in nose], [F[0], F[1], NOSE_Z]))


def _coupler_rule(objects):
    """The locking balls free the male only while the sleeve is pushed back 6 mm and the line is relieved. If the
    sleeve springs back before the nose is 3 mm out, the balls catch it again (the lock is re-engaged)."""
    sleeve = kit.q(objects, "sleeve_travel")
    locked = "coupler_lock" not in kit._released
    if locked and sleeve >= UNLOCK_AT and mk.MEMORY.get("relieved"):
        kit.release("coupler_lock")
        mk.MEMORY["unlocks"] = mk.MEMORY.get("unlocks", 0) + 1
    elif not locked and sleeve < RELOCK_BELOW and _male_seated_offset(objects) < 0.003:
        kit._released.discard("coupler_lock")
        mk._set_joint_enabled(kit, "coupler_lock", True)


def _tilt_between(qa, qb) -> float:
    """Angle (deg) between the z axes of two orientations."""
    za = mk.quat_to_mat(qa)[:, 2]
    zb = mk.quat_to_mat(qb)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(za, zb))))))


def _female_cap_fit(objects):
    """(lateral mm, axial gap mm above seated, axis error deg) of the dust plug in the female's mouth."""
    p, q = kit.pose(objects, "female_cap")
    tip = kit.point(objects, "female_cap", (0, 0, 0))
    lat = math.dist((float(tip[0]), float(tip[1])), F)
    return lat, float(tip[2]) - FCAP_SEAT_Z, _tilt_between(q, (0, 0, 0, 1.0))


def _male_cap_fit(objects):
    """(lateral, axial gap above seated, axis error deg) of the male's nose in the cup, in the cup's frame."""
    t_cap = kit._pose_T(objects, "male_cap")
    nose = kit.point(objects, "male", (0, 0, 0))
    local = mk._inv(t_cap)[:3, :3] @ nose + mk._inv(t_cap)[:3, 3]
    lat = math.hypot(float(local[0]), float(local[1]))
    gap = float(local[2]) - (0.002 + MCAP_GAP)
    _, qm = kit.pose(objects, "male")
    _, qc = kit.pose(objects, "male_cap")
    return lat, gap, _tilt_between(qm, qc)


SNAP_LAT, SNAP_GAP, SNAP_DEG, JAM_DEG, JAM_DEPTH = 0.0015, 0.002, 3.0, 6.0, 0.004


def _cap_rule(objects):
    """A cap pushed on square snaps over its ring (weld); pushed on crooked past the ring it jams (latched)."""
    for name, fit in (("female_cap", _female_cap_fit), ("male_cap", _male_cap_fit)):
        if mk.MEMORY.get(name + "_snapped"):
            continue
        lat, gap, err = fit(objects)
        if lat <= SNAP_LAT and gap <= SNAP_GAP and err <= SNAP_DEG:
            kit.engage(name + "_snap")
            mk.MEMORY[name + "_snapped"] = True
        elif lat <= 0.004 and gap <= JAM_DEPTH and err > JAM_DEG:
            mk.MEMORY["jammed"] = True


def _drop_rule(objects):
    for n in ("male", "female_cap", "male_cap"):
        if float(objects[n].pos[2]) < TOP - 0.03:
            mk.MEMORY["dropped"] = True


kit.on_step(_relief_rule)
kit.on_step(_coupler_rule)
kit.on_step(_cap_rule)
kit.on_step(_drop_rule)


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def male_out(objects) -> bool:
    """The male's nose end 30 mm or more from the centre of the female's mating face."""
    nose = kit.point(objects, "male", (0, 0, 0))
    return math.dist([float(v) for v in nose], [F[0], F[1], MOUTH]) >= 0.030


PARK_NOSE = (HOLDER_XY[0], HOLDER_XY[1], TOP + 0.002 + MCAP_GAP)  # nose end standing in the cup in its holder
PARK_LINE_END = (HOLDER_XY[0] + (_REACH - 0.020) * DIR[0], HOLDER_XY[1] + (_REACH - 0.020) * DIR[1],
                 TOP + 0.002 + MCAP_GAP + ARM_Z)
PARK_TOL, PARK_LINE_TOL, PARK_DEG = 0.008, 0.015, 5.0


def male_parked(objects) -> bool:
    """The male stands upright (axis within 5 deg of vertical) with its nose in the cup holder (within 8 mm) and
    its line's end on the line rest (within 15 mm)."""
    nose = [float(v) for v in kit.point(objects, "male", (0, 0, 0))]
    end = [float(v) for v in kit.point(objects, "male", _along(_REACH - 0.020, ARM_Z))]
    _, q = kit.pose(objects, "male")
    return (math.dist(nose, PARK_NOSE) <= PARK_TOL and math.dist(end, PARK_LINE_END) <= PARK_LINE_TOL
            and _tilt_between(q, (0, 0, 0, 1.0)) <= PARK_DEG)


def female_capped(objects) -> bool:
    if not mk.MEMORY.get("female_cap_snapped"):
        return False
    lat, gap, err = _female_cap_fit(objects)
    return lat <= 0.002 and abs(gap) <= 0.002 and err < 3.0


def male_capped(objects) -> bool:
    if not mk.MEMORY.get("male_cap_snapped"):
        return False
    lat, gap, err = _male_cap_fit(objects)
    return lat <= 0.002 and abs(gap) <= 0.002 and err < 3.0


def settled(objects) -> bool:
    """Male and both caps let go of (no gripper within 2 cm) and still (under 1 cm/s)."""
    return all(_released(objects[n]) and objects[n].resting(max_speed=0.01, max_spin=float("inf"))
               for n in ("male", "female_cap", "male_cap"))


def failed(objects) -> bool:
    """Latched: the male fell off the bench, or a cap was jammed on crooked."""
    return bool(mk.MEMORY.get("dropped") or mk.MEMORY.get("jammed"))


def score(objects) -> float:
    """0 if failed(). Otherwise the sum of, with the male out 30 mm and parked upright on its holder and rest: line
    relieved 0.15, male out 0.35, everything let go and still 0.1; and female capped 0.2; male capped 0.2."""
    if failed(objects):
        return 0.0
    s = 0.0
    if male_out(objects) and male_parked(objects):
        s += 0.50 if mk.MEMORY.get("relieved") else 0.35
        if settled(objects):
            s += 0.10
    if female_capped(objects):
        s += 0.20
    if male_capped(objects):
        s += 0.20
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    nose = kit.point(objects, "male", (0, 0, 0))
    plug = kit.point(objects, "female_cap", (0, 0, 0))
    d = max(0.0, 0.030 - math.dist([float(v) for v in nose], [F[0], F[1], MOUTH]))
    return float(d + math.dist([float(v) for v in plug], [F[0], F[1], FCAP_SEAT_Z]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: sweeps the relief lever over into its relieving detent (the gauge falls to zero), then waits over the
# coupler. Right arm: grips the steel line over the male's centre of mass (so the male hangs level in the pinch). Left
# arm: comes down with its fingers either side of the male's body onto the sleeve and holds it on its stop; the right
# arm draws the male up out of the female, carries it to the cup holder and pushes the nose into the dust cup, lets
# go (the line on its rest). Left arm: lets the sleeve up, takes the dust plug from the tray and pushes it into the
# female's mouth, lets go.

TIP = 0.013
TRAVEL = TOP + 0.130
PARK_R = (0.20, -0.20, TRAVEL)
STRADDLE = -0.0135  # pads' inner faces 12.5 mm off the axis: clear of the 22 mm body, on the sleeve's top
PRESS_X = F[0] - 0.012
PRESS_OVER = 0.008  # commanded this far below the sleeve stop: the arm's position error is the push force
PRESS_Z = MOUTH + SLEEVE_PROUD - SLEEVE_TRAVEL - PRESS_OVER + TIP  # onto the sleeve's stop
LEVER_Z = VALVE_TOP + 0.0085 + TIP - 0.004
PULL = 0.044  # the male drawn up this far


def _valve_sweep(start=-5, stop=115, step=15):
    (vx, vy), r, off = VALVE_XY, 0.045, 0.014

    def at(deg):
        a = math.radians(deg)
        return (vx + r * math.sin(a) - off * math.cos(a), vy + r * math.cos(a) + off * math.sin(a))

    plan = [Waypoint(2.0, (*at(start), TRAVEL), FINGER_CLOSED, "over the relief lever", 0.0),
            Waypoint(1.5, (*at(start), LEVER_Z), FINGER_CLOSED, "down beside the lever", 0.0)]
    plan += [Waypoint(1.0, (*at(d), LEVER_Z), FINGER_CLOSED, f"sweep the lever to {d} deg", 0.0)
             for d in range(start + step, stop + 1, step)]
    return plan + [Waypoint(1.0, (*at(stop - 10), TRAVEL), FINGER_CLOSED, "up off the lever", 0.0)]


def _sleeve_hold(hold_s, wait_s=0.0):
    px, py = F
    return [
        Waypoint(2.0, (PRESS_X, py, TRAVEL), STRADDLE, "over the coupler", 0.0),
        Waypoint(max(wait_s, 0.1), (PRESS_X, py, TRAVEL), STRADDLE, "wait for the other hand", 0.0),
        Waypoint(1.5, (PRESS_X, py, MOUTH + TIP + 0.010), STRADDLE, "down either side of the male", 0.0),
        Waypoint(1.0, (PRESS_X, py, PRESS_Z), STRADDLE, "push the sleeve back", 0.0),
        Waypoint(hold_s, (PRESS_X, py, PRESS_Z), STRADDLE, "hold it back while the male comes out", 0.0),
        Waypoint(0.5, (PRESS_X, py, PRESS_Z + 0.004), -0.020, "let the sleeve up, open clear of the male", 0.0),
        Waypoint(1.0, (PRESS_X - 0.045, py, PRESS_Z + 0.004), -0.020, "back out from round the male", 0.0),
        Waypoint(1.0, (PRESS_X - 0.045, py, TRAVEL), -0.020, "up", 0.0),
    ]


def _plug_female():
    tx, ty = TRAY_XY
    grip = PLUG_STEM + 0.0015 + TIP  # pad tips 1.5 mm up the head
    in_tray = TRAY_TOP - PLUG_STEM
    return [
        Waypoint(2.0, (tx, ty, TRAVEL), -0.017, "over the dust plug", 0.0),
        Waypoint(1.5, (tx, ty, in_tray + grip), -0.017, "down round its head", 0.0),
        Waypoint(0.5, (tx, ty, in_tray + grip), -0.017, "settle", 0.0),
        Waypoint(1.0, (tx, ty, in_tray + grip), FINGER_CLOSED, "grip it", 0.0),
        Waypoint(1.5, (tx, ty, TRAVEL), FINGER_CLOSED, "lift", 0.0),
        Waypoint(2.0, (F[0], F[1], TRAVEL), FINGER_CLOSED, "over the female", 0.0),
        Waypoint(1.5, (F[0], F[1], FCAP_SEAT_Z + 0.012 + grip), FINGER_CLOSED, "stem over the mouth", 0.0),
        Waypoint(0.5, (F[0], F[1], FCAP_SEAT_Z + 0.012 + grip), FINGER_CLOSED, "settle", 0.0),
        Waypoint(1.5, (F[0], F[1], FCAP_SEAT_Z - 0.0005 + grip), FINGER_CLOSED, "push it home", 0.0),
        Waypoint(0.6, (F[0], F[1], FCAP_SEAT_Z - 0.0005 + grip), FINGER_CLOSED, "hold", 0.0),
        Waypoint(0.8, (F[0], F[1], FCAP_SEAT_Z - 0.0005 + grip), -0.017, "let go", 0.0),
        Waypoint(1.0, (F[0], F[1], TRAVEL), -0.017, "up", 0.0),
        Waypoint(1.5, (0.20, 0.20, TRAVEL), -0.017, "clear", 0.0),
    ]


def _dur(plan):
    return sum(w.duration for w in plan)


GRIP_DZ = ARM_Z - 0.004 + TIP  # TCP above the nose end with the elbow arm gripped
PRE_PULL = 7.0  # right arm: from its start to gripping the elbow


# Measured in sim, not tuned by eye: drawing the nose out of the bore turns the male about 2.2 deg (counter-clockwise
# seen from above) in the two-pad pinch on the 10 mm line (pad friction torque is small next to the drag on the nose),
# so the carry turns the wrist back by that much. What is left, CUP_AIM, is the right arm's tracking lag as it comes
# down with the 120 g male held 15 cm off the wrist: the nose lands about 4 mm short in x and 1.5 mm in y.
SLIP_DEG = 2.2
CUP_AIM = (0.0040, 0.0015)


def _right_pull():
    gx, gy = F[0] + GRIP_R * DIR[0], F[1] + GRIP_R * DIR[1]
    z0 = NOSE_Z + GRIP_DZ
    hx, hy = HOLDER_XY[0] + GRIP_R * DIR[0] + CUP_AIM[0], HOLDER_XY[1] + GRIP_R * DIR[1] + CUP_AIM[1]
    cup_floor = TOP + 0.002
    Y = ARM_YAW
    Yc = ARM_YAW - math.radians(SLIP_DEG)  # turn the wrist back by the yaw the male slipped in the pinch
    return [
        Waypoint(2.0, (gx, gy, z0 + 0.045), -0.014, "above the elbow", Y),
        Waypoint(1.5, (gx, gy, z0 + 0.025), -0.014, "over the elbow arm", Y),
        Waypoint(1.5, (gx, gy, z0), -0.014, "down round it", Y),
        Waypoint(1.0, (gx, gy, z0), -0.014, "settle", Y),
        Waypoint(1.0, (gx, gy, z0), FINGER_CLOSED, "grip it", Y),
    ], [
        Waypoint(2.5, (gx, gy, z0 + PULL), FINGER_CLOSED, "draw the male straight out", Y),
        Waypoint(2.5, (gx, gy, z0 + PULL), FINGER_CLOSED, "hold while the other hand backs out", Y),
        Waypoint(2.5, (hx, hy, z0 + PULL), FINGER_CLOSED, "over the dust cup", Yc),
        Waypoint(1.5, (hx, hy, cup_floor + 0.014 + GRIP_DZ), FINGER_CLOSED, "nose over the cup", Yc),
        Waypoint(0.5, (hx, hy, cup_floor + 0.014 + GRIP_DZ), FINGER_CLOSED, "settle", Yc),
        Waypoint(1.5, (hx, hy, cup_floor + MCAP_GAP - 0.001 + GRIP_DZ), FINGER_CLOSED, "push the nose into the cup",
                 Yc),
        Waypoint(0.6, (hx, hy, cup_floor + MCAP_GAP - 0.001 + GRIP_DZ), FINGER_CLOSED, "hold", Yc),
        Waypoint(1.0, (hx, hy, cup_floor + MCAP_GAP - 0.001 + GRIP_DZ), -0.014, "let go", Yc),
        Waypoint(1.5, (hx, hy - 0.040, TRAVEL), -0.014, "up and away", Yc),
        Waypoint(1.5, PARK_R, -0.014, "clear", Y),
    ]


def _plans(relieve=True):
    sweep = _valve_sweep() if relieve else [Waypoint(2.0, (0.20, 0.20, TRAVEL), FINGER_CLOSED, "wait", 0.0)]
    grip, pull = _right_pull()
    t_grip = _dur(sweep) + 0.5 + _dur(grip)  # right hand on the elbow, after the left has left the valve
    hover = 8.0
    t_press = _dur(sweep) + _dur(_sleeve_hold(0, hover)[:4]) + 0.5  # the sleeve held back from here
    hold = 2.5 + 0.3  # until the male is drawn out
    left = sweep + _sleeve_hold(hold + 0.5, hover) + _plug_female()
    right = [Waypoint(_dur(sweep) + 0.5, PARK_R, -0.014, "wait", 0.0), *grip,
             Waypoint(max(0.5, t_press - t_grip), grip[-1].pos, FINGER_CLOSED, "wait for the sleeve", ARM_YAW), *pull]
    return {"left": left, "right": right}


def demo():
    # DEMO_VARIANT=no_relief runs the degenerate check (same moves without relieving the line)
    import os
    v = os.environ.get("DEMO_VARIANT")
    return demo_no_relief() if v == "no_relief" else demo_relieve_only() if v == "relieve_only" else _plans()


def demo_relieve_only():
    """Degenerate: relieves the line, then both arms park."""
    return {"left": _valve_sweep(), "right": [Waypoint(2.0, PARK_R, -0.014, "wait", 0.0)]}


def demo_no_relief():
    """Degenerate: skips the relief lever, otherwise the same moves."""
    return _plans(relieve=False)


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    start = kit.fake_states()
    male_p, male_q = kit.pose(start, "male")
    up = np.array([0, 0, PULL])
    out = {"male": (male_p + up, male_q)}
    hold_p = np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + 0.002 + MCAP_GAP])
    in_cup = {"male": (hold_p, male_q), "male_cap": (np.array([HOLDER_XY[0], HOLDER_XY[1], TOP]), male_q)}
    fseat = (np.array([F[0], F[1], FCAP_SEAT_Z]), np.array([0, 0, 0, 1.0]))
    tilt5 = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(5)))
    tilt20 = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(20)))

    def st(**moved):
        return kit.fake_states(moved=moved)

    full = st(**in_cup, female_cap=fseat)
    R = {"relieved": True}
    SN = {"relieved": True, "female_cap_snapped": True, "male_cap_snapped": True}
    cases = {  # name: (state, tcps, MEMORY, score)
        "start": (start, [], {}, 0.0),
        "relieved only": (start, [], R, 0.0),
        "relieved, male out in the air, held": (st(**out), [kit.point(start, "male", (GRIP_R * DIR[0], GRIP_R * DIR[1], ARM_Z + PULL))], R, 0.0),
        "relieved, male out in the air, let go": (st(**out), [], R, 0.0),
        "relieved, male parked uncapped, let go": (st(male=(hold_p, male_q)), [], R, 0.6),
        "relieved, male parked, held": (st(male=(hold_p, male_q)), [kit.point(st(male=(hold_p, male_q)), "male", (0, 0, 0.03))], R, 0.5),
        "male tipped 20 deg on the holder": (st(male=(hold_p, tilt20), female_cap=fseat), [], SN, 0.2),
        "male out 25 mm only": (st(male=(male_p + np.array([0, 0, 0.037]), male_q)), [], R, 0.0),
        "everything done": (full, [], SN, 1.0),
        "female cap not snapped": (full, [], {**SN, "female_cap_snapped": False}, 0.8),
        "male not capped": (st(male=(hold_p, male_q), female_cap=fseat), [], {**SN, "male_cap_snapped": False}, 0.8),
        "female cap tilted 5 deg after snap": (st(**in_cup, female_cap=(fseat[0], tilt5)), [], SN, 0.8),
        "male still held at the end": (full, [kit.point(full, "male", (0, 0, 0.03))], SN, 0.9),
        "male dropped off the bench": (full, [], {**SN, "dropped": True}, 0.0),
        "cap jammed crooked": (full, [], {**SN, "jammed": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        # snap geometry on the hand-built states
        assert _female_cap_fit(full)[0] < 1e-6 and abs(_female_cap_fit(full)[1]) < 1e-6
        lat, gap, err = _male_cap_fit(full)
        assert lat < 1e-6 and abs(gap) < 1e-6 and err < 1e-6, (lat, gap, err)
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
