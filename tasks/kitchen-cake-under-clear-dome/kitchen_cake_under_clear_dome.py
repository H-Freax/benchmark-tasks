"""A slice of layer cake sits on a glass cake plate under a clear cloche (cake dome). The job: lift the dome off by
its knob, set it down on the table, and move the slice onto the dessert plate beside it, upright and undamaged.

Scored from the end state: the dome resting upright on the table clear of the cake plate (0.3), the slice standing
upright on the dessert plate (0.7), both let go and still. A slice that is ever tipped past 30 degrees, let fall or knocked off
onto the table is ruined, and a dome that is ever tipped past 45 degrees has been dropped: either scores 0.

Shapes come from scenekit.py (in this folder), which adds multi-shape parts to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
           -E task=tasks/kitchen-cake-under-clear-dome/kitchen_cake_under_clear_dome.py \\
           -E object_jitter=0 -E control_hz=20 --instruction "<the instruction in task.json>" --scorer task_score
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scenekit as sk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
component, part = sk.component, sk.part

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
# sRGB colours
WOOD = (0.46, 0.25, 0.11)
GLASS_PLATE = (0.86, 0.90, 0.90)
LOWER_CRUMB = (1.00, 0.82, 0.34)
UPPER_CRUMB = (1.00, 0.62, 0.16)
PINK_FILLING = (1.00, 0.50, 0.82)
IVORY_ICING = (1.00, 0.96, 0.68)
GLASS = (0.96, 0.98, 1.00)
WHITE = (0.95, 0.95, 0.92)
GOLD_CARD = (0.90, 0.68, 0.20)

# Cake plate: a 290 mm glass plate on a low wooden foot, with a 6 mm seating ring for the dome.
STAND_XY = (0.270, -0.080)
STAND_DECK_TOP = TOP + 0.012
DOME_ROOT_Z = TOP + 0.018
CAKE_STAND = part(
    "cake_stand", pos=(*STAND_XY, TOP), size=(0.290, 0.290, 0.018), profile="glass_cake_plate", friction=0.9,
    components=(
        component("foot", "cylinder", (0, 0, 0.002), (0.180, 0.180, 0.004), WOOD),
        component("platter", "cylinder", (0, 0, 0.008), (0.290, 0.290, 0.008), GLASS_PLATE),
        component("dome_seat", "ring", (0, 0, 0.015), (0.290, 0.260, 0.006), GLASS_PLATE),
    ),
)

# Slice: a 110 x 78 x 70 mm wedge of layer cake on a gold card, 60 g; tip toward the robot (-x), cut face away.
SLICE_LEN, SLICE_W, SLICE_H = 0.112, 0.082, 0.070
SLICE_XY = STAND_XY
SLICE_ROOT_Z = STAND_DECK_TOP


def _layer(name, cz, h, color, length=0.110, width=0.078):
    return component(name, "wedge", (0.0, 0.0, cz), (length, width, h), color)


CAKE_SLICE = part(
    "cake_slice", pos=(*SLICE_XY, SLICE_ROOT_Z), size=(SLICE_LEN, SLICE_W, SLICE_H), profile="layer_cake_wedge",
    mass=0.060, friction=1.35,
    components=(
        _layer("card", 0.00075, 0.0015, GOLD_CARD, length=SLICE_LEN, width=SLICE_W),
        _layer("lower_crumb", 0.01025, 0.0165, LOWER_CRUMB),
        _layer("filling", 0.02725, 0.0165, PINK_FILLING),
        _layer("upper_crumb", 0.04425, 0.0165, UPPER_CRUMB),
        _layer("icing", 0.06100, 0.0160, IVORY_ICING),
    ),
)

# Dome: a 305 mm clear cloche, 74 mm shell plus a wooden knob (a 72 mm bar, 24 mm thick) on a short stem, 180 g.
# Turned 90 degrees so the knob's bar runs along x.
DOME_D, DOME_H = 0.305, 0.108
KNOB_Z = 0.094  # knob centre above the dome's rim
CLEAR_DOME = part(
    "clear_dome", pos=(*STAND_XY, DOME_ROOT_Z), size=(DOME_D, DOME_D, DOME_H), profile="clear_cloche",
    mass=0.18, friction=2.0, yaw=math.pi / 2,
    components=(
        component("shell", "dome_shell", (0.0, 0.0, 0.037), (DOME_D, DOME_D, 0.074), GLASS),
        component("knob_stem", "cylinder", (0.0, 0.0, 0.080), (0.018, 0.018, 0.012), WOOD),
        component("knob", "capsule", (0.0, 0.0, KNOB_Z), (0.024, 0.072, 0.024), WOOD, axis="Y"),
    ),
)

# Dessert plate: 200 mm white plate with a shallow basin (top 11 mm up) and a raised rim, fixed.
PLATE_XY = (0.285, 0.200)
PLATE_BASIN_TOP = TOP + 0.011
DESSERT_PLATE = part(
    "dessert_plate", pos=(*PLATE_XY, TOP), size=(0.200, 0.200, 0.014), profile="dessert_plate", friction=0.75,
    components=(
        component("plate_foot", "cylinder", (0, 0, 0.003), (0.150, 0.150, 0.006), WHITE),
        component("basin", "cylinder", (0, 0, 0.007), (0.190, 0.190, 0.008), WHITE),
        component("rim", "ring", (0, 0, 0.011), (0.200, 0.180, 0.006), WHITE),
    ),
)

OBJECTS = [CAKE_STAND, CAKE_SLICE, CLEAR_DOME, DESSERT_PLATE]

# -- scoring ---------------------------------------------------------------------------------------------------

SLICE_TIP_DEG = 30.0  # ever tipped past this: the slice has fallen over (ruined)
DOME_TIP_DEG = 45.0  # ever tipped past this: the dome was dropped
DROP_VZ = 0.35  # m/s downward: the slice was let fall (about 6 mm of free fall), not set down
UPRIGHT_DEG = 12.0
DOME_CLEAR_M = 0.30  # dome centre at least this far from the cake plate's centre: its rim is clear of the plate
ON_PLATE_M = 0.055  # slice centre within this of the dessert plate's centre (the wedge stays inside the basin)
STILL_V, STILL_W = 0.01, 0.5  # m/s, rad/s

_EP = {"env": None, "bad": None}
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
TCP_PAST_GRIPPER = 0.134


def tcps() -> list:
    """World positions of both grippers' TCPs (left, right); [] offline."""
    if _TEST_TCPS is not None:
        return [np.asarray(t, float) for t in _TEST_TCPS]
    env = _EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = sk._numpy(base.scene.env_origins)[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = sk._numpy(art.data.body_pos_w)[0] - origin
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * TCP_PAST_GRIPPER)
    return out


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in tcps())


def _still(st) -> bool:
    return float(np.linalg.norm(st.lin_vel)) < STILL_V and float(np.linalg.norm(st.ang_vel)) < STILL_W


def _tilt_deg(st) -> float:
    x, y, z, w = (float(v) for v in st.quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    zz = 1 - 2 * ((x * x + y * y) / (n * n))  # world z of the body's up axis
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def _xy_dist(st, xy) -> float:
    return float(math.hypot(st.pos[0] - xy[0], st.pos[1] - xy[1]))


def _update_bad(objects) -> None:
    cake, dome = objects["cake_slice"], objects["clear_dome"]
    if _EP["bad"]:
        return
    if _tilt_deg(cake) > SLICE_TIP_DEG:
        _EP["bad"] = "slice tipped over"
    elif cake.lin_vel[2] < -DROP_VZ:
        _EP["bad"] = "slice dropped"
    elif cake.pos[2] < TOP + 0.004:
        _EP["bad"] = "slice fell onto the table"
    elif _tilt_deg(dome) > DOME_TIP_DEG:
        _EP["bad"] = "dome dropped"


def dome_set_aside(dome) -> bool:
    return (abs(dome.pos[2] - TOP) < 0.005 and _tilt_deg(dome) < UPRIGHT_DEG
            and _xy_dist(dome, STAND_XY) > DOME_CLEAR_M and _released(dome) and _still(dome))


def slice_served(cake) -> bool:
    return (_xy_dist(cake, PLATE_XY) < ON_PLATE_M and PLATE_BASIN_TOP - 0.003 < cake.pos[2] < PLATE_BASIN_TOP + 0.006
            and _tilt_deg(cake) < UPRIGHT_DEG and _released(cake) and _still(cake))


def score(objects) -> float:
    _update_bad(objects)
    if _EP["bad"]:
        return 0.0
    return 0.3 * dome_set_aside(objects["clear_dome"]) + 0.7 * slice_served(objects["cake_slice"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


@sk.on_reset(__file__, "episode")
def _on_reset(environment) -> None:
    _EP["env"], _EP["bad"] = environment, None


@sk.on_step(__file__, "latch")
def _on_step(environment) -> None:
    _EP["env"] = environment
    names = ("cake_slice", "clear_dome")
    specs = {o.name: o for o in OBJECTS}
    _update_bad({n: sk.part_state(environment, specs[n]) for n in names})


# -- scripted demo (fixed initial layout only) ------------------------------------------------------------------

def demo():
    o, c = FINGER_OPEN, FINGER_CLOSED
    # Right arm: the dome. TCP on the knob's centre line, fingers closing across the 24 mm bar.
    kx, ky = STAND_XY
    k_grip = DOME_ROOT_Z + KNOB_Z + 0.002
    k_up = DOME_ROOT_Z + KNOB_Z + 0.090  # rim 90 mm up: clear of the 70 mm slice
    px, py = 0.265, -0.410  # set-down spot on the table
    p_down = TOP + KNOB_Z + 0.002
    right = [
        Waypoint(2.0, (kx, ky, k_grip + 0.06), o, "over knob"),
        Waypoint(1.5, (kx, ky, k_grip), o, "down to knob"),
        Waypoint(1.0, (kx, ky, k_grip - 0.002), c, "pinch knob"),
        Waypoint(2.5, (kx, ky, k_up), c, "lift dome"),
        Waypoint(3.0, (px, py, k_up), c, "carry aside"),
        Waypoint(2.5, (px, py, p_down - 0.004), c, "set down"),
        Waypoint(1.0, (px, py, p_down - 0.004), o, "let go"),
        Waypoint(1.5, (px, py, p_down + 0.06), o, "up"),
        Waypoint(2.0, (0.15, -0.36, p_down + 0.10), o, "park"),
    ]
    t_right = sum(w.duration for w in right)
    # Left arm: the slice, gripped across the wedge near its centre of mass (53 mm wide there).
    sx, sy = SLICE_XY[0] + 0.018, SLICE_XY[1]
    s_grip = SLICE_ROOT_Z + 0.045
    s_up = SLICE_ROOT_Z + 0.110
    dx, dy = PLATE_XY[0] + 0.018, PLATE_XY[1]
    d_grip = PLATE_BASIN_TOP + 0.002 + 0.045
    left = [
        Waypoint(2.0, (0.20, 0.10, 0.90), o, "ready"),
        Waypoint(t_right - 5.0, (0.20, 0.10, 0.90), o, "wait for the dome"),
        Waypoint(2.0, (sx, sy, s_up), o, "over slice"),
        Waypoint(2.0, (sx, sy, s_grip), o, "down"),
        Waypoint(1.0, (sx, sy, s_grip), c, "grip slice"),
        Waypoint(2.0, (sx, sy, s_up), c, "lift"),
        Waypoint(3.0, (dx, dy, s_up), c, "carry"),
        Waypoint(2.0, (dx, dy, d_grip), c, "lower"),
        Waypoint(1.0, (dx, dy, d_grip), o, "let go"),
        Waypoint(1.5, (dx, dy, d_grip + 0.08), o, "up"),
        Waypoint(1.5, (0.18, 0.25, 0.90), o, "park"),
    ]
    return {"left": left, "right": right}
