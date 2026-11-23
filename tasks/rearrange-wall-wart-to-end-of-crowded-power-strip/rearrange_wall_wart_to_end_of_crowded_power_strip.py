"""On the station table: an unwired 4-outlet NEMA 5-15R power strip (Woods / GE 222 x 54 x 30 mm ivory housing, 360 g
free-standing on a charcoal desk mat at x = 0.245, y = -0.020) has four grounded 3-slot outlets along its top face at
34.0 mm pitch (Outlet 1 at y_rel = +42 mm, Outlet 2 at +8 mm, Outlet 3 at -26 mm, and end Outlet 4 at -60 mm; strip ends
at y_rel = +124 mm and -98 mm). Outlets 1 and 2 hold two seated appliance plugs (blue monitor plug and grey lamp plug)
that must remain plugged in. Outlet 3 holds a bulky white AC wall-wart power brick (TP-Link 12V-style, 36 mm X x 54 mm Y
x 40 mm Z, 105 g) whose off-center prongs sit 10 mm from its +Y face so its 44 mm -Y body overhang completely covers
end Outlet 4.

To the robot's right at (x = 0.245, y = -0.185) on an olive-slate mat, a black appliance cord's molded 3-prong NEMA 5-15P
plug (36 mm X x 24 mm Y x 38 mm Z, 64 g) rests prongs-down on a 18.5 mm teal desktop cord cradle so its 16.0 mm ground
pin hangs 2.5 mm clear of the mat.

The task: plug the black cord on the right into the power strip, leaving everything already on the strip plugged in.
Because the bulky white wall-wart in Outlet 3 covers Outlet 4, the robot must steady the free-standing power strip while
unplugging the wall-wart from Outlet 3, reseat the wall-wart into end Outlet 4 (uncovering Outlet 3), and seat the black
cord's 3-prong plug flush into Outlet 3 without unseating Outlets 1 or 2 or dragging/tipping the strip, then let go.

Requires -E object_jitter=0: the power strip, pre-seated plugs, bulky wall-wart, and cord holder match the desk layout.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402
import gripkit  # noqa: E402
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import (  # noqa: E402
    Joint,
    Mech,
    MechState,
    box,
    hull,
    lathe,
    on_reset,
    on_step,
    released,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

gripkit.use_rubber_pads()

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SCORE_OVER = "end"
SUCCESS_HOLD_S = 6.0
CONTROL_DT = 0.05  # 20 Hz
DWELL_STEPS = 20   # 1.0 s continuous stillness & release hold

# High-contrast realistic hardware palette
CHARCOAL = (0.16, 0.18, 0.21)
OLIVE_SLATE = (0.26, 0.33, 0.29)
STRIP_IVORY = (0.88, 0.86, 0.80)
SLOT_DARK = (0.16, 0.17, 0.19)
SWITCH_RED = (0.84, 0.20, 0.18)
PLUG_BLUE = (0.20, 0.44, 0.76)
PLUG_GREY = (0.44, 0.47, 0.52)
WART_WHITE = (0.93, 0.94, 0.96)
WART_LABEL = (0.58, 0.62, 0.68)
PLUG_BLACK = (0.17, 0.18, 0.20)
BRASS = (0.86, 0.71, 0.24)
HOLDER_TEAL = (0.20, 0.54, 0.56)

# ---------------------------------------------------------------------------------------------------------------------
# Desk layout & 1:1 NEMA 5-15R / NEMA 5-15P / Wall-Wart dimensions (station frame, m)
MAT_H = 0.004
MAT_TOP = TOP + MAT_H

STRIP_XY = (0.245, -0.020)
STRIP_MAT_SIZE = (0.160, 0.280, MAT_H)

CORD_MAT_XY = (0.245, -0.185)
CORD_MAT_SIZE = (0.140, 0.090, MAT_H)

STRIP_H = 0.030
STRIP_W_X = 0.050
STRIP_Y_MIN = -0.098
STRIP_Y_MAX = +0.124  # Total length = 222.0 mm (4-outlet strip with generous +Y switch deck & -Y end deck so wall-wart sits rock-solid!)
STRIP_MASS_KG = 0.360

# Four NEMA 5-15R outlet centers along y_rel of POWER_STRIP (34.0 mm standard pitch):
OUTLET_Y = (
    +0.042,  # Outlet 1 (Monitor plug — stays seated)
    +0.008,  # Outlet 2 (Lamp plug — stays seated)
    -0.026,  # Outlet 3 (Wall-wart starts here, blocking Outlet 4; Black plug ends here!)
    -0.060,  # Outlet 4 (End outlet — Wall-wart moves here so its -44 mm overhang sits stably on the -98 mm strip end!)
)

# NEMA 5-15P 3-prong offsets relative to each outlet center (polarized blades at x = -4.5 mm, y = +/-6.35 mm;
# longer round ground pin at x = +6.5 mm, y = 0.0 mm):
BLADE_X = -0.0045
BLADE_DY = 0.00635
GROUND_X = +0.0065
SEAT_CLEARANCE_Z = 0.0003  # Seated plug underside sits at z_strip = +30.0 mm!

# Weighted silicone desktop cord holder supporting the black plug prongs-down symmetrically at (0.245, -0.150)
NEW_PLUG_STOW_XY = (0.245, -0.185)
HOLDER_XY = (0.245, -0.185)
NEW_PLUG_SPAWN_Z = MAT_TOP + 0.0185  # Symmetric side shoulders rest on 18.5 mm cradle rails; 16 mm ground pin hangs 2.5 mm clear!


# ---------------------------------------------------------------------------------------------------------------------
# Mats & Weighted Silicone Desktop Cable Holder
STRIP_MAT = Mech(
    "strip_mat",
    pos=(STRIP_XY[0], STRIP_XY[1], TOP),
    mass=None,
    friction=1.0,
    shapes=(
        box((0.0, 0.0, MAT_H / 2.0), STRIP_MAT_SIZE, CHARCOAL),
        # Low 4.5 mm molded silicone desk-mat corner cleats prevent lateral skating while still letting strip lift/tip if unbraced!
        box((-0.0305, -0.075, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((+0.0305, -0.075, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((-0.0305, +0.095, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((+0.0305, +0.095, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((0.0, -0.1015, MAT_H + 0.00225), (0.050, 0.005, 0.0045), CHARCOAL),
        box((0.0, +0.1395, MAT_H + 0.00225), (0.050, 0.005, 0.0045), CHARCOAL),
    ),
)

CORD_MAT = Mech(
    "cord_mat",
    pos=(CORD_MAT_XY[0], CORD_MAT_XY[1], TOP),
    mass=None,
    friction=1.0,
    shapes=(box((0.0, 0.0, MAT_H / 2.0), CORD_MAT_SIZE, OLIVE_SLATE),),
)

CORD_HOLDER = Mech(
    "cord_holder",
    pos=(HOLDER_XY[0], HOLDER_XY[1], MAT_TOP),
    mass=None,
    friction=0.30,
    shapes=(
        # Low base floor (z = 0..1.5 mm) + +/-X support ledges (-X at -14.5 mm, +X at +15.0 mm -> >4 mm clear of all 3 prongs!)
        box((0.0, 0.0, 0.00075), (0.046, 0.044, 0.0015), HOLDER_TEAL),
        box((-0.0145, 0.0, 0.00925), (0.005, 0.026, 0.0185), HOLDER_TEAL),
        box((+0.0150, 0.0, 0.00925), (0.005, 0.026, 0.0185), HOLDER_TEAL),
    ),
)


# ---------------------------------------------------------------------------------------------------------------------
# Free-Standing 4-Outlet Power Strip with Real 3-Cavity NEMA 5-15R Sockets
def _power_strip_shapes() -> tuple:
    """184 x 54 x 30 mm 4-outlet power strip built from exact non-overlapping walls around 4 x 3-slot NEMA 5-15R cavities.
    Each outlet at yc in OUTLET_Y has:
    - Two blade cavities: x in [-0.0068, -0.0022] (4.6 mm), y in [yc +/- 0.00635 - 0.0042 .. + 0.0042] (8.4 mm), z = 11..30 mm
    - One ground cavity:  x in [+0.0033, +0.0097] (6.4 mm), y in [yc - 0.0032 .. yc + 0.0032] (6.4 mm), z = 11..30 mm"""
    y_mid = 0.5 * (STRIP_Y_MIN + STRIP_Y_MAX)
    y_len = STRIP_Y_MAX - STRIP_Y_MIN
    parts = [
        # Solid lower chassis floor (z = 0..8 mm, full 54 x 222 mm footprint -> 22 mm deep cavities so 16 mm pins never bottom out!)
        box((0.0, y_mid, 0.0040), (0.054, y_len, 0.0080), STRIP_IVORY),
        # Outer -X rail (-0.027..-0.0105 m -> 5.1 mm clear of -0.0054 blade!), central divider (+0.0005 width 0.0018 -> 3.8 mm clear of both blade & ground!), +X rail (+0.0135..+0.027 m -> 4.8 mm clear of +0.0087 ground!)
        box((-0.01875, y_mid, 0.0190), (0.0165, y_len, 0.0220), STRIP_IVORY),
        box((+0.00050, y_mid, 0.0190), (0.0018, y_len, 0.0220), STRIP_IVORY),
        box((+0.02025, y_mid, 0.0190), (0.0135, y_len, 0.0220), STRIP_IVORY),
        # Left (+Y = +0.064..+0.124 m) switch deck + illuminated red rocker bezel + +Y cord strain-relief boot
        box((0.0, +0.094, 0.0190), (0.024, 0.060, 0.0220), STRIP_IVORY),
        dataclasses.replace(box((0.0, +0.084, 0.0306), (0.012, 0.020, 0.0012), SWITCH_RED), collide=False),
        box((0.0, +0.130, 0.012), (0.014, 0.012, 0.012), PLUG_GREY),
        # Right (-Y = -0.098..-0.074 m) solid end deck supporting seated wall-wart
        box((0.0, -0.086, 0.0190), (0.024, 0.024, 0.0220), STRIP_IVORY),
    ]

    # >= 4.2 mm per-side PhysX clearance around every blade & ground pin while blocking reversed/shifted plugs!
    blade_cuts = []
    ground_cuts = []
    for yc in OUTLET_Y:
        blade_cuts.append((yc - BLADE_DY - 0.0068, yc + BLADE_DY + 0.0068))
        ground_cuts.append((yc - 0.0062, yc + 0.0062))
        parts.append(dataclasses.replace(box((BLADE_X, yc - BLADE_DY, 0.0301), (0.0024, 0.0072, 0.0002), SLOT_DARK), collide=False))
        parts.append(dataclasses.replace(box((BLADE_X, yc + BLADE_DY, 0.0301), (0.0024, 0.0072, 0.0002), SLOT_DARK), collide=False))
        parts.append(dataclasses.replace(box((GROUND_X, yc, 0.0301), (0.0050, 0.0050, 0.0002), SLOT_DARK), collide=False))

    def _lane_bridges(xc: float, wx: float, cuts: list[tuple[float, float]]) -> None:
        cuts_s = sorted(cuts)
        cur = -0.074
        for a, b in cuts_s:
            if a - cur > 0.0008:
                parts.append(box((xc, 0.5 * (cur + a), 0.0190), (wx, a - cur, 0.0220), STRIP_IVORY))
            cur = max(cur, b)
        if +0.064 - cur > 0.0008:
            parts.append(box((xc, 0.5 * (cur + 0.064), 0.0190), (wx, +0.064 - cur, 0.0220), STRIP_IVORY))

    _lane_bridges(-0.0054, 0.0096, blade_cuts)
    _lane_bridges(+0.0074, 0.0116, ground_cuts)
    return tuple(parts)


POWER_STRIP = Mech(
    "power_strip",
    pos=(STRIP_XY[0], STRIP_XY[1], MAT_TOP + 0.0002),
    mass=STRIP_MASS_KG,
    friction=0.95,
    shapes=_power_strip_shapes(),
)


# ---------------------------------------------------------------------------------------------------------------------
# Chamfered 3-Prong NEMA 5-15P Prong Builder + Seated Plugs + Bulky Wall-Wart + Loose Black Cord Plug
def _nema_5_15p_prongs() -> tuple:
    """Two tapered flat brass blades (14.5 mm long at x = -4.5 mm, y = +/-6.35 mm) + longer conical-tip round ground pin
    (16.0 mm long at x = +6.5 mm, y = 0.0 mm) so the ground pin enters first and self-centers the flat blades!"""
    blades = []
    for sy in (-BLADE_DY, +BLADE_DY):
        pts = [
            # Root at z = 0.0 (1.8 mm X x 6.0 mm Y)
            (BLADE_X - 0.0009, sy - 0.0030, 0.0),
            (BLADE_X + 0.0009, sy - 0.0030, 0.0),
            (BLADE_X - 0.0009, sy + 0.0030, 0.0),
            (BLADE_X + 0.0009, sy + 0.0030, 0.0),
            # Shoulder at z = -0.0120 m
            (BLADE_X - 0.0009, sy - 0.0030, -0.0120),
            (BLADE_X + 0.0009, sy - 0.0030, -0.0120),
            (BLADE_X - 0.0009, sy + 0.0030, -0.0120),
            (BLADE_X + 0.0009, sy + 0.0030, -0.0120),
            # Chamfered lead-in tip at z = -0.0145 m (0.8 mm X x 3.6 mm Y)
            (BLADE_X - 0.0004, sy - 0.0018, -0.0145),
            (BLADE_X + 0.0004, sy - 0.0018, -0.0145),
            (BLADE_X - 0.0004, sy + 0.0018, -0.0145),
            (BLADE_X + 0.0004, sy + 0.0018, -0.0145),
        ]
        blades.append(hull((0.0, 0.0, 0.0), pts, BRASS))
    # Ground pin from z = -0.0160 m conical tip up to z = 0.0 m (Ø 4.4 mm shaft, Ø 1.8 mm lead-in tip)
    gnd = lathe((GROUND_X, 0.0, -0.0160), [(0.0009, 0.0), (0.0022, 0.0025), (0.0022, 0.0160)], BRASS)
    return (*blades, gnd)


def _std_plug_shapes(color: tuple[float, float, float], cord_dir_x: float = +1.0) -> tuple:
    return (
        *_nema_5_15p_prongs(),
        box((0.0, 0.0, 0.014), (0.030, 0.025, 0.028), color),
        box((0.022 * cord_dir_x, 0.0, 0.014), (0.016, 0.010, 0.010), color),
    )


PLUG_1 = Mech(
    "plug_1",
    pos=(STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[0], MAT_TOP + STRIP_H + SEAT_CLEARANCE_Z),
    mass=0.048,
    friction=0.90,
    shapes=_std_plug_shapes(PLUG_BLUE, cord_dir_x=+1.0),
)

PLUG_2 = Mech(
    "plug_2",
    pos=(STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[1], MAT_TOP + STRIP_H + SEAT_CLEARANCE_Z),
    mass=0.048,
    friction=0.90,
    shapes=_std_plug_shapes(PLUG_GREY, cord_dir_x=+1.0),
)


def _wall_wart_shapes() -> tuple:
    """TP-Link 12V-style bulky AC wall-wart brick (36 mm X x 54 mm Y x 40 mm Z, 105 g):
    Prong origin (0, 0, 0) sits 10 mm inside the +Y face, so y_wart spans [-0.044, +0.010 m]!
    When seated in Outlet 3 (y_rel = -0.026 m), the brick spans y_rel = [-0.070, -0.016 m], completely covering
    Outlet 4 (y_rel = -0.060 m)! When moved to end Outlet 4 (y_rel = -0.060 m), the brick spans y_rel = [-0.104, -0.050 m]
    with CoM at y_rel = -0.077 m (21 mm inside the -0.098 m strip end) and 24 mm open clearance to Outlet 3!"""
    return (
        *_nema_5_15p_prongs(),
        box((0.0, -0.017, 0.020), (0.036, 0.054, 0.040), WART_WHITE),
        # Upper molded finger-grip bezel lip prevents vertical/rotational slip during prong extraction!
        box((0.0, -0.010, 0.0385), (0.042, 0.036, 0.003), WART_WHITE),
        dataclasses.replace(box((0.0, -0.017, 0.0403), (0.026, 0.038, 0.0005), WART_LABEL), collide=False),
        box((+0.023, -0.036, 0.019), (0.010, 0.008, 0.008), PLUG_BLACK),
    )


WALL_WART = Mech(
    "wall_wart",
    pos=(STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[2], MAT_TOP + STRIP_H + SEAT_CLEARANCE_Z),
    mass=0.105,
    friction=1.35,
    shapes=_wall_wart_shapes(),
)

NEW_PLUG = Mech(
    "new_plug",
    pos=(NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], NEW_PLUG_SPAWN_Z),
    mass=0.064,
    friction=1.35,
    shapes=(
        *_nema_5_15p_prongs(),
        box((0.0, 0.0, 0.019), (0.036, 0.024, 0.038), PLUG_BLACK),
        # Upper molded pull-cap lip above +/-18 mm X body so vertical lift never slips!
        box((0.0, 0.0, 0.0365), (0.042, 0.024, 0.003), PLUG_BLACK),
        dataclasses.replace(box((0.0, 0.0, 0.040), (0.009, 0.009, 0.006), PLUG_BLACK), collide=False),
    ),
)

OBJECTS = [STRIP_MAT, CORD_MAT, CORD_HOLDER, POWER_STRIP, PLUG_1, PLUG_2, WALL_WART, NEW_PLUG]


# ---------------------------------------------------------------------------------------------------------------------
# Live Electrical/Geometric Inspection & End-Step Scorer (zero permanent positive latches)
_STATE: dict[str, Any] = {
    "existing_unplugged": False,
    "strip_tipped": False,
    "last_poses": None,
    "still_steps": 0,
}


def reset_episode_state() -> None:
    _STATE["existing_unplugged"] = False
    _STATE["strip_tipped"] = False
    _STATE["last_poses"] = None
    _STATE["still_steps"] = 0


@on_reset
def _on_sim_reset(env) -> None:
    reset_episode_state()


def _quat_to_rot(quat_xyzw: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _tilt_deg(quat_xyzw: tuple[float, float, float, float]) -> float:
    rot = _quat_to_rot(quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def _rel_yaw_deg(plug_st: MechState, strip_st: MechState) -> float:
    rp = _quat_to_rot(plug_st.quat_xyzw)
    rs = _quat_to_rot(strip_st.quat_xyzw)
    rel = rs.T @ rp
    return math.degrees(math.atan2(float(rel[1, 0]), float(rel[0, 0])))


def _plug_in_outlet(plug_st: MechState, strip_st: MechState, outlet_idx: int, face_gap_max: float = 0.0050) -> bool:
    rs = _quat_to_rot(strip_st.quat_xyzw)
    rel_p = (np.asarray(plug_st.origin, dtype=float) - np.asarray(strip_st.origin, dtype=float)) @ rs
    dx = abs(float(rel_p[0]))
    dy = abs(float(rel_p[1]) - OUTLET_Y[outlet_idx])
    dz_face = float(rel_p[2]) - STRIP_H  # 0.0 when flush on +30.0 mm strip top face!
    rel_tilt = math.degrees(
        math.acos(max(-1.0, min(1.0, float(np.dot(_quat_to_rot(plug_st.quat_xyzw)[:, 2], rs[:, 2])))))
    )
    rel_yaw = abs(_rel_yaw_deg(plug_st, strip_st))
    return (
        dx <= 0.0055
        and dy <= 0.0055
        and (-0.0025 <= dz_face <= face_gap_max)
        and rel_tilt <= 8.0
        and rel_yaw <= 12.0
    )


def _strip_undisturbed(strip_st: MechState) -> bool:
    sx, sy, sz = (float(v) for v in strip_st.origin)
    drift = math.hypot(sx - STRIP_XY[0], sy - STRIP_XY[1])
    return drift <= 0.030 and abs(sz - MAT_TOP) <= 0.006 and _tilt_deg(strip_st.quat_xyzw) <= 8.0


def evaluate_step_state(
    strip_st: MechState,
    p1_st: MechState,
    p2_st: MechState,
    wart_st: MechState,
    new_st: MechState,
    arms_released: bool = False,
) -> None:
    # 1) Unplugging either active appliance (Outlet 1 monitor or Outlet 2 lamp, > 6 mm pulled out) fails!
    if not _plug_in_outlet(p1_st, strip_st, 0, face_gap_max=0.0060) or not _plug_in_outlet(
        p2_st, strip_st, 1, face_gap_max=0.0060
    ):
        _STATE["existing_unplugged"] = True

    # 2) Tipping (> 25 deg) or dragging (> 6 cm) the free-standing power strip fails!
    sx, sy, sz = (float(v) for v in strip_st.origin)
    if _tilt_deg(strip_st.quat_xyzw) > 25.0 or math.hypot(sx - STRIP_XY[0], sy - STRIP_XY[1]) > 0.060 or sz < TOP - 0.005:
        _STATE["strip_tipped"] = True

    cur = np.concatenate([np.asarray(strip_st.origin), np.asarray(wart_st.origin), np.asarray(new_st.origin)])
    prev = _STATE["last_poses"]
    max_speed = (
        max(
            float(np.linalg.norm(cur[0:3] - prev[0:3])),
            float(np.linalg.norm(cur[3:6] - prev[3:6])),
            float(np.linalg.norm(cur[6:9] - prev[6:9])),
        )
        / CONTROL_DT
        if prev is not None
        else 0.0
    )
    _STATE["last_poses"] = cur

    all_four_seated = (
        _strip_undisturbed(strip_st)
        and _plug_in_outlet(p1_st, strip_st, 0)
        and _plug_in_outlet(p2_st, strip_st, 1)
        and _plug_in_outlet(wart_st, strip_st, 3)
        and _plug_in_outlet(new_st, strip_st, 2)
    )
    if not _STATE["existing_unplugged"] and not _STATE["strip_tipped"] and all_four_seated and arms_released and max_speed < 0.008:
        _STATE["still_steps"] = int(_STATE["still_steps"]) + 1
    else:
        _STATE["still_steps"] = 0


@on_step
def _on_sim_step(env) -> None:
    if env is None:
        return
    states = {m.name: mechkit._mech_state(env, m) for m in OBJECTS}
    req = ("power_strip", "plug_1", "plug_2", "wall_wart", "new_plug")
    if any(states.get(k) is None for k in req):
        return
    arms_clear = all(released(states[k], clearance=0.030, environment=env) for k in ("power_strip", "wall_wart", "new_plug"))
    evaluate_step_state(
        states["power_strip"],
        states["plug_1"],
        states["plug_2"],
        states["wall_wart"],
        states["new_plug"],
        arms_released=arms_clear,
    )


def failed(objects: dict[str, Any]) -> bool:
    return bool(_STATE["existing_unplugged"] or _STATE["strip_tipped"])


def score(objects: dict[str, Any]) -> float:
    strip_st = objects.get("power_strip")
    p1_st = objects.get("plug_1")
    p2_st = objects.get("plug_2")
    wart_st = objects.get("wall_wart")
    new_st = objects.get("new_plug")
    if failed(objects):
        return 0.0
    if any(x is None for x in (strip_st, p1_st, p2_st, wart_st, new_st)):
        return 0.0

    if not (_strip_undisturbed(strip_st) and _plug_in_outlet(p1_st, strip_st, 0) and _plug_in_outlet(p2_st, strip_st, 1)):
        return 0.0

    wart_in_end = _plug_in_outlet(wart_st, strip_st, 3)
    new_in_freed = _plug_in_outlet(new_st, strip_st, 2)

    if not wart_in_end and not new_in_freed:
        # Check if wall-wart has been cleanly unplugged from Outlet 3 (clearing Outlet 3 & 4 without disturbing 1 & 2) -> 0.25
        rs = _quat_to_rot(strip_st.quat_xyzw)
        rel_w = (np.asarray(wart_st.origin, dtype=float) - np.asarray(strip_st.origin, dtype=float)) @ rs
        wart_cleared_3 = abs(float(rel_w[1]) - OUTLET_Y[2]) > 0.022 or (float(rel_w[2]) - STRIP_H) > 0.018
        return 0.25 if wart_cleared_3 else 0.00

    if wart_in_end and not new_in_freed:
        return 0.50

    if wart_in_end and new_in_freed:
        if int(_STATE["still_steps"]) >= DWELL_STEPS:
            return 1.00
        return 0.75

    # If black plug was seated in 3 while leaving wall-wart unplugged -> 0.25 (since wall-wart was already plugged in!)
    return 0.25


def success(objects: dict[str, Any]) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# ---------------------------------------------------------------------------------------------------------------------
# Bimanual Closed-Loop 6-DoF Demonstration (state_reads = 0).
# Note: closedloop.py hooks YamScriptedPolicy at import time to execute demo_script(ctx) directly; placeholder_demo
# satisfies yam_arena.task_file's legacy module attribute check.
demo = placeholder_demo


def demo_script(ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    L, R = ctx.left, ctx.right

    PAD_TIP_BELOW_TCP = 0.0129
    YAW_L = -math.pi / 2.0  # Left arm jaws open along X (< 0.08 mm IK error!)
    YAW_R = +math.pi / 2.0  # Right arm jaws open along X; hold_L_xy is >136 mm away so wrists stay >45 mm clear!

    if variant == "unplug_existing":
        # Negative control: unplugs active monitor plug_1 from Outlet 1 -> latches existing_unplugged (0.00)!
        p1_xy = (STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[0])
        hov1 = np.array([p1_xy[0], p1_xy[1], MAT_TOP + STRIP_H + 0.075], dtype=float)
        yield from L.move(tuple(hov1), 1.1, FINGER_OPEN, yaw=YAW_L)
        yield from L.trim_to_measured(tuple(hov1), max_duration=0.6)
        tr = L.pos - hov1
        g1 = np.array([p1_xy[0], p1_xy[1], MAT_TOP + STRIP_H + 0.012 + PAD_TIP_BELOW_TCP], dtype=float)
        yield from L.move(tuple(g1 + tr), 0.7, FINGER_OPEN, yaw=YAW_L)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move(tuple(hov1 + tr), 0.8, FINGER_CLOSED, yaw=YAW_L)
        yield from L.home(1.5)
        return

    # 1) Left arm braces down on the left (+Y = +0.110 m) switch deck of the free-standing 340 g power strip with closed
    #    rubber pads (zero jaw-pinch torque!) so blade withdrawal/insertion forces cannot lift or slide the strip!
    hold_L_xy = (STRIP_XY[0], STRIP_XY[1] + 0.116)
    hov_L = np.array([hold_L_xy[0], hold_L_xy[1], MAT_TOP + STRIP_H + 0.060], dtype=float)

    wart3_grip_y = STRIP_XY[1] + OUTLET_Y[2] - 0.010  # Centered right over prongs & under 42 mm top lip -> zero pitch!
    hov_R_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.085], dtype=float)

    yield from ctx.both(
        L.move(tuple(hov_L), 1.15, FINGER_CLOSED, yaw=YAW_L),
        R.move(tuple(hov_R_w3), 1.15, FINGER_OPEN, yaw=YAW_R),
    )
    yield from L.trim_to_measured(tuple(hov_L), max_duration=0.65)
    trim_L = L.pos - hov_L
    yield from R.trim_to_measured(tuple(hov_R_w3), max_duration=0.65)
    trim_R_strip = R.pos - hov_R_w3

    brace_L = np.array([hold_L_xy[0], hold_L_xy[1], MAT_TOP + STRIP_H - 0.0040 + PAD_TIP_BELOW_TCP], dtype=float)
    pinch_R_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.020 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from ctx.both(
        L.move(tuple(brace_L + trim_L), 0.70, FINGER_CLOSED, yaw=YAW_L),
        R.move(tuple(pinch_R_w3 + trim_R_strip), 0.75, FINGER_OPEN, yaw=YAW_R),
    )
    yield from R.grip(FINGER_CLOSED, 0.65)
    yield from ctx.wait(0.15)

    # 2) Right arm pulls WALL_WART straight up +52 mm out of Outlet 3 while Left arm holds POWER_STRIP flat on the mat!
    lift_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.072 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(lift_w3 + trim_R_strip), 0.80, FINGER_CLOSED, yaw=YAW_R)

    if variant == "unplug_wart_only":
        park_w = np.array([STRIP_XY[0], STRIP_XY[1] - 0.125, MAT_TOP + 0.016 + PAD_TIP_BELOW_TCP], dtype=float)
        yield from R.move(tuple(park_w + trim_R_strip), 0.9, FINGER_CLOSED, yaw=YAW_R)
        yield from R.grip(FINGER_OPEN, 0.45)
        yield from L.move(tuple(hov_L + trim_L), 0.50, FINGER_CLOSED, yaw=YAW_L)
        yield from ctx.both(L.home(1.4), R.home(1.4))
        yield from ctx.wait(1.4)
        return

    # 3) Shift WALL_WART -34.0 mm along -Y to end Outlet 4 (y_rel = -0.060 m), trim sag AT Outlet 4, and press flush!
    s_xy = np.asarray(ctx.obj("power_strip").origin, dtype=float)[:2] if not ctx.offline else np.array(STRIP_XY, dtype=float)
    wart4_grip_y = float(s_xy[1]) + OUTLET_Y[3] - 0.010
    over_w4 = np.array([float(s_xy[0]), wart4_grip_y, MAT_TOP + STRIP_H + 0.072 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(over_w4), 0.80, FINGER_CLOSED, yaw=YAW_R)
    yield from R.trim_to_measured(tuple(over_w4), max_duration=0.65)
    trim_R_w4 = R.pos - over_w4

    corr_w4 = np.zeros(2, dtype=float)
    if not ctx.offline:
        w_xy = np.asarray(ctx.obj("wall_wart").origin, dtype=float)[:2]
        corr_w4 = np.array([float(s_xy[0] - w_xy[0]), float(s_xy[1] + OUTLET_Y[3] - w_xy[1])], dtype=float)

    seat_w4 = np.array([float(s_xy[0] + corr_w4[0]), wart4_grip_y + float(corr_w4[1]), MAT_TOP + STRIP_H + 0.0175 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(seat_w4 + trim_R_w4), 0.95, FINGER_CLOSED, yaw=YAW_R)
    yield from ctx.wait(0.15)
    yield from R.grip(FINGER_OPEN, 0.45)
    yield from R.move(tuple(over_w4 + trim_R_w4), 0.60, FINGER_OPEN, yaw=YAW_R)

    if variant == "move_wart_only":
        yield from L.move(tuple(hov_L + trim_L), 0.55, FINGER_CLOSED, yaw=YAW_L)
        yield from ctx.both(L.home(1.4), R.home(1.4))
        yield from ctx.wait(1.4)
        return

    # 4) Left arm returns to READY while Right arm grasps NEW_PLUG right under its 42 mm top cap and seats into Outlet 3!
    hov_new = np.array([NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], NEW_PLUG_SPAWN_Z + 0.080], dtype=float)
    yield from L.move(tuple(hov_L + trim_L), 0.50, FINGER_CLOSED, yaw=YAW_L)
    yield from ctx.both(
        L.home(duration=1.10),
        R.move(tuple(hov_new), 1.00, FINGER_OPEN, yaw=YAW_R),
    )
    yield from R.trim_to_measured(tuple(hov_new), max_duration=0.65)
    trim_R_new = R.pos - hov_new

    grasp_new = np.array([NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], NEW_PLUG_SPAWN_Z + 0.020 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(grasp_new + trim_R_new), 0.75, FINGER_OPEN, yaw=YAW_R)
    yield from R.grip(FINGER_CLOSED, 0.65)
    yield from ctx.wait(0.15)

    lift_new = np.array([NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], MAT_TOP + STRIP_H + 0.105 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(lift_new + trim_R_new), 0.75, FINGER_CLOSED, yaw=YAW_R)

    s_xy2 = np.asarray(ctx.obj("power_strip").origin, dtype=float)[:2] if not ctx.offline else np.array(STRIP_XY, dtype=float)
    out3_xy = (float(s_xy2[0]), float(s_xy2[1]) + OUTLET_Y[2])
    align_base = np.array([out3_xy[0], out3_xy[1], MAT_TOP + STRIP_H + 0.056 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(align_base), 1.05, FINGER_CLOSED, yaw=YAW_R)
    yield from R.trim_to_measured(tuple(align_base), max_duration=0.65)
    trim_R_out3 = R.pos - align_base

    corr_n3 = np.zeros(2, dtype=float)
    if not ctx.offline:
        n_xy = np.asarray(ctx.obj("new_plug").origin, dtype=float)[:2]
        corr_n3 = np.array([out3_xy[0] - float(n_xy[0]), out3_xy[1] - float(n_xy[1])], dtype=float)
        yield from R.move((align_base[0] + corr_n3[0] + trim_R_out3[0], align_base[1] + corr_n3[1] + trim_R_out3[1], align_base[2] + trim_R_out3[2]), 0.45, FINGER_CLOSED, yaw=YAW_R)

    seat_new = np.array([out3_xy[0] + corr_n3[0], out3_xy[1] + corr_n3[1], MAT_TOP + STRIP_H + 0.0165 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(seat_new + trim_R_out3), 0.85, FINGER_CLOSED, yaw=YAW_R)
    yield from ctx.wait(0.15)

    # 5) Release gripper cleanly, lift straight up clear of all 4 seated plugs, return to READY, and hold 1.4 s!
    yield from R.grip(FINGER_OPEN, 0.45)
    yield from R.move((seat_new[0] + trim_R_out3[0], seat_new[1] + trim_R_out3[1], MAT_TOP + STRIP_H + 0.095 + PAD_TIP_BELOW_TCP), 0.60, FINGER_OPEN, yaw=YAW_R)
    yield from R.home(duration=1.3)
    yield from ctx.wait(1.4)
