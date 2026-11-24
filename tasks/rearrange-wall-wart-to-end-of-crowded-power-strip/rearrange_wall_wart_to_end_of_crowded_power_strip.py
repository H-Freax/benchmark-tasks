"""On the station table: an unwired 4-outlet longitudinal NEMA 5-15R power strip (CyberPower CSB404 form factor,
214 x 50 x 35 mm ivory housing, 380 g free-standing inside a 4.5 mm desk-mat alignment frame at x = 0.245, y = -0.020)
has four grounded 3-slot longitudinal outlets along its top face at 32.0 mm pitch (Outlet 1 at y_rel = +36 mm, Outlet 2
at +4 mm, Outlet 3 at -28 mm, and end Outlet 4 at -60 mm; strip spans y_rel = -86 mm to +128 mm; each outlet has two
polarized blade slots at x_rel = +/-6.35 mm, y_slot = y_outlet + 5.2 mm and a round ground bore at x_rel = 0, y_slot =
y_outlet - 6.7 mm). Outlets 1 and 2 hold two connected 3-prong appliance plugs (blue monitor cord and grey lamp cord,
routed +X off the front desk edge) that must remain connected throughout. Outlet 3 holds a white 2-prong Class II AC
wall-wart adapter (TP-Link T120150-2B1 12V 1.5A, 34 mm X x 62 mm Y x 38 mm Z, 105 g) whose blades sit 14 mm from its +Y
top face so its 48 mm -Y body overhang spans y_rel = -76 to -14 mm, completely covering end Outlet 4.

To the robot's right at (x = 0.245, y = -0.185) on an olive-slate mat, a black 6 ft 18 AWG SVT appliance cord's molded
3-prong NEMA 5-15P plug (34 mm X x 24 mm Y x 38 mm Z, 64 g) rests prongs-down on a 18.5 mm teal desktop cord cradle
(plug_cord_cradle.stl) with a loose +X S-curve slack loop routed off the front desk edge.

The task: plug the black cord on the right into the power strip so all four cords end up plugged in; keep the blue and
grey plugs connected throughout, though you may unplug and move the white adapter. Steadying the free-standing strip,
moving the white adapter from Outlet 3 to end Outlet 4 (where its 48 mm overhang extends 22 mm harmlessly past the
-86 mm strip end, uncovering Outlet 3), and seating both the adapter and the black plug flush (<= 1.5 mm face gap, zero
exposed blade metal) earns 1.0 after releasing >= 3.0 cm clear for 1.0 s.

Requires -E object_jitter=0: the power strip, pre-seated cords, overhanging adapter, and cord cradle follow the desk layout.
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
CORD_IVORY = (0.82, 0.80, 0.75)
CLIP_SLATE = (0.25, 0.27, 0.31)
BRASS = (0.86, 0.71, 0.24)
HOLDER_TEAL = (0.20, 0.54, 0.56)

# ---------------------------------------------------------------------------------------------------------------------
# Desk layout & 1:1 CyberPower CSB404 longitudinal NEMA 5-15R / TP-Link T120150-2B1 dimensions (station frame, m)
MAT_H = 0.004
MAT_TOP = TOP + MAT_H

STRIP_XY = (0.245, -0.020)
STRIP_MAT_SIZE = (0.160, 0.280, MAT_H)

CORD_MAT_XY = (0.245, -0.185)
CORD_MAT_SIZE = (0.140, 0.090, MAT_H)

STRIP_H = 0.035          # Exact CyberPower CSB404 height: 35.0 mm (1.38 in)
STRIP_W_X = 0.050        # Exact CyberPower CSB404 width:  50.0 mm (1.97 in)
STRIP_Y_MIN = -0.086
STRIP_Y_MAX = +0.128     # Exact CyberPower CSB404 length: 214.0 mm (8.44 in)
STRIP_MASS_KG = 0.380

# Four longitudinal NEMA 5-15R outlet centers along y_rel of POWER_STRIP (32.0 mm pitch):
OUTLET_Y = (
    +0.036,  # Outlet 1 (Blue monitor cord — must remain connected throughout)
    +0.004,  # Outlet 2 (Grey lamp cord — must remain connected throughout)
    -0.028,  # Outlet 3 (White wall-wart starts here covering Outlet 4; Black cord ends here!)
    -0.060,  # Outlet 4 (End outlet 26 mm from -86 mm strip end — Wall-wart moves here, overhanging 22 mm past end!)
)

# Standard longitudinal NEMA WD 6 / 5-15 offsets relative to each outlet center (0, yc):
#   - Two parallel flat blades across X at x = +/-6.35 mm, y = yc + 5.2 mm
#   - Round ground pin along -Y at x = 0.0 mm, y = yc - 6.7 mm (11.9 mm center-to-center from blades)
BLADE_DX = 0.00635
BLADE_Y_OFF = +0.0052
GROUND_Y_OFF = -0.0067
SEAT_CLEARANCE_Z = 0.0002   # Seated plug underside rests dead flush on z_strip = +35.0 mm!
FLUSH_FACE_GAP_MAX = 0.0015  # <= 1.5 mm no-go feeler gauge: zero exposed brass blade metal in sim & real trials!
DISTURB_GAP_MAX = 0.0015     # > 1.5 mm lift on Outlet 1 or Outlet 2 exposes blade metal -> latches existing_unplugged!

# Printable teal desktop cord cradle (plug_cord_cradle.stl) staging the black plug prongs-down at (0.245, -0.185)
NEW_PLUG_STOW_XY = (0.245, -0.185)
HOLDER_XY = (0.245, -0.185)
NEW_PLUG_SPAWN_Z = MAT_TOP + 0.0185  # Side shoulders rest on 18.5 mm rails; 16.0 mm ground pin hangs 2.5 mm clear!


# ---------------------------------------------------------------------------------------------------------------------
# Mats, Printable Cord Cradle, and Routed +X Off-Workspace Cable Guides
STRIP_MAT = Mech(
    "strip_mat",
    pos=(STRIP_XY[0], STRIP_XY[1], TOP),
    mass=None,
    friction=1.0,
    shapes=(
        box((0.0, 0.0, MAT_H / 2.0), STRIP_MAT_SIZE, CHARCOAL),
        # Low 4.5 mm alignment cleats (strip_cleat_frame.stl) prevent lateral skating while allowing tip/lift if unbraced!
        box((-0.0285, -0.062, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((+0.0285, -0.062, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((-0.0285, +0.096, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((+0.0285, +0.096, MAT_H + 0.00225), (0.005, 0.036, 0.0045), CHARCOAL),
        box((0.0, -0.0895, MAT_H + 0.00225), (0.048, 0.005, 0.0045), CHARCOAL),
        box((0.0, +0.1375, MAT_H + 0.00225), (0.048, 0.005, 0.0045), CHARCOAL),
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
        # Low 1.5 mm base + +/-X support ledges at +/-15.5 mm (>4.5 mm clear of +/-6.35 mm blades & central ground pin!)
        box((0.0, 0.0, 0.00075), (0.048, 0.046, 0.0015), HOLDER_TEAL),
        box((-0.0155, 0.0, 0.00925), (0.005, 0.028, 0.0185), HOLDER_TEAL),
        box((+0.0155, 0.0, 0.00925), (0.005, 0.028, 0.0185), HOLDER_TEAL),
    ),
)

# Visual representation of the 5 routed cords & front-edge desk cable clips running forward (+X -> x = 0.47 m) away from arms
CORD_ROUTING = Mech(
    "cord_routing",
    pos=(0.0, 0.0, MAT_TOP),
    mass=None,
    friction=0.5,
    shapes=(
        # Strip's own 1.2 m (4 ft) 14 AWG SJT ivory supply cord routed +X from +Y switch end to unplugged floor drop
        dataclasses.replace(box((0.360, STRIP_XY[1] + 0.133, 0.008), (0.220, 0.008, 0.008), CORD_IVORY), collide=False),
        dataclasses.replace(box((0.350, STRIP_XY[1] + 0.133, 0.008), (0.016, 0.018, 0.014), CLIP_SLATE), collide=False),
        # Outlet 1 blue 1.8 m (6 ft) 18 AWG SVT cord & Outlet 2 grey 1.8 m (6 ft) 18 AWG SJT cord routed straight +X
        dataclasses.replace(box((0.370, STRIP_XY[1] + OUTLET_Y[0], 0.006), (0.200, 0.0065, 0.0065), PLUG_BLUE), collide=False),
        dataclasses.replace(box((0.370, STRIP_XY[1] + OUTLET_Y[1], 0.006), (0.200, 0.0065, 0.0065), PLUG_GREY), collide=False),
        dataclasses.replace(box((0.360, STRIP_XY[1] + 0.020, 0.007), (0.016, 0.056, 0.012), CLIP_SLATE), collide=False),
        # Wall-wart 1.5 m (5 ft) 22 AWG DC barrel cable slack loop routed +X through clip at (0.360, -0.086)
        dataclasses.replace(box((0.370, -0.086, 0.004), (0.200, 0.0045, 0.0045), PLUG_BLACK), collide=False),
        dataclasses.replace(box((0.360, -0.086, 0.006), (0.014, 0.016, 0.010), CLIP_SLATE), collide=False),
        # Staged black 1.8 m (6 ft) 18 AWG SVT appliance cord S-curve slack loop routed +X from cradle through clip at (0.380, -0.175)
        dataclasses.replace(box((0.370, -0.180, 0.005), (0.200, 0.0065, 0.0065), PLUG_BLACK), collide=False),
        dataclasses.replace(box((0.380, -0.180, 0.007), (0.016, 0.020, 0.012), CLIP_SLATE), collide=False),
    ),
)


# ---------------------------------------------------------------------------------------------------------------------
# Free-Standing 4-Outlet Power Strip (CyberPower CSB404: 214 x 50 x 35 mm) with True Longitudinal NEMA 5-15R Cavities
def _power_strip_shapes() -> tuple:
    """214 x 50 x 35 mm longitudinal 4-outlet power strip built from non-overlapping walls around 4 x 3-slot NEMA 5-15R
    receptacles at yc in OUTLET_Y:
    - Two parallel blade slots: x in [-0.0108, -0.0019] and [+0.0019, +0.0108] (separated by 3.8 mm central bridge!),
      y in [yc - 0.0003, yc + 0.0107] (11.0 mm), z = 10..35 mm (25 mm deep -> 16 mm pins never bottom out!)
    - Longitudinal ground bore: x in [-0.0056, +0.0056] (11.2 mm), y in [yc - 0.0121, yc - 0.0013] (10.8 mm), z = 10..35 mm"""
    y_mid = 0.5 * (STRIP_Y_MIN + STRIP_Y_MAX)
    y_len = STRIP_Y_MAX - STRIP_Y_MIN
    wall_h = STRIP_H - 0.010  # 25.0 mm upper cavity walls above 10.0 mm solid floor
    wall_zc = 0.010 + 0.5 * wall_h

    parts = [
        # Solid 10 mm lower chassis floor (50 x 214 mm)
        box((0.0, y_mid, 0.0050), (STRIP_W_X, y_len, 0.0100), STRIP_IVORY),
        # Outer -X and +X side rails (|x| in [0.0108, 0.0250], width 14.2 mm each)
        box((-0.0179, y_mid, wall_zc), (0.0142, y_len, wall_h), STRIP_IVORY),
        box((+0.0179, y_mid, wall_zc), (0.0142, y_len, wall_h), STRIP_IVORY),
        # Left (+Y = +0.056..+0.128 m) switch deck + illuminated red reset/off rocker + +Y strain-relief boot
        box((0.0, +0.092, wall_zc), (0.0216, 0.072, wall_h), STRIP_IVORY),
        dataclasses.replace(box((0.0, +0.084, STRIP_H + 0.0006), (0.012, 0.022, 0.0012), SWITCH_RED), collide=False),
        box((0.0, +0.133, 0.014), (0.014, 0.010, 0.012), CORD_IVORY),
        # Right (-Y = -0.086..-0.073 m) solid end deck supporting seated wall-wart
        box((0.0, -0.0795, wall_zc), (0.0216, 0.013, wall_h), STRIP_IVORY),
    ]

    for idx, yc in enumerate(OUTLET_Y):
        # 1) Central 3.4 mm dielectric bridge between the two parallel blade slots (blocks 180-deg reversed ground pin!)
        parts.append(box((0.0, yc + 0.0056, wall_zc), (0.0034, 0.0114, wall_h), STRIP_IVORY))
        # 2) 0.8 mm transverse divider separating blade slots from ground bore (>= 3.8 mm per-side prong clearance!)
        parts.append(box((0.0, yc - 0.0006, wall_zc), (0.0216, 0.0008, wall_h), STRIP_IVORY))
        # 3) Side cheeks flanking the central ground bore (|x| in [0.0062, 0.0108], y in [yc - 0.0125, yc - 0.0010])
        parts.append(box((-0.0085, yc - 0.00675, wall_zc), (0.0046, 0.0115, wall_h), STRIP_IVORY))
        parts.append(box((+0.0085, yc - 0.00675, wall_zc), (0.0046, 0.0115, wall_h), STRIP_IVORY))
        # 4) Inter-outlet solid bridge between this outlet's ground bore (yc - 0.0125) and next outlet's blade top (yc_next + 0.0113)
        if idx < len(OUTLET_Y) - 1:
            y_hi = yc - 0.0125
            y_lo = OUTLET_Y[idx + 1] + 0.0113
            parts.append(box((0.0, 0.5 * (y_lo + y_hi), wall_zc), (0.0216, y_hi - y_lo, wall_h), STRIP_IVORY))
        else:
            parts.append(box((0.0, -0.07275, wall_zc), (0.0216, 0.0005, wall_h), STRIP_IVORY))
        if idx == 0:
            parts.append(box((0.0, 0.5 * (0.0473 + 0.0560), wall_zc), (0.0216, 0.0560 - 0.0473, wall_h), STRIP_IVORY))

        # High-contrast dark receptacle slot visuals on top face
        parts.append(dataclasses.replace(box((-BLADE_DX, yc + BLADE_Y_OFF, STRIP_H + 0.0001), (0.0024, 0.0072, 0.0002), SLOT_DARK), collide=False))
        parts.append(dataclasses.replace(box((+BLADE_DX, yc + BLADE_Y_OFF, STRIP_H + 0.0001), (0.0024, 0.0072, 0.0002), SLOT_DARK), collide=False))
        parts.append(dataclasses.replace(box((0.0, yc + GROUND_Y_OFF, STRIP_H + 0.0001), (0.0048, 0.0048, 0.0002), SLOT_DARK), collide=False))

    return tuple(parts)


POWER_STRIP = Mech(
    "power_strip",
    pos=(STRIP_XY[0], STRIP_XY[1], MAT_TOP + 0.0002),
    mass=STRIP_MASS_KG,
    friction=0.95,
    shapes=_power_strip_shapes(),
)


# ---------------------------------------------------------------------------------------------------------------------
# Chamfered Longitudinal NEMA 5-15P (3-Prong) & NEMA 1-15P (2-Prong Wall-Wart) Builders
def _nema_blades_2prong() -> tuple:
    """Two tapered flat brass blades (14.5 mm long at x = +/-6.35 mm, y = +5.2 mm)."""
    blades = []
    for sx in (-BLADE_DX, +BLADE_DX):
        cy = BLADE_Y_OFF
        pts = [
            (sx - 0.00085, cy - 0.0021, 0.0),
            (sx + 0.00085, cy - 0.0021, 0.0),
            (sx - 0.00085, cy + 0.0021, 0.0),
            (sx + 0.00085, cy + 0.0021, 0.0),
            (sx - 0.00085, cy - 0.0021, -0.0120),
            (sx + 0.00085, cy - 0.0021, -0.0120),
            (sx - 0.00085, cy + 0.0021, -0.0120),
            (sx + 0.00085, cy + 0.0021, -0.0120),
            (sx - 0.00035, cy - 0.0012, -0.0145),
            (sx + 0.00035, cy - 0.0012, -0.0145),
            (sx - 0.00035, cy + 0.0012, -0.0145),
            (sx + 0.00035, cy + 0.0012, -0.0145),
        ]
        blades.append(hull((0.0, 0.0, 0.0), pts, BRASS))
    return tuple(blades)


def _nema_5_15p_prongs() -> tuple:
    """3-prong grounded NEMA 5-15P: two flat blades + longer 16.0 mm conical-tip round ground pin at (0, -6.7 mm)."""
    gnd = lathe((0.0, GROUND_Y_OFF, -0.0160), [(0.0009, 0.0), (0.0022, 0.0025), (0.0022, 0.0160)], BRASS)
    return (*_nema_blades_2prong(), gnd)


def _std_plug_shapes(color: tuple[float, float, float]) -> tuple:
    return (
        *_nema_5_15p_prongs(),
        box((0.0, 0.0, 0.014), (0.028, 0.024, 0.028), color),
        box((+0.020, 0.0, 0.012), (0.014, 0.009, 0.009), color),
    )


PLUG_1 = Mech(
    "plug_1",
    pos=(STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[0], MAT_TOP + STRIP_H + SEAT_CLEARANCE_Z),
    mass=0.048,
    friction=0.90,
    shapes=_std_plug_shapes(PLUG_BLUE),
)

PLUG_2 = Mech(
    "plug_2",
    pos=(STRIP_XY[0], STRIP_XY[1] + OUTLET_Y[1], MAT_TOP + STRIP_H + SEAT_CLEARANCE_Z),
    mass=0.048,
    friction=0.90,
    shapes=_std_plug_shapes(PLUG_GREY),
)


def _wall_wart_shapes() -> tuple:
    """TP-Link T120150-2B1 12V 1.5A Class II 2-prong switching wall-wart (34 mm X x 62 mm Y x 38 mm Z, 105 g):
    2-prong NEMA 1-15P blades sit near the +Y top end so y_wart spans [-0.048, +0.014 m]!
    - In Outlet 3 (yc = -0.028 m): spans y_rel = [-0.076, -0.014 m], completely covering Outlet 4 (yc = -0.060 m)!
    - In end Outlet 4 (yc = -0.060 m): spans y_rel = [-0.108, -0.046 m], overhanging 22 mm past the -0.086 m strip end
      with CoM at y_rel = -0.077 m (9 mm inside the solid strip end!) and leaving Outlet 3 100% open!"""
    return (
        *_nema_blades_2prong(),
        box((0.0, -0.017, 0.019), (0.034, 0.062, 0.038), WART_WHITE),
        # Upper 39 mm X bezel lip (<41 mm open jaws -> clean vertical extraction & zero slip!)
        box((0.0, -0.010, 0.0365), (0.039, 0.036, 0.003), WART_WHITE),
        dataclasses.replace(box((0.0, -0.017, 0.0383), (0.024, 0.042, 0.0005), WART_LABEL), collide=False),
        # +X 22 AWG DC output cord strain-relief boot (visual cuff merging into +X cord guide)
        dataclasses.replace(box((+0.022, -0.038, 0.014), (0.012, 0.007, 0.007), PLUG_BLACK), collide=False),
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
        box((0.0, 0.0, 0.019), (0.034, 0.024, 0.038), PLUG_BLACK),
        # Upper 39 mm X molded pull-cap lip + +X 18 AWG SVT strain-relief boot (visual cuff clear of +X jaw descent!)
        box((0.0, 0.0, 0.0365), (0.039, 0.024, 0.003), PLUG_BLACK),
        dataclasses.replace(box((+0.022, 0.0, 0.010), (0.012, 0.009, 0.009), PLUG_BLACK), collide=False),
    ),
)

OBJECTS = [STRIP_MAT, CORD_MAT, CORD_HOLDER, CORD_ROUTING, POWER_STRIP, PLUG_1, PLUG_2, WALL_WART, NEW_PLUG]


# ---------------------------------------------------------------------------------------------------------------------
# Live Flush-Seating (<= 1.5 mm, Zero Exposed Blade Metal) Inspection & End-Step Scorer (zero positive latches)
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


def _plug_in_outlet(
    plug_st: MechState, strip_st: MechState, outlet_idx: int, face_gap_max: float = FLUSH_FACE_GAP_MAX
) -> bool:
    """True iff plug sits coaxially in Outlet `outlet_idx` with <= 1.5 mm face gap (zero exposed blade metal)."""
    rs = _quat_to_rot(strip_st.quat_xyzw)
    rel_p = (np.asarray(plug_st.origin, dtype=float) - np.asarray(strip_st.origin, dtype=float)) @ rs
    dx = abs(float(rel_p[0]))
    dy = abs(float(rel_p[1]) - OUTLET_Y[outlet_idx])
    dz_face = float(rel_p[2]) - STRIP_H  # 0.0 when molded plug underside rests flush on +35.0 mm strip top face!
    rel_tilt = math.degrees(
        math.acos(max(-1.0, min(1.0, float(np.dot(_quat_to_rot(plug_st.quat_xyzw)[:, 2], rs[:, 2])))))
    )
    rel_yaw = abs(_rel_yaw_deg(plug_st, strip_st))
    return (
        dx <= 0.0045
        and dy <= 0.0045
        and (-0.0015 <= dz_face <= face_gap_max)
        and rel_tilt <= 5.0
        and rel_yaw <= 8.0
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
    # 1) Unplugging or lifting (> 1.5 mm, exposing blade metal) either connected appliance (Outlet 1 or 2) fails permanently!
    if not _plug_in_outlet(p1_st, strip_st, 0, face_gap_max=DISTURB_GAP_MAX) or not _plug_in_outlet(
        p2_st, strip_st, 1, face_gap_max=DISTURB_GAP_MAX
    ):
        _STATE["existing_unplugged"] = True

    # 2) Tipping (> 25 deg) or dragging (> 6 cm) the free-standing power strip fails permanently!
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

    return 0.25


def success(objects: dict[str, Any]) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# ---------------------------------------------------------------------------------------------------------------------
# Bimanual Closed-Loop 6-DoF Demonstration
demo = placeholder_demo


def demo_script(ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    L, R = ctx.left, ctx.right

    PAD_TIP_BELOW_TCP = 0.0129
    YAW_L = -math.pi / 2.0  # Left arm jaws open along X (< 0.08 mm IK error!)
    YAW_R = +math.pi / 2.0  # Right arm jaws open along X; hold_L_xy is >140 mm away so wrists stay >45 mm clear!

    if variant == "unplug_existing":
        # Negative control: lifts connected blue monitor plug_1 out of Outlet 1 -> latches existing_unplugged (0.00)!
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

    # 1) Left arm braces down on the +Y/-X (+0.122 m) switch deck corner of the free-standing 380 g power strip
    #    (>170 mm away from Outlet 3 so Left & Right wrist housings never touch!)
    hold_L_xy = (STRIP_XY[0] - 0.012, STRIP_XY[1] + 0.122)
    hov_L = np.array([hold_L_xy[0], hold_L_xy[1], MAT_TOP + STRIP_H + 0.055], dtype=float)

    wart3_grip_y = STRIP_XY[1] + OUTLET_Y[2] - 0.010
    hov_R_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.080], dtype=float)

    yield from ctx.both(
        L.move(tuple(hov_L), 1.15, FINGER_CLOSED, yaw=YAW_L),
        R.move(tuple(hov_R_w3), 1.15, FINGER_OPEN, yaw=YAW_R),
    )
    yield from L.trim_to_measured(tuple(hov_L), max_duration=0.65)
    trim_L = L.pos - hov_L
    yield from R.trim_to_measured(tuple(hov_R_w3), max_duration=0.65)
    trim_R_strip = R.pos - hov_R_w3

    brace_L = np.array([hold_L_xy[0], hold_L_xy[1], MAT_TOP + STRIP_H - 0.0040 + PAD_TIP_BELOW_TCP], dtype=float)
    pinch_R_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.019 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from ctx.both(
        L.move(tuple(brace_L + trim_L), 0.70, FINGER_CLOSED, yaw=YAW_L),
        R.move(tuple(pinch_R_w3 + trim_R_strip), 0.75, FINGER_OPEN, yaw=YAW_R),
    )
    yield from R.grip(FINGER_CLOSED, 0.65)
    yield from ctx.wait(0.15)

    # 2) Right arm pulls WALL_WART straight up +52 mm out of Outlet 3 while Left arm keeps POWER_STRIP flat!
    lift_w3 = np.array([STRIP_XY[0], wart3_grip_y, MAT_TOP + STRIP_H + 0.070 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(lift_w3 + trim_R_strip), 0.80, FINGER_CLOSED, yaw=YAW_R)

    if variant == "unplug_wart_only":
        park_w = np.array([STRIP_XY[0], STRIP_XY[1] - 0.135, MAT_TOP + 0.020 + PAD_TIP_BELOW_TCP], dtype=float)
        yield from R.move(tuple(park_w + trim_R_strip), 0.9, FINGER_CLOSED, yaw=YAW_R)
        yield from R.grip(FINGER_OPEN, 0.45)
        yield from L.move(tuple(hov_L + trim_L), 0.50, FINGER_CLOSED, yaw=YAW_L)
        yield from ctx.both(L.home(1.4), R.home(1.4))
        yield from ctx.wait(1.4)
        return

    # 3) Shift WALL_WART -32.0 mm along -Y to end Outlet 4 (y_rel = -0.060 m), align live XY & yaw, and press dead flush!
    s_xy = np.asarray(ctx.obj("power_strip").origin, dtype=float)[:2] if not ctx.offline else np.array(STRIP_XY, dtype=float)
    wart4_grip_y = float(s_xy[1]) + OUTLET_Y[3] - 0.010
    over_w4 = np.array([float(s_xy[0]), wart4_grip_y, MAT_TOP + STRIP_H + 0.068 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(over_w4), 0.80, FINGER_CLOSED, yaw=YAW_R)
    yield from R.trim_to_measured(tuple(over_w4), max_duration=0.65)
    trim_R_w4 = R.pos - over_w4

    pre_w4 = np.array([float(s_xy[0]), wart4_grip_y, MAT_TOP + STRIP_H + 0.048 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(pre_w4 + trim_R_w4), 0.40, FINGER_CLOSED, yaw=YAW_R)

    corr_w4 = np.zeros(2, dtype=float)
    if not ctx.offline:
        w_xy = np.asarray(ctx.obj("wall_wart").origin, dtype=float)[:2]
        corr_w4 = np.array([float(s_xy[0] - w_xy[0]), float(s_xy[1] + OUTLET_Y[3] - w_xy[1])], dtype=float)

    seat_w4 = np.array([float(s_xy[0] + corr_w4[0]), wart4_grip_y + float(corr_w4[1]), MAT_TOP + STRIP_H + 0.0165 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(pre_w4 + trim_R_w4 + np.array([corr_w4[0], corr_w4[1], 0.0])), 0.35, FINGER_CLOSED, yaw=YAW_R)
    yield from R.move(tuple(seat_w4 + trim_R_w4), 0.90, FINGER_CLOSED, yaw=YAW_R)
    yield from ctx.wait(0.15)
    yield from R.grip(FINGER_OPEN, 0.45)
    yield from R.move(tuple(over_w4 + trim_R_w4), 0.55, FINGER_OPEN, yaw=YAW_R)

    if variant == "move_wart_only":
        yield from L.move(tuple(hov_L + trim_L), 0.55, FINGER_CLOSED, yaw=YAW_L)
        yield from ctx.both(L.home(1.4), R.home(1.4))
        yield from ctx.wait(1.4)
        return

    # Retract Left arm to READY while Right arm approaches NEW_PLUG (strip is already seated inside 4.5 mm mat cleats
    # and downward insertion into Outlet 3 presses the strip into the mat, giving unlimited overhead clearance at Outlet 3!)
    hov_new = np.array([NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], NEW_PLUG_SPAWN_Z + 0.075], dtype=float)
    yield from L.move(tuple(hov_L + trim_L), 0.45, FINGER_CLOSED, yaw=YAW_L)
    yield from ctx.both(
        L.home(duration=1.2),
        R.move(tuple(hov_new), 0.95, FINGER_OPEN, yaw=YAW_R),
    )
    yield from R.trim_to_measured(tuple(hov_new), max_duration=0.65)
    trim_R_new = R.pos - hov_new

    grasp_new = np.array([NEW_PLUG_STOW_XY[0], NEW_PLUG_STOW_XY[1], NEW_PLUG_SPAWN_Z + 0.019 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(grasp_new + trim_R_new), 0.70, FINGER_OPEN, yaw=YAW_R)
    yield from R.grip(FINGER_CLOSED, 0.65)
    yield from ctx.wait(0.15)
    yield from R.move((NEW_PLUG_STOW_XY[0] + trim_R_new[0], NEW_PLUG_STOW_XY[1] + trim_R_new[1], MAT_TOP + STRIP_H + 0.090), 0.70, FINGER_CLOSED, yaw=YAW_R)

    s_xy2 = np.asarray(ctx.obj("power_strip").origin, dtype=float)[:2] if not ctx.offline else np.array(STRIP_XY, dtype=float)
    out3_xy = np.array([float(s_xy2[0]), float(s_xy2[1]) + OUTLET_Y[2]], dtype=float)
    over_out3 = np.array([out3_xy[0], out3_xy[1], MAT_TOP + STRIP_H + 0.082 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(over_out3), 0.90, FINGER_CLOSED, yaw=YAW_R)
    yield from R.trim_to_measured(tuple(over_out3), max_duration=0.65)
    trim_R_o3 = R.pos - over_out3

    align_base = np.array([out3_xy[0], out3_xy[1], MAT_TOP + STRIP_H + 0.052 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(align_base + trim_R_o3), 0.45, FINGER_CLOSED, yaw=YAW_R)

    corr_yaw = 0.0
    if not ctx.offline:
        corr_yaw = -math.radians(_rel_yaw_deg(ctx.obj("new_plug"), ctx.obj("power_strip")))
        yield from R.move(tuple(align_base + trim_R_o3), 0.30, FINGER_CLOSED, yaw=YAW_R + corr_yaw)

    corr_n3 = np.zeros(2, dtype=float)
    if not ctx.offline:
        n_xy = np.asarray(ctx.obj("new_plug").origin, dtype=float)[:2]
        corr_n3 = np.array([out3_xy[0] - float(n_xy[0]), out3_xy[1] - float(n_xy[1])], dtype=float)

    seat_out3 = np.array([out3_xy[0] + corr_n3[0], out3_xy[1] + corr_n3[1], MAT_TOP + STRIP_H + 0.0165 + PAD_TIP_BELOW_TCP], dtype=float)
    yield from R.move(tuple(align_base + trim_R_o3 + np.array([corr_n3[0], corr_n3[1], 0.0])), 0.35, FINGER_CLOSED, yaw=YAW_R + corr_yaw)
    yield from R.move(tuple(seat_out3 + trim_R_o3), 0.95, FINGER_CLOSED, yaw=YAW_R + corr_yaw)
    yield from ctx.wait(0.15)
    yield from R.grip(FINGER_OPEN, 0.45)
    yield from R.move(tuple(over_out3 + trim_R_o3), 0.60, FINGER_OPEN, yaw=YAW_R)

    # 5) Return Right arm cleanly to READY and hold 1.5 s continuous stillness
    yield from R.home(duration=1.4)
    yield from ctx.wait(1.5)
