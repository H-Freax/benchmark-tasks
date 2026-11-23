"""On the station table: an affordable Magnasonic CD2800 600 mL benchtop ultrasonic cleaner ($44.99, white/silver
contoured ABS body with brushed 304 stainless tank 156 x 96 x 52 mm, rear-hinged smoked-blue domed flip cover propped
open against its 102-degree rear gravity stop, front oval control apron with spring-return ON and OFF tactile push
buttons and a red cycle-status LED) sits on a charcoal non-slip silicone bench mat at (x = 0.250, y = -0.055), filled
with 480 mL of tap water to its engraved MAX line (38.0 mm depth, water surface at +70.0 mm above the mat, 14.0 mm
below the +84.0 mm tank rim). To the robot's left on an olive-slate stow mat at (x = 0.250, y = +0.170) sits an
open-well PETG staging cradle (fabricate.py: staging_cradle.stl) holding a Heathrow Scientific HS29041B 8-well
closed-cell EVA foam floating microtube rack (54 x 82 x 20 mm, 6.0 g) loaded with eight capped 1.5 mL Eppendorf
Safe-Lock polypropylene microcentrifuge tubes each filled with 1.00 mL of water (8 x 2.0 g = 16.0 g; total loaded rack
mass 22.0 g), whose 19.0 mm tapered conical lower tips protrude below the foam underside into the cradle's open recess.

The task: float the sample-loaded microtube rack level in the ultrasonic water bath, close the hinged cover flush,
press ON to start sonication, and release cleanly without tilting the rack or wetting either gripper.

Requires -E object_jitter=0: the bath, staging cradle, water-line clearance and button coordinates are measured from
the fixed benchtop layout.
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
from closedloop import grip_rot, placeholder_demo  # noqa: E402

gripkit.use_rubber_pads()  # nitrile-cotted finger pads (1.0 / 0.8 static/dynamic friction)
from mechkit import (  # noqa: E402
    Joint,
    Mech,
    MechState,
    bore,
    box,
    cyl,
    extrude,
    finger_points,
    lamp,
    lathe,
    on_reset,
    on_step,
    released,
    set_lamp,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SCORE_OVER = "end"
SUCCESS_HOLD_S = 6.0
CONTROL_DT = 0.05  # 20 Hz (-E control_hz=20)
DWELL_STEPS = 20   # 1.0 s continuous stillness & release hold

# High-contrast colours at 320x176 camera resolution
CHARCOAL = (0.16, 0.18, 0.21)
OLIVE_SLATE = (0.27, 0.34, 0.29)
CRADLE_PETG = (0.32, 0.36, 0.40)
ABS_PEARL = (0.91, 0.92, 0.94)
ABS_SILVER = (0.72, 0.75, 0.79)
STEEL_TANK = (0.64, 0.67, 0.72)
WATER_BLUE = (0.22, 0.60, 0.88)
MAX_LINE_WHITE = (0.96, 0.96, 0.96)
LID_SMOKED_BLUE = (0.58, 0.76, 0.90)
LID_TAB_DARK = (0.24, 0.28, 0.34)
BTN_ON_TEAL = (0.10, 0.66, 0.58)
BTN_OFF_GREY = (0.32, 0.34, 0.38)
LED_ON_RED = (0.98, 0.15, 0.12)
LED_OFF_DARK = (0.16, 0.17, 0.19)
FOAM_COBALT = (0.14, 0.44, 0.86)
TUBE_PP = (0.92, 0.93, 0.95)
CAP_CORAL = (0.90, 0.24, 0.18)

# ---------------------------------------------------------------------------------------------------------------------
# Benchtop layout & Archimedes hydrostatics (station frame, m)
MAT_H = 0.004
MAT_TOP = TOP + MAT_H

BATH_XY = (0.250, -0.055)
BATH_MAT_SIZE = (0.220, 0.220, MAT_H)

STOW_XY = (0.250, 0.170)
STOW_MAT_SIZE = (0.150, 0.150, MAT_H)

# Heathrow Scientific HS29041B 8-well EVA foam rack + 8 x 1.5 mL Eppendorf tubes (1.00 mL water each)
RACK_W_X = 0.054          # 54.0 mm across X (pinched by parallel jaws across X)
RACK_L_Y = 0.082          # 82.0 mm along Y
RACK_FOAM_H = 0.020       # 20.0 mm closed-cell EVA foam thickness (z_local = -0.010..+0.010)
TUBE_TIP_BELOW = 0.019    # 19.0 mm tapered conical tube tips protruding below foam underside (z_local down to -0.029)
TUBE_CAP_ABOVE = 0.0045   # 4.5 mm proud collar + snap cap above foam top (z_local up to +0.0145)
RACK_MASS_KG = 0.022      # 6.0 g EVA foam + 8 x 2.0 g water-filled 1.5 mL tubes = 22.0 g exact

# Staging cradle (fabricate.py: staging_cradle.stl) on STOW_MAT
CRADLE_LEDGE_H = 0.023    # +23.0 mm foam-underside support ledge above STOW_MAT (tube tips 4.0 mm clear of 3.0 mm floor)
RACK_SPAWN_Z = MAT_TOP + CRADLE_LEDGE_H + RACK_FOAM_H / 2.0 + 0.0004  # foam center at MAT_TOP + 0.0334 m

# Magnasonic CD2800 600 mL tank & Archimedes floating equilibrium (relative to MAT_TOP)
TANK_HALF_X = 0.048       # 96.0 mm inner width along X
TANK_HALF_Y = 0.078       # 156.0 mm inner length along Y
TANK_FLOOR_REL_Z = 0.032  # +32.0 mm stainless tank floor above MAT_TOP
TANK_RIM_REL_Z = 0.084    # +84.0 mm top tank rim above MAT_TOP (52.0 mm tank depth)
WATER_REL_Z = 0.070       # +70.0 mm MAX fill line (38.0 mm water depth = 480 mL; 14.0 mm below rim)
WATER_WORLD_Z = MAT_TOP + WATER_REL_Z

# Archimedes hydrostatics of the 22.0 g loaded rack:
#   - 8 submerged 19.0 mm conical tips (r = 2.0..5.3 mm) displace 8 x 0.913 mL = 7.30 mL (7.30 g).
#   - Remaining 14.70 g is supported by 38.1 cm^2 net foam underside -> 3.86 mm foam draft (~3.9 mm).
#   - Foam underside sits at WATER_REL_Z - 0.0039 = +0.0661 m; tube tips sit at +0.0471 m (15.1 mm above +0.0320 m floor).
#   - Foam center sits at +0.0761 m; foam top sits at +0.0861 m (16.1 mm dry foam freeboard above +0.0700 m water!).
FOAM_DRAFT_M = 0.0039
HYDRO_SUPPORT_TOP_REL_Z = WATER_REL_Z - FOAM_DRAFT_M - TUBE_TIP_BELOW  # +0.0471 m above MAT_TOP
RACK_FLOAT_WORLD_Z = MAT_TOP + WATER_REL_Z - FOAM_DRAFT_M + RACK_FOAM_H / 2.0  # MAT_TOP + 0.0761 m
DRY_FINGER_MIN_WORLD_Z = WATER_WORLD_Z + 0.004  # any finger pad below water + 4.0 mm inside open tank wets gripper!

# Rear-hinged domed flip cover (hinge at rear +X rim, open at q=0 leaning 12 deg past vertical, closed flat at q=-1.780 rad)
LID_HINGE_AT = (0.052, 0.0, 0.085)
LID_CLOSED_RAD = -math.radians(102.0)  # -1.7802 rad: brings the +102 deg open lid flat (0 deg) onto the +84 mm rim
LID_CLOSED_MAX_DEG = 6.0               # within 6 deg of -102 deg closed stop

# Front oval control apron buttons (vertical -Z press, 4.0 mm travel, >= 2.5 mm actuation click, 1.0 mm bottom clearance above +54.0 mm apron)
ON_BTN_AT = (-0.090, -0.026, 0.059)
OFF_BTN_AT = (-0.090, +0.026, 0.059)
LED_AT = (-0.090, 0.000, 0.0545)
BTN_TRAVEL = 0.0040
BTN_CLICK_M = -0.0025  # 2.5 mm downward press clicks the tactile switch
BTN_SPRING = 240.0     # N/m (~0.96 N crisp tactile PCB dome switch at 4.0 mm travel)


def _qy(rad: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) rotating by ``rad`` about +Y."""
    h = 0.5 * rad
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def _rot_y_pt(pt: tuple[float, float, float], rad: float) -> tuple[float, float, float]:
    c, s = math.cos(rad), math.sin(rad)
    x, y, z = pt
    return (c * x + s * z, y, -s * x + c * z)


def _qmul(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


# ---------------------------------------------------------------------------------------------------------------------
# Mats & Open-Well Staging Cradle (matching fabricate.py: staging_cradle.stl)
BATH_MAT = Mech(
    "bath_mat",
    pos=(BATH_XY[0], BATH_XY[1], TOP),
    mass=None,
    friction=1.0,
    shapes=(box((0.0, 0.0, MAT_H / 2.0), BATH_MAT_SIZE, CHARCOAL),),
)

STOW_MAT = Mech(
    "stow_mat",
    pos=(STOW_XY[0], STOW_XY[1], TOP),
    mass=None,
    friction=1.0,
    shapes=(box((0.0, 0.0, MAT_H / 2.0), STOW_MAT_SIZE, OLIVE_SLATE),),
)


def _staging_cradle_shapes() -> tuple:
    # Exact 74 x 102 x 26 mm open-well cradle matching fabricate.py (origin at MAT_TOP)
    ox, oy = 0.037, 0.051
    wx, wy = 0.0225, 0.0375  # 45 x 75 mm central tube-tip well (> 4.5 mm X / 2.4 mm Y clearance around cones)
    py = 0.044               # 88 mm Y pocket between 3 mm end stops (3.0 mm clearance per end on 82 mm rack)
    wall_x = ox - wx         # 14.5 mm side ledge walls
    wall_y = oy - wy         # 13.5 mm end ledge walls
    lip_y = oy - py          # 7.0 mm end registration lips (z = 23..26 mm); X grip flanks stay 100% open!
    return (
        box((0.0, 0.0, 0.0015), (2 * ox, 2 * oy, 0.003), CRADLE_PETG),
        box((-(wx + wall_x / 2), 0.0, 0.0130), (wall_x, 2 * oy, 0.020), CRADLE_PETG),
        box((+(wx + wall_x / 2), 0.0, 0.0130), (wall_x, 2 * oy, 0.020), CRADLE_PETG),
        box((0.0, -(wy + wall_y / 2), 0.0130), (2 * wx, wall_y, 0.020), CRADLE_PETG),
        box((0.0, +(wy + wall_y / 2), 0.0130), (2 * wx, wall_y, 0.020), CRADLE_PETG),
        box((0.0, -(py + lip_y / 2), 0.0245), (2 * ox, lip_y, 0.003), CRADLE_PETG),
        box((0.0, +(py + lip_y / 2), 0.0245), (2 * ox, lip_y, 0.003), CRADLE_PETG),
    )


STAGING_CRADLE = Mech(
    "staging_cradle",
    pos=(STOW_XY[0], STOW_XY[1], MAT_TOP),
    mass=None,
    friction=0.9,
    shapes=_staging_cradle_shapes(),
)


# ---------------------------------------------------------------------------------------------------------------------
# 8-Well Bored EVA Foam Floating Rack + 8 Turned 1.5 mL Eppendorf Microcentrifuge Tubes (22.0 g total)
def _foam_rack_shapes() -> tuple:
    shapes = []
    # 2 x 4 grid of bored 27.0 x 20.5 x 20.0 mm closed-cell EVA foam cells -> 54.0 x 82.0 x 20.0 mm rack with 8 real
    # 11.0 mm through-bored wells (chamfered 1.0 mm lead-in).
    cell_x, cell_y = 0.0270, 0.0205
    xs = (-0.0135, +0.0135)
    ys = (-0.03075, -0.01025, +0.01025, +0.03075)
    for cx in xs:
        for cy in ys:
            shapes.append(
                bore(
                    (cx, cy, 0.0),
                    (cell_x, cell_y, RACK_FOAM_H),
                    radius=0.0055,
                    color=FOAM_COBALT,
                    sides=20,
                    lead_in=0.001,
                )
            )
            # Turned 1.5 mL Eppendorf tube: conical lower sample tip z = -0.029..-0.010 m + barrel through foam
            shapes.append(
                lathe(
                    (cx, cy, 0.0),
                    [
                        (0.0018, -TUBE_TIP_BELOW - RACK_FOAM_H / 2.0),  # z = -0.0290 m rounded conical tip
                        (0.0041, -0.0150),
                        (0.0044, -0.0102),
                        (0.0052, -0.0095),
                        (0.0052, +0.0105),
                        (0.0062, +0.0112),  # tube rim lip resting on foam deck
                        (0.0062, +0.0120),
                    ],
                    color=TUBE_PP,
                    sides=18,
                )
            )
            # Coral-red Safe-Lock snap cap + look-only side thumb tab for unmistakable tube legibility
            shapes.append(
                cyl((cx, cy, +0.01325), radius=0.0065, height=0.0025, color=CAP_CORAL, sides=18)
            )
            shapes.append(
                dataclasses.replace(
                    box((cx + (0.0072 if cx > 0 else -0.0072), cy, +0.0133), (0.0035, 0.0050, 0.0018), CAP_CORAL),
                    collide=False,
                )
            )
    return tuple(shapes)


FOAM_RACK = Mech(
    "foam_rack",
    pos=(STOW_XY[0], STOW_XY[1], RACK_SPAWN_Z),
    mass=RACK_MASS_KG,
    friction=0.85,
    shapes=_foam_rack_shapes(),
)


# ---------------------------------------------------------------------------------------------------------------------
# Magnasonic CD2800 600 mL Ultrasonic Cleaner (body + rear-hinged domed flip cover + ON/OFF buttons + cycle LED)
def _bath_body_shapes() -> tuple:
    shapes = []
    # Outer sculpted housing: X in [-0.070, +0.070], Y in [-0.100, +0.100], Z in [0.0, +0.084],
    # hollow around the 96 x 156 mm stainless tank (X in [-0.048, +0.048], Y in [-0.078, +0.078], floor Z = +0.032).
    wall_x = 0.022  # (-0.070..-0.048) and (+0.048..+0.070)
    wall_y = 0.022  # (-0.100..-0.078) and (+0.078..+0.100)
    # Lower base plinth under the stainless tank (z = 0..0.032 m)
    shapes.append(box((0.0, 0.0, 0.016), (0.140, 0.200, 0.032), ABS_PEARL))
    # Stainless steel tank floor lining (look + collider top at z = +0.032 m)
    shapes.append(box((0.0, 0.0, 0.031), (2 * TANK_HALF_X, 2 * TANK_HALF_Y, 0.002), STEEL_TANK))
    # Four perimeter walls forming the 96 x 156 x 52 mm tank basin (z = 0.032..0.084 m)
    shapes.append(box((-TANK_HALF_X - wall_x / 2, 0.0, 0.058), (wall_x, 0.200, 0.052), ABS_PEARL))
    shapes.append(box((+TANK_HALF_X + wall_x / 2, 0.0, 0.058), (wall_x, 0.200, 0.052), ABS_PEARL))
    shapes.append(box((0.0, -TANK_HALF_Y - wall_y / 2, 0.058), (2 * TANK_HALF_X, wall_y, 0.052), ABS_PEARL))
    shapes.append(box((0.0, +TANK_HALF_Y + wall_y / 2, 0.058), (2 * TANK_HALF_X, wall_y, 0.052), ABS_PEARL))
    # Brushed stainless inner wall liners (visual contrast)
    shapes.append(dataclasses.replace(box((-TANK_HALF_X + 0.001, 0.0, 0.058), (0.002, 2 * TANK_HALF_Y, 0.050), STEEL_TANK), collide=False))
    shapes.append(dataclasses.replace(box((+TANK_HALF_X - 0.001, 0.0, 0.058), (0.002, 2 * TANK_HALF_Y, 0.050), STEEL_TANK), collide=False))
    shapes.append(dataclasses.replace(box((0.0, -TANK_HALF_Y + 0.001, 0.058), (2 * TANK_HALF_X, 0.002, 0.050), STEEL_TANK), collide=False))
    shapes.append(dataclasses.replace(box((0.0, +TANK_HALF_Y - 0.001, 0.058), (2 * TANK_HALF_X, 0.002, 0.050), STEEL_TANK), collide=False))
    # Engraved MAX 480 mL waterline witness stripe at z = +0.070 m & translucent water surface (look-only)
    shapes.append(
        dataclasses.replace(
            box((0.0, 0.0, WATER_REL_Z - 0.001), (2 * TANK_HALF_X - 0.004, 2 * TANK_HALF_Y - 0.004, 0.002), WATER_BLUE),
            collide=False,
        )
    )
    shapes.append(
        dataclasses.replace(
            box((+TANK_HALF_X - 0.0025, 0.0, WATER_REL_Z + 0.0005), (0.0015, 0.120, 0.0015), MAX_LINE_WHITE),
            collide=False,
        )
    )
    # Invisible hydrostatic equilibrium support plane at z_top = +0.0471 m (supports the 8 conical tube tips at exact
    # Archimedes 3.9 mm foam draft -> foam centre settles at MAT_TOP + 0.0761 m with 16.1 mm dry foam freeboard).
    shapes.append(
        dataclasses.replace(
            box((0.0, 0.0, HYDRO_SUPPORT_TOP_REL_Z / 2.0), (2 * TANK_HALF_X - 0.004, 2 * TANK_HALF_Y - 0.004, HYDRO_SUPPORT_TOP_REL_Z), WATER_BLUE),
            visible=False,
        )
    )
    # Sculpted front oval control apron (-X side: x = -0.106..-0.070 m, y = -0.062..+0.062 m, z = 0..+0.054 m)
    shapes.append(box((-0.088, 0.0, 0.027), (0.036, 0.124, 0.054), ABS_PEARL))
    shapes.append(
        dataclasses.replace(
            cyl((-0.088, 0.0, 0.0542), radius=0.044, height=0.0015, color=ABS_SILVER, sides=28),
            collide=False,
        )
    )
    # Red cycle-status indicator LED right between ON (y=-26 mm) and OFF (y=+26 mm)
    shapes.extend(
        lamp("cycle_led", at=LED_AT, radius=0.0045, colors={"on": LED_ON_RED}, off=LED_OFF_DARK, height=0.0025)
    )
    return tuple(shapes)


def _lid_open_shapes() -> tuple:
    """Build the domed smoked-blue Magnasonic CD2800 cover in its closed frame relative to LID_HINGE_AT, then rotate
     every shape by +102 deg (+1.7802 rad) about +Y so q = 0.0 is the gravity-stable open pose and q = -1.7802 rad is
    closed flat on the +84 mm tank rim."""
    open_rad = -LID_CLOSED_RAD  # +1.7802 rad (+102 deg)
    q_open = _qy(open_rad)

    # Closed-frame arch profile in (u=z, v=x) for axis="y", spanning x_rel = -0.108..+0.004 m, domed to +0.021 m
    # with hollow underside ceiling at +0.016 m (z_bath = +0.101 m -> 10.4 mm clear above +0.0906 m tube caps!).
    closed_parts = [
        # Domed roof plate (centred at dx = -0.052 m, dz = +0.0185 m; 108 mm in X, 172 mm in Y, 5 mm thick)
        ((-0.052, 0.0, +0.0185), (0.108, 0.172, 0.005), LID_SMOKED_BLUE, (1.0, 0.0, 0.0, 0.0)),
        # Rear hinge knuckle skirt (dx = -0.003 m, dz = +0.008 m)
        ((-0.003, 0.0, +0.0080), (0.010, 0.156, 0.016), LID_SMOKED_BLUE, (1.0, 0.0, 0.0, 0.0)),
        # Front rim skirt that lands flush on the front tank rim at q = -1.7802 rad (dx = -0.102 m, dz = +0.0075 m)
        ((-0.102, 0.0, +0.0075), (0.010, 0.168, 0.017), LID_SMOKED_BLUE, (1.0, 0.0, 0.0, 0.0)),
        # Left and right side skirts (dy = +/-0.082 m, dz = +0.008 m)
        ((-0.052, -0.082, +0.0080), (0.096, 0.008, 0.016), LID_SMOKED_BLUE, (1.0, 0.0, 0.0, 0.0)),
        ((-0.052, +0.082, +0.0080), (0.096, 0.008, 0.016), LID_SMOKED_BLUE, (1.0, 0.0, 0.0, 0.0)),
        # Front molded finger-grip handle tab (dx = -0.109 m, width 40 mm in Y, thickness 8 mm)
        ((-0.109, 0.0, +0.0085), (0.012, 0.040, 0.008), LID_TAB_DARK, (1.0, 0.0, 0.0, 0.0)),
    ]
    out = []
    for at_c, sz, col, q_local in closed_parts:
        out.append(box(_rot_y_pt(at_c, open_rad), sz, col, tilt=_qmul(q_open, q_local)))
    return tuple(out)


def _button_cap_shapes(color: tuple[float, float, float]) -> tuple:
    # Matches fabricate.py: button_actuator_cap.stl (14 mm X x 22 mm Y x 6 mm Z tactile actuator cap)
    return (
        box((0.0, 0.0, 0.002), (0.014, 0.022, 0.004), color),
        box((0.0, 0.0, 0.005), (0.009, 0.017, 0.002), color),
    )


ULTRASONIC_BATH = Mech(
    "ultrasonic_bath",
    pos=(BATH_XY[0], BATH_XY[1], MAT_TOP),
    mass=None,
    friction=0.9,
    shapes=_bath_body_shapes(),
    joints=(
        Joint(
            "lid",
            "hinge",
            axis="y",
            at=LID_HINGE_AT,
            low=LID_CLOSED_RAD,
            high=0.0,
            spring=0.0,
            damping=0.10,
            hold=0.0,
            mass=0.140,
            armature=0.006,
            shapes=_lid_open_shapes(),
        ),
        Joint(
            "on_btn",
            "slide",
            axis="z",
            at=ON_BTN_AT,
            low=-BTN_TRAVEL,
            high=0.0,
            spring=BTN_SPRING,
            damping=6.0,
            mass=0.015,
            armature=0.040,
            shapes=_button_cap_shapes(BTN_ON_TEAL),
        ),
        Joint(
            "off_btn",
            "slide",
            axis="z",
            at=OFF_BTN_AT,
            low=-BTN_TRAVEL,
            high=0.0,
            spring=BTN_SPRING,
            damping=6.0,
            mass=0.015,
            armature=0.040,
            shapes=_button_cap_shapes(BTN_OFF_GREY),
        ),
    ),
)

OBJECTS = [BATH_MAT, STOW_MAT, STAGING_CRADLE, ULTRASONIC_BATH, FOAM_RACK]


# ---------------------------------------------------------------------------------------------------------------------
# Episode trackers & real-time physics hooks (solid-state ON/OFF timer latch + flood & wet-gripper hazard latches)
_STATE: dict[str, Any] = {
    "cycle_on": False,
    "on_was_down": False,
    "off_was_down": False,
    "rack_left_cradle": False,
    "rack_flooded": False,
    "gripper_wet": False,
    "still_steps": 0,
    "last_rack_pos": None,
}


def reset_episode_state() -> None:
    _STATE["cycle_on"] = False
    _STATE["on_was_down"] = False
    _STATE["off_was_down"] = False
    _STATE["rack_left_cradle"] = False
    _STATE["rack_flooded"] = False
    _STATE["gripper_wet"] = False
    _STATE["still_steps"] = 0
    _STATE["last_rack_pos"] = None
    set_lamp(ULTRASONIC_BATH, "cycle_led", "off")


@on_reset
def _on_sim_reset(env) -> None:
    reset_episode_state()


def _quat_tilt_deg(quat_xyzw: tuple[float, float, float, float]) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y = x / n, y / n
    up_z = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.degrees(math.acos(up_z))


def _lid_gap_from_closed_deg(bath_state: MechState) -> float:
    q_lid = float(bath_state.joints.get("lid", 0.0))
    return abs(math.degrees(q_lid - LID_CLOSED_RAD))


def _rack_floating_level(rack_state: MechState) -> bool:
    rx, ry, rz = (float(v) for v in rack_state.origin)
    dx = abs(rx - BATH_XY[0])
    dy = abs(ry - BATH_XY[1])
    dz = abs(rz - RACK_FLOAT_WORLD_Z)
    tilt = _quat_tilt_deg(rack_state.quat_xyzw)
    # Tank interior clearance is +/-21 mm in X and +/-37 mm in Y; floating center is MAT_TOP + 76.1 mm (+/- 6 mm)
    return dx <= 0.020 and dy <= 0.036 and dz <= 0.006 and tilt <= 6.0


def evaluate_step_state(
    rack_state: MechState,
    bath_state: MechState,
    pad_clouds: list[np.ndarray] | None = None,
    arms_released: bool = False,
) -> None:
    """Update solid-state ON/OFF electronics, irreversible flood / wet-gripper latches, and 20-step stillness counter."""
    rx, ry, rz = (float(v) for v in rack_state.origin)
    tilt = _quat_tilt_deg(rack_state.quat_xyzw)

    # Detect departure from staging cradle (> 8 mm lift or > 15 mm lateral move)
    if abs(rx - STOW_XY[0]) > 0.015 or abs(ry - STOW_XY[1]) > 0.015 or rz > RACK_SPAWN_Z + 0.008:
        _STATE["rack_left_cradle"] = True

    # Irreversible Defect 1: tilting rack > 18 deg or dropping below benchtop floods/spills sample wells
    if tilt > 18.0 or rz < MAT_TOP + 0.015:
        _STATE["rack_flooded"] = True

    # Irreversible Defect 2: wetting shared grippers (any finger pad entering open tank XY below WATER_WORLD_Z + 4 mm)
    lid_gap_deg = _lid_gap_from_closed_deg(bath_state)
    if pad_clouds and lid_gap_deg > 15.0:
        pts = np.asarray(pad_clouds, dtype=float).reshape(-1, 3)
        if pts.size > 0:
            in_tank = (
                (np.abs(pts[:, 0] - BATH_XY[0]) < (TANK_HALF_X + 0.004))
                & (np.abs(pts[:, 1] - BATH_XY[1]) < (TANK_HALF_Y + 0.004))
            )
            if np.any(in_tank) and float(np.min(pts[in_tank, 2])) < DRY_FINGER_MIN_WORLD_Z:
                _STATE["gripper_wet"] = True

    # Magnasonic CD2800 solid-state ON / OFF tactile switch latch (no bogus "empty bath = dry run" penalty!)
    q_on = float(bath_state.joints.get("on_btn", 0.0))
    q_off = float(bath_state.joints.get("off_btn", 0.0))
    on_down = q_on <= BTN_CLICK_M
    off_down = q_off <= BTN_CLICK_M

    if off_down and not _STATE["off_was_down"]:
        _STATE["cycle_on"] = False
        set_lamp(ULTRASONIC_BATH, "cycle_led", "off")
    elif on_down and not _STATE["on_was_down"]:
        _STATE["cycle_on"] = True
        set_lamp(ULTRASONIC_BATH, "cycle_led", "on")

    _STATE["on_was_down"] = on_down
    _STATE["off_was_down"] = off_down

    # Track instantaneous rack linear speed for 1.0 s (20-step) stillness dwell
    cur_pos = np.array([rx, ry, rz], dtype=float)
    prev_pos = _STATE["last_rack_pos"]
    speed = float(np.linalg.norm(cur_pos - prev_pos) / CONTROL_DT) if prev_pos is not None else 0.0
    _STATE["last_rack_pos"] = cur_pos

    buttons_released = (q_on > -0.0010) and (q_off > -0.0010)
    if (
        not _STATE["rack_flooded"]
        and not _STATE["gripper_wet"]
        and _rack_floating_level(rack_state)
        and lid_gap_deg <= LID_CLOSED_MAX_DEG
        and _STATE["cycle_on"]
        and buttons_released
        and arms_released
        and speed < 0.010
    ):
        _STATE["still_steps"] = int(_STATE["still_steps"]) + 1
    else:
        _STATE["still_steps"] = 0


@on_step
def _on_sim_step(env) -> None:
    if env is None:
        return
    rack_st = mechkit._mech_state(env, FOAM_RACK)
    bath_st = mechkit._mech_state(env, ULTRASONIC_BATH)
    pads = finger_points(env)
    arms_clear = released(rack_st, clearance=0.030, environment=env) and released(
        bath_st, clearance=0.030, environment=env
    )
    evaluate_step_state(rack_st, bath_st, pad_clouds=pads, arms_released=arms_clear)


def failed(objects: dict[str, Any]) -> bool:
    return bool(_STATE["rack_flooded"] or _STATE["gripper_wet"])


def score(objects: dict[str, Any]) -> float:
    """End-of-episode physical score (zero positive latches; every condition checked live at the current step)."""
    if failed(objects):
        return 0.0

    rack_st = objects.get("foam_rack")
    bath_st = objects.get("ultrasonic_bath")
    if rack_st is None or bath_st is None:
        return 0.0

    floating = _rack_floating_level(rack_st)
    if not floating:
        return 0.0

    lid_closed = _lid_gap_from_closed_deg(bath_st) <= LID_CLOSED_MAX_DEG
    if not lid_closed:
        return 0.25

    if not _STATE["cycle_on"]:
        return 0.50

    if int(_STATE["still_steps"]) >= DWELL_STEPS:
        return 1.00

    return 0.75


def success(objects: dict[str, Any]) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# ---------------------------------------------------------------------------------------------------------------------
# Closed-Loop 6-DoF Bimanual Demonstration (dry high-rim rack transfer + over-center lid close + ON click + release)
demo = placeholder_demo


def _lid_push_world(q_lid: float, radius_m: float = 0.065, y_off: float = -0.025) -> np.ndarray:
    """World point on the outer roof face of the domed flip cover near centre-right (61 mm inside right skirt)."""
    alpha = -LID_CLOSED_RAD + q_lid
    hx, hy, hz = _rot_y_pt((-radius_m, y_off, +0.021), alpha)
    return np.array(
        [
            BATH_XY[0] + LID_HINGE_AT[0] + hx,
            BATH_XY[1] + LID_HINGE_AT[1] + hy,
            MAT_TOP + LID_HINGE_AT[2] + hz,
        ],
        dtype=float,
    )


def demo_script(ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    L, R = ctx.left, ctx.right

    # YAM URDF exact fingertip geometry:
    #   - At yaw = -pi/2, parallel jaws open/close along world X (93.9 mm open gap straddling the 54.0 mm X-wide rack
    #     with 20.0 mm clearance per side on cradle and 14.7 mm outer-pad clearance inside the 96.0 mm wide tank).
    #   - Bottom of finger pads sits at TCP_Z - 0.0129 m. Setting TCP_Z = rack_center_z + 0.0124 m places pad bottoms
    #     at rack_center_z - 0.0005 m (10.5 mm vertical foam grip, 9.9 mm above cradle ledge, and >= 8.1 mm DRY above water!).
    PAD_TIP_BELOW_TCP = 0.0129
    RACK_TCP_Z_OFF = 0.0124
    YAW_X_JAWS = -math.pi / 2.0

    # 1) Left arm hovers 55 mm above staged 8-well foam rack, trims PD wrist sag, and pinches the upper 10.5 mm dry foam band.
    hover_L = np.array([STOW_XY[0], STOW_XY[1], RACK_SPAWN_Z + 0.055], dtype=float)
    yield from L.move(tuple(hover_L), 1.2, FINGER_OPEN, yaw=YAW_X_JAWS)
    yield from L.trim_to_measured(tuple(hover_L), max_duration=0.8)
    trim_L_stow = L.pos - hover_L

    grasp_L = np.array([STOW_XY[0], STOW_XY[1], RACK_SPAWN_Z + RACK_TCP_Z_OFF], dtype=float)
    yield from L.move(tuple(grasp_L + trim_L_stow), 0.9, FINGER_OPEN, yaw=YAW_X_JAWS)
    yield from L.trim_to_measured(tuple(grasp_L), max_duration=0.6)
    yield from L.grip(FINGER_CLOSED, 0.80)
    yield from ctx.wait(0.25)

    # Lift straight up to clear cradle & tank rim (+135 mm above MAT_TOP), keeping rack dead level (< 0.1 deg IK error)
    transit_z = MAT_TOP + 0.135
    lift_L = np.array([STOW_XY[0], STOW_XY[1], transit_z], dtype=float)
    yield from L.move(tuple(lift_L + trim_L_stow), 0.9, FINGER_CLOSED, yaw=YAW_X_JAWS)

    if variant == "tilt_flood":
        # Negative control: tilt > 18 deg during transfer -> floods sample wells -> permanent 0.0
        yield from L.move((BATH_XY[0], 0.050, transit_z), 1.2, FINGER_CLOSED, yaw=YAW_X_JAWS, pitch=0.45)
        yield from ctx.wait(1.0)
        return

    # Translate level over the open Magnasonic CD2800 tank and re-trim gravity sag for the bath reach + 22 g payload
    over_bath = np.array([BATH_XY[0], BATH_XY[1], transit_z], dtype=float)
    yield from L.move(tuple(over_bath + trim_L_stow), 1.5, FINGER_CLOSED, yaw=YAW_X_JAWS)
    yield from L.trim_to_measured(tuple(over_bath), max_duration=0.8)
    trim_L_bath = L.pos - over_bath

    # Lower rack gently 2.5 mm above Archimedes floating equilibrium (foam center settles at MAT_TOP + 0.0761 m;
    # pad bottoms stop at MAT_TOP + 0.0781 m -> 8.1 mm DRY above WATER_WORLD_Z = MAT_TOP + 0.0700 m!)
    place_L = np.array([BATH_XY[0], BATH_XY[1], RACK_FLOAT_WORLD_Z + RACK_TCP_Z_OFF + 0.0025], dtype=float)
    yield from L.move(tuple(place_L + trim_L_bath), 1.1, FINGER_CLOSED, yaw=YAW_X_JAWS)
    yield from ctx.wait(0.20)
    yield from L.grip(FINGER_OPEN, 0.50)
    yield from ctx.wait(0.15)

    # Retract left arm straight up out of tank and return to READY
    yield from L.move(tuple(over_bath + trim_L_bath), 0.9, FINGER_OPEN, yaw=YAW_X_JAWS)
    if variant == "rack_only":
        yield from L.home(duration=1.8)
        yield from ctx.wait(1.5)
        return

    # 2) Right arm positions closed pad tips 12 mm behind the open lid roof (r=65 mm, y_off=-25 mm), trims sag, and
    #    sweeps forward (-X) & down along the hinge arc so gravity + gentle press seats the cover flush on the +84 mm rim.
    p_open = _lid_push_world(0.0, radius_m=0.065, y_off=-0.025)
    app_R_lid = np.array([p_open[0] + 0.030, p_open[1], p_open[2] + PAD_TIP_BELOW_TCP], dtype=float)
    yield from ctx.both(
        L.home(duration=1.9),
        R.move(tuple(app_R_lid), 1.7, FINGER_CLOSED),
    )
    yield from R.trim_to_measured(tuple(app_R_lid), max_duration=0.8)
    trim_R_lid = R.pos - app_R_lid

    for frac in (0.15, 0.35, 0.60, 0.85, 1.00):
        q_step = frac * LID_CLOSED_RAD
        alpha = -LID_CLOSED_RAD + q_step
        p_arc = _lid_push_world(q_step, radius_m=0.065, y_off=-0.025)
        pt = np.array(
            [
                p_arc[0] + 0.013 * math.sin(alpha),
                p_arc[1],
                p_arc[2] + PAD_TIP_BELOW_TCP - 0.0025 * math.cos(alpha),
            ],
            dtype=float,
        )
        yield from R.move(tuple(pt + trim_R_lid), 0.45, FINGER_CLOSED)
    yield from ctx.wait(0.20)

    if variant == "no_sonicate":
        yield from R.move((p_arc[0], p_arc[1], MAT_TOP + 0.185), 0.50, FINGER_CLOSED)
        yield from R.move((0.200, -0.220, MAT_TOP + 0.210), 0.65, FINGER_CLOSED)
        yield from R.home(duration=1.3)
        yield from ctx.wait(1.5)
        return

    # 3) Lift clear of closed lid, rotate wrist to yaw = +pi/2 (13.2 mm narrow X profile -> >19 mm clear of closed lid tab!),
    #    hover above the front teal ON button, trim sag, and press straight down (-Z) past the 2.5 mm click threshold.
    yaw_on_R = +math.pi / 2.0
    btn_touch_tcp_z = MAT_TOP + ON_BTN_AT[2] + 0.006 + PAD_TIP_BELOW_TCP
    app_on = np.array([BATH_XY[0] + ON_BTN_AT[0], BATH_XY[1] + ON_BTN_AT[1], btn_touch_tcp_z + 0.035], dtype=float)
    yield from R.move((p_arc[0], p_arc[1], MAT_TOP + 0.165), 0.55, FINGER_CLOSED)
    yield from R.move(tuple(app_on), 0.95, FINGER_CLOSED, yaw=yaw_on_R)
    yield from R.trim_to_measured(tuple(app_on), max_duration=0.7)
    trim_R_btn = R.pos - app_on

    press_on = np.array([app_on[0], app_on[1], btn_touch_tcp_z - 0.0080], dtype=float)
    yield from R.move(tuple(press_on + trim_R_btn), 0.65, FINGER_CLOSED, yaw=yaw_on_R)
    yield from ctx.wait(0.25)

    if variant == "no_release":
        yield from ctx.wait(2.0)
        return

    # 4) Retract straight up, then pull forward-right (-X, -Y) completely clear of the bath & closed lid skirt with
    #    jaws STILL CLOSED before unwinding wrist yaw over open table and homing; hold 1.6 s still (>= 20 steps).
    yield from R.move(tuple(app_on + trim_R_btn), 0.50, FINGER_CLOSED, yaw=yaw_on_R)
    yield from R.move((0.165, -0.195, MAT_TOP + 0.175), 0.55, FINGER_CLOSED, yaw=yaw_on_R)
    yield from R.move((0.200, -0.220, MAT_TOP + 0.210), 0.55, FINGER_CLOSED, yaw=0.0)
    yield from R.home(duration=1.2)
    yield from ctx.wait(1.6)
