"""Task: clamp-standard-microplate-in-orbital-shaker-platform

A standard ANSI/SLAS 96-well microplate (127.8 mm x 85.5 mm x 14.3 mm, clear polystyrene)
sits on a clean laboratory staging pad beside a benchtop orbital shaker.
The orbital shaker features a recessed platform with fixed rear and left registration
corner stops, a spring-loaded clamping retaining bracket (prismatic slider), and a front
panel power switch.

The robot must:
1. Transfer the microplate from the staging pad into the orbital shaker platform nest,
   seating it flush against the rear and left corner registration stops.
2. Advance/clamp the spring retaining bracket against the right rim of the microplate
   to mechanically lock it against high-speed orbital agitation.
3. Toggle the front panel power switch to start the orbital shaker.
4. Retract both robot arms > 50 mm clear of the apparatus for a continuous 20-step stillness hold.

Causal Defect Latch:
Toggling the power switch before the retaining clamp bracket is secured against the microplate
permanently latches task score to 0.00 (simulating centrifugal plate ejection and biohazard spill).
Dropping the microplate below the benchtop also permanently latches score to 0.00.
"""

import math
import os
from dataclasses import dataclass
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
import numpy as np
import mechkit as mk

# -- Configuration & Dimensions (ANSI/SLAS Standard Microplate & Shaker) ---------------------------

TOP = 0.750  # Tabletop height in arena (meters)

# Microplate dimensions (ANSI/SLAS 1-2004 footprint: 127.76 x 85.48 x 14.35 mm)
PLATE_L = 0.128  # Length along X (128 mm)
PLATE_W = 0.086  # Width along Y (86 mm)
PLATE_H = 0.014  # Height along Z (14 mm)
PLATE_MASS = 0.045  # 45 grams

# Staging pad position (Left side of workspace)
PAD_X = 0.280
PAD_Y = 0.180
PAD_THICKNESS = 0.005
PLATE_INIT_Z = TOP + PAD_THICKNESS + PLATE_H / 2.0  # 0.762 m

# Orbital Shaker Base & Platform Nest (Center/Right of workspace)
# Centered at X = 0.280, Y = -0.060
SHAKER_X = 0.280
SHAKER_Y = -0.060
SHAKER_L = 0.220  # Base length along X
SHAKER_W = 0.200  # Base width along Y
SHAKER_H = 0.050  # Base height
SHAKER_TOP_Z = TOP + SHAKER_H  # 0.800 m

# Nest well dimensions: fits 128 x 86 mm plate with 1 mm clearance
NEST_CENTER_X = SHAKER_X
NEST_CENTER_Y = SHAKER_Y
NEST_DEPTH = 0.008  # 8 mm recess
NEST_FLOOR_Z = SHAKER_TOP_Z - NEST_DEPTH  # 0.792 m
PLATE_SEATED_Z = NEST_FLOOR_Z + PLATE_H / 2.0  # 0.799 m

# Retaining clamp bracket (prismatic slider along Y)
# Slides from retracted open (Y = NEST_CENTER_Y - PLATE_W/2 - 0.020) inward to clamp plate rim
CLAMP_TRAVEL = 0.025  # 25 mm travel
CLAMP_ENGAGED_STROKE = 0.012  # Clamps firmly at >= 12 mm travel

# Power switch (revolute rocker switch on front panel facing robot at +X)
SWITCH_X = SHAKER_X - SHAKER_L / 2.0 - 0.010
SWITCH_Y = SHAKER_Y
SWITCH_Z = TOP + SHAKER_H * 0.60
SWITCH_TRAVEL_DEG = 25.0

# Visual colors
COLOR_CHASSIS = (0.88, 0.88, 0.90)  # Off-white laboratory enamel
COLOR_PLATFORM = (0.20, 0.20, 0.22)  # Non-slip black silicone / anodized bed
COLOR_PLATE = (0.25, 0.65, 0.90)     # Translucent laboratory cyan/blue microplate
COLOR_WELL_LIP = (0.35, 0.75, 0.95)
COLOR_CLAMP = (0.75, 0.20, 0.20)     # Anodized safety red retaining bracket
COLOR_SWITCH = (0.15, 0.75, 0.20)    # Green rocker power switch
COLOR_PAD = (0.40, 0.45, 0.50)       # Grey silicone staging pad

# -- Assembly Construction via MechKit -------------------------------------------------------------

kit = mk.Kit()

# 1. Staging Pad (Static)
kit.part("staging_pad", (PAD_X, PAD_Y, TOP + PAD_THICKNESS / 2.0),
         [mk.box((0.160, 0.120, PAD_THICKNESS))],
         mass=None, color=COLOR_PAD)

# 2. ANSI/SLAS Standard 96-Well Microplate (Dynamic contact body)
def _plate_boxes():
    # Base perimeter skirt and 96-well grid representation
    boxes = [
        # Solid bottom skirt
        mk.box((PLATE_L, PLATE_W, 0.004), at=(0.0, 0.0, -PLATE_H / 2.0 + 0.002)),
        # Four perimeter outer rim walls
        mk.box((PLATE_L, 0.003, PLATE_H - 0.004), at=(0.0, PLATE_W / 2.0 - 0.0015, 0.002)),
        mk.box((PLATE_L, 0.003, PLATE_H - 0.004), at=(0.0, -PLATE_W / 2.0 + 0.0015, 0.002)),
        mk.box((0.003, PLATE_W - 0.006, PLATE_H - 0.004), at=(PLATE_L / 2.0 - 0.0015, 0.0, 0.002)),
        mk.box((0.003, PLATE_W - 0.006, PLATE_H - 0.004), at=(-PLATE_L / 2.0 + 0.0015, 0.0, 0.002)),
        # Inner well dividing ribs (3 structural grid dividers for genuine contact friction)
        mk.box((PLATE_L - 0.010, 0.002, 0.008), at=(0.0, 0.0, 0.0)),
        mk.box((0.002, PLATE_W - 0.010, 0.008), at=(-0.030, 0.0, 0.0)),
        mk.box((0.002, PLATE_W - 0.010, 0.008), at=(0.030, 0.0, 0.0)),
    ]
    return boxes

kit.part("microplate", (PAD_X, PAD_Y, PLATE_INIT_Z),
         _plate_boxes(),
         mass=PLATE_MASS, color=COLOR_PLATE, solver_position_iterations=16)

# 3. Orbital Shaker Base Chassis (Static)
def _shaker_chassis_boxes():
    boxes = [
        # Main housing base
        mk.box((SHAKER_L, SHAKER_W, SHAKER_H - NEST_DEPTH),
               at=(0.0, 0.0, (SHAKER_H - NEST_DEPTH) / 2.0)),
        # Raised rim walls forming the recessed corner registration nest
        # Rear registration wall (+X side of nest)
        mk.box((0.020, SHAKER_W, NEST_DEPTH),
               at=(SHAKER_L / 2.0 - 0.010, 0.0, SHAKER_H - NEST_DEPTH / 2.0)),
        # Front wall (-X side)
        mk.box((0.020, SHAKER_W, NEST_DEPTH),
               at=(-SHAKER_L / 2.0 + 0.010, 0.0, SHAKER_H - NEST_DEPTH / 2.0)),
        # Left fixed registration shoulder (+Y side stop against which microplate registers)
        mk.box((PLATE_L + 0.020, 0.025, NEST_DEPTH),
               at=(0.0, NEST_CENTER_Y - SHAKER_Y + PLATE_W / 2.0 + 0.0125 + 0.002, SHAKER_H - NEST_DEPTH / 2.0)),
    ]
    return boxes

kit.part("shaker_chassis", (SHAKER_X, SHAKER_Y, TOP),
         _shaker_chassis_boxes(),
         mass=None, color=COLOR_CHASSIS)

# 4. Spring-Loaded Retaining Clamp Bracket (Prismatic Slider along Y)
# Located on -Y side of nest, advances along +Y toward plate to clamp it against the +Y stop
CLAMP_INIT_Y = NEST_CENTER_Y - (PLATE_W / 2.0 + 0.020)
def _clamp_boxes():
    return [
        # Clamping bar extending across plate length
        mk.box((PLATE_L + 0.010, 0.014, 0.016), at=(0.0, 0.0, 0.0)),
        # Grip handle / thumb tab for robot gripper
        mk.box((0.020, 0.020, 0.024), at=(0.0, -0.012, 0.010)),
    ]

kit.part("retaining_clamp", (NEST_CENTER_X, CLAMP_INIT_Y, SHAKER_TOP_Z + 0.008),
         _clamp_boxes(),
         mass=0.120, color=COLOR_CLAMP, solver_position_iterations=16)

# Prismatic slider joint: moves along +Y (toward +Y fixed stop), spring rest at 0.0 (retracted)
kit.slider("clamp_slider", "shaker_chassis", "retaining_clamp", axis=(0, 1, 0),
           lower=0.0, upper=CLAMP_TRAVEL, start=0.0,
           friction=35.0)

# 5. Front Rocker Power Switch (Revolute joint)
kit.part("power_switch", (SWITCH_X, SWITCH_Y, SWITCH_Z),
         [mk.box((0.015, 0.025, 0.012))],
         mass=0.020, color=COLOR_SWITCH, solver_position_iterations=16)

# Hinge joint: rocker tilts down/inward about Y axis when pressed
kit.hinge("switch_hinge", "shaker_chassis", "power_switch",
          anchor=(SWITCH_X, SWITCH_Y, SWITCH_Z), axis=(0, 1, 0),
          lower_deg=0.0, upper_deg=SWITCH_TRAVEL_DEG, start_deg=0.0,
          friction=0.5)

OBJECTS = kit.objects()


# -- Anti-Cheat Scorer & Step Callbacks -----------------------------------------------------------

_TEST_TCPS = None
HOLD_STEPS_REQUIRED = 20


def hands_clear(objects) -> bool:
    targets = ["microplate", "retaining_clamp", "power_switch"]
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    if not tcps:
        return True
    for tcp in tcps:
        for name in targets:
            pos, _ = kit.pose(objects, name)
            if np.linalg.norm(np.asarray(tcp) - pos) < 0.060:
                return False
    return True


def _on_step(objects) -> None:
    p_pos, _ = kit.pose(objects, "microplate")
    clamp_travel = kit.q(objects, "clamp_slider")
    switch_deg = math.degrees(kit.q(objects, "switch_hinge"))
    mk.MEMORY["_step_cnt"] = mk.MEMORY.get("_step_cnt", 0) + 1

    # Drop failure latch: microplate dropped below tabletop
    if p_pos[2] < TOP:
        mk.MEMORY["defect"] = True

    # Causal defect latch: power switch pressed before plate is clamped in nest
    # Simulates violent orbital agitation ejecting an unsecured microplate
    plate_in_nest = (abs(p_pos[0] - NEST_CENTER_X) <= 0.035 and
                     abs(p_pos[1] - NEST_CENTER_Y) <= 0.035 and
                     abs(p_pos[2] - PLATE_SEATED_Z) <= 0.018)
    clamp_locked = clamp_travel >= CLAMP_ENGAGED_STROKE

    if switch_deg >= 15.0 and (not plate_in_nest or not clamp_locked):
        mk.MEMORY["defect"] = True

    if mk.MEMORY.get("defect"):
        mk.MEMORY["hold"] = 0
        return

    # Track instantaneous continuous stillness hold at final valid state
    # Must be currently seated in nest + clamp locked + switch on + hands clear
    shaker_started = plate_in_nest and clamp_locked and (switch_deg >= 15.0)
    if shaker_started and hands_clear(objects):
        mk.MEMORY["hold"] = mk.MEMORY.get("hold", 0) + 1
    else:
        mk.MEMORY["hold"] = 0


kit.on_step(_on_step)


def score(objects) -> float:
    if mk.MEMORY.get("defect"):
        return 0.0

    p_pos, _ = kit.pose(objects, "microplate")
    clamp_travel = kit.q(objects, "clamp_slider")
    switch_deg = math.degrees(kit.q(objects, "switch_hinge"))

    plate_in_nest = (abs(p_pos[0] - NEST_CENTER_X) <= 0.035 and
                     abs(p_pos[1] - NEST_CENTER_Y) <= 0.035 and
                     abs(p_pos[2] - PLATE_SEATED_Z) <= 0.018)
    clamp_locked = clamp_travel >= CLAMP_ENGAGED_STROKE
    switch_on = switch_deg >= 15.0

    # 1.00: Full successful operation with continuous stillness dwell
    if mk.MEMORY.get("hold", 0) >= HOLD_STEPS_REQUIRED:
        return 1.0

    # 0.75: Shaker currently started while plate is seated and clamped
    if plate_in_nest and clamp_locked and switch_on:
        return 0.75

    # 0.50: Plate seated and retaining clamp currently locked
    if plate_in_nest and clamp_locked:
        return 0.50

    # 0.35: Plate currently seated flush in nest
    if plate_in_nest:
        return 0.35

    # 0.15: Plate lifted clear of staging pad
    d_pad = float(np.linalg.norm(p_pos[:2] - np.array([PAD_X, PAD_Y])))
    if d_pad > 0.040 or (p_pos[2] - PLATE_INIT_Z) > 0.025:
        return 0.15

    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- Scripted Baseline Demonstration ---------------------------------------------------------------

@dataclass
class Waypoint:
    duration: float
    pos: tuple[float, float, float]
    finger: float
    label: str = ""
    yaw: float = 0.0


def demo() -> dict[str, list[Waypoint]]:
    plans: dict[str, list[Waypoint]] = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}
    variant = os.environ.get("DEMO_VARIANT", "")

    def add(arm: str, dt: float, pos, finger: float, label: str = "", yaw: float = 0.0) -> None:
        plans[arm].append(Waypoint(float(dt), tuple(float(v) for v in pos), float(finger), label, float(yaw)))
        clock[arm] += float(dt)

    def wait_until(arm: str, t: float, label: str = "wait") -> None:
        dt = t - clock[arm]
        if dt > 1e-6:
            last = plans[arm][-1]
            add(arm, dt, last.pos, last.finger, label, last.yaw)

    LEFT_PARK = (0.160, 0.250, 0.940)
    RIGHT_PARK = (0.160, -0.250, 0.940)
    # Using yam_arena constants
    
    PLATE_GRIP_FINGER = FINGER_CLOSED

    # Initial parked poses
    add("left", 1.0, LEFT_PARK, FINGER_OPEN, "left park", yaw=0.0)
    add("right", 1.0, RIGHT_PARK, FINGER_CLOSED, "right park", yaw=math.pi / 2)

    # 1. Left Arm: approach and grasp microplate on staging pad
    # Plate is at (PAD_X=0.280, PAD_Y=0.180, PLATE_INIT_Z=0.762)
    grip_x, grip_y = PAD_X, PAD_Y
    grip_z = 0.764

    add("left", 1.2, (grip_x, grip_y, grip_z + 0.070), FINGER_OPEN, "pre-grasp plate", yaw=0.0)
    add("left", 0.6, (grip_x, grip_y, grip_z), FINGER_OPEN, "descend on plate", yaw=0.0)
    add("left", 0.5, (grip_x, grip_y, grip_z), PLATE_GRIP_FINGER, "grasp plate rim", yaw=0.0)
    add("left", 0.8, (grip_x, grip_y, grip_z + 0.070), PLATE_GRIP_FINGER, "lift plate clear", yaw=0.0)

    if variant == "no_seat":
        add("left", 2.0, (grip_x, grip_y, grip_z + 0.070), PLATE_GRIP_FINGER, "hold suspended", yaw=0.0)
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # 2. Transfer microplate to orbital shaker nest
    target_x = NEST_CENTER_X
    target_y = NEST_CENTER_Y
    target_z = 0.801

    add("left", 1.2, (target_x, target_y, target_z + 0.060), PLATE_GRIP_FINGER, "carry over nest", yaw=0.0)
    add("left", 0.8, (target_x, target_y, target_z), PLATE_GRIP_FINGER, "seat into nest", yaw=0.0)
    add("left", 0.4, (target_x, target_y, target_z), FINGER_OPEN, "release plate", yaw=0.0)
    add("left", 0.6, (target_x, target_y, target_z + 0.040), FINGER_OPEN, "retract left arm", yaw=0.0)
    add("left", 1.0, LEFT_PARK, FINGER_OPEN, "park left arm", yaw=0.0)

    # 3. Right Arm: Clamp retaining bracket
    # Retaining bracket tab is at (NEST_CENTER_X=0.280, CLAMP_INIT_Y - 0.012 = -0.115, SHAKER_TOP_Z + 0.018 = 0.818)
    tab_x = NEST_CENTER_X
    tab_y = CLAMP_INIT_Y - 0.012
    tab_z = SHAKER_TOP_Z + 0.018

    wait_until("right", 7.2, "wait for plate transfer")
    add("right", 1.0, (tab_x, tab_y - 0.040, tab_z + 0.040), FINGER_OPEN, "approach clamp tab", yaw=0.0)
    add("right", 0.6, (tab_x, tab_y, tab_z), FINGER_OPEN, "descend on clamp tab", yaw=0.0)
    add("right", 0.4, (tab_x, tab_y, tab_z), FINGER_CLOSED, "pinch clamp tab", yaw=0.0)

    if variant == "no_clamp":
        add("right", 0.4, (tab_x, tab_y, tab_z), FINGER_OPEN, "release uncompressed", yaw=0.0)
        add("right", 1.0, RIGHT_PARK, FINGER_CLOSED, "park right arm", yaw=math.pi / 2)
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Advance clamp bracket along +Y by 0.018 m (firmly clamping plate rim)
    clamped_y = tab_y + 0.018
    add("right", 0.8, (tab_x, clamped_y, tab_z), FINGER_CLOSED, "advance clamp to lock", yaw=0.0)
    # Release clamp tab and hold position
    add("right", 0.4, (tab_x, clamped_y, tab_z), FINGER_OPEN, "release clamp handle", yaw=0.0)
    add("right", 0.6, (tab_x, clamped_y - 0.030, tab_z + 0.030), FINGER_OPEN, "retract from clamp", yaw=0.0)

    # 4. Right Arm: Toggle front panel power switch
    # Power switch is at (SWITCH_X=0.160, SWITCH_Y=-0.060, SWITCH_Z=0.780)
    sw_x, sw_y, sw_z = SWITCH_X, SWITCH_Y, SWITCH_Z
    add("right", 1.0, (sw_x - 0.040, sw_y, sw_z + 0.030), FINGER_CLOSED, "approach power switch", yaw=0.0)
    add("right", 0.6, (sw_x, sw_y, sw_z), FINGER_CLOSED, "press power switch", yaw=0.0)

    if variant == "no_release":
        # Keep finger pressed on switch without retracting
        end_t = max(clock.values()) + 1.5
        for arm in ("left", "right"):
            wait_until(arm, end_t, "hold hands on switch")
        return plans

    # 5. Retract Right Arm to Park Pose
    add("right", 0.6, (sw_x - 0.060, sw_y, sw_z + 0.040), FINGER_CLOSED, "retract from switch", yaw=0.0)
    add("right", 1.0, RIGHT_PARK, FINGER_CLOSED, "park right arm", yaw=math.pi / 2)

    # Synchronize both arms for 20-step stillness hold
    end_t = max(clock.values()) + 1.5
    for arm in ("left", "right"):
        wait_until(arm, end_t, "settle and dwell")

    return plans
