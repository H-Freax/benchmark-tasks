"""Benchmark Task: seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas

Wet-Lab Research Automation & Life Sciences QC Task (Category: "Other").
Maps directly to:
- BLS SOC 19-4021: Biological Technicians (>82,000 workers)
- BLS SOC 19-4031: Chemical Technicians (>67,000 workers)
- BLS SOC 19-4010: Agricultural & Food Science Technicians (>34,000 workers)
- BLS SOC 29-2010: Clinical Laboratory Technologists & Technicians (>342,000 workers)
- BLS SOC 19-1021: Biochemists & Biophysicists (>34,000 workers)
Total Workforce: >559,000 professional laboratory and life-sciences personnel.

Apparatus:
1. Benchtop ultrasonic sonicator bath (Branson CPX / Fisherbrand style):
   - Exterior chassis: 230 mm (X) x 190 mm (Y) x 80 mm (Z) brushed stainless housing at (X=0.250, Y=-0.060, Z=TABLE_TOP).
   - Internal stainless fluid tank chamber cavity: 150 mm (X) x 120 mm (Y) x 45 mm deep, centered at (X=0.250, Y=-0.060).
   - Stainless tank liquid floor at Z = TABLE_TOP + 35 mm; resting fluid plane top at Z = TABLE_TOP + 65.5 mm (world Z = 0.813 m); foam rack center resting on fluid surface at world Z = 0.828 m.
   - Hinged transparent acrylic safety lid: revolute joint along rear rim (X=0.165, Y=-0.060, Z=TABLE_TOP + 86 mm) propped open at 70.0 degrees with counterbalance spring and friction.
   - Front membrane control console: momentary tactile DEGAS pushbutton (prismatic slider along -X, 8.0 mm stroke, >= 3.0 mm actuation) on front console at (X=0.405, Y=-0.045, Z=TABLE_TOP + 35 mm).
2. Floating microtube foam rack (buoyant EVA foam rack):
   - Dimension: 80 mm (X) x 56 mm (Y) x 30 mm (Z), mass 0.09 kg, floating/resting staging platform at (X=0.250, Y=0.180, Z=TABLE_TOP + 15 mm).
   - Loaded with 8 microcentrifuge tubes (nested in 2x4 array).

Robotic Bimanual Sequence:
1. Left arm reaches to the floating microtube foam rack in staging area (Y=+0.180), grasps body with parallel gripper, lifts vertically (Z >= 0.783 m), transfers laterally into ultrasonic bath tank, and seats it level on the interior tank fluid resting plane (Z ~ 0.828 m).
2. Left arm maintains level posture throughout transfer. If tilt exceeds 15.0 degrees at any point, liquid floods the foam wells, locking score to 0.00 (catastrophic liquid flood defect).
3. Right arm reaches to the propped-open acrylic lid (70.0 deg), swings it closed flush over the tank rim (angle <= 14.0 deg).
4. Right arm reaches to front control panel and depresses the tactile DEGAS membrane button inward along -X by >= 3.0 mm.
5. Both arms release and retract clear (TCP clearance >= 50 mm) for a continuous 20-step (1.0 s at 20 Hz) stillness hold while rack remains seated level inside tank, lid closed, and DEGAS registered.

Scoring Stages (Strict Instantaneous Physics / Zero Permanent Positive Latches):
- 0.00: Base / idle / or catastrophic rack-tip defect (> 15.0 deg tilt) / premature dry-run cavitation triggered.
- 0.25: Foam rack lifted vertically (Z >= 0.783 m) and transferred inside bath chamber perimeter (|dx| <= 38 mm, |dy| <= 35 mm, Z < 0.843 m).
- 0.50: Foam rack fully seated level on bath interior fluid resting plane (|dz| <= 12 mm from 0.828 m, tilt <= 8.0 deg).
- 0.75: Acrylic lid closed flush over tank (angle <= 14.0 deg) AND tactile DEGAS button depressed inward >= 3.0 mm.
- 1.00: DEGAS cycle triggered, rack seated level right now, lid closed flush right now, hands retracted clear >= 50 mm, and 20-step stillness dwell satisfied.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict

import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

try:
    from yam_arena.task_file import TABLE_TOP
    SCENE = "station"
    TOP = float(TABLE_TOP[SCENE])
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
except ImportError:
    TOP = 0.7475
    FINGER_OPEN = -0.04695
    FINGER_CLOSED = 0.000
    class Waypoint:
        def __init__(self, duration, pos, finger=-0.04695, label="", yaw=0.0):
            self.duration = duration
            self.pos = pos
            self.finger = finger
            self.label = label
            self.yaw = yaw

import mechkit as mk

# Visual Styling
STEEL_BRUSHED = (0.75, 0.77, 0.80)
STEEL_TANK = (0.65, 0.68, 0.72)
WATER_BLUE = (0.25, 0.60, 0.85)
LID_ACRYLIC = (0.80, 0.90, 0.95)
LID_HANDLE = (0.20, 0.22, 0.25)
FOAM_BLUE = (0.15, 0.45, 0.85)
TUBE_POLYPROPYLENE = (0.90, 0.92, 0.94)
TUBE_CAP_RED = (0.85, 0.20, 0.20)
PANEL_DARK = (0.22, 0.24, 0.26)
BUTTON_TEAL = (0.10, 0.65, 0.70)
DISPLAY_GREEN = (0.10, 0.80, 0.30)

# Table surface
TABLE_SURFACE_Z = TOP

# Benchtop Ultrasonic Bath Dimensions & Placement
# Placed on operator right/center: X = 0.250, Y = -0.060
BATH_X = 0.250
BATH_Y = -0.060
BATH_W_X = 0.230    # 230 mm outer length
BATH_D_Y = 0.190    # 190 mm outer depth
BATH_H = 0.080      # 80 mm chassis height
DECK_Z = TABLE_SURFACE_Z + BATH_H  # 0.8275 m

# Internal Tank Chamber Cavity
# Centered at X = 0.250, Y = -0.060
TANK_CENTER_X = 0.250
TANK_CENTER_Y = -0.060
TANK_W_X = 0.150    # 150 mm inner length
TANK_D_Y = 0.120    # 120 mm inner width
TANK_DEPTH = 0.045  # 45 mm deep well
TANK_FLOOR_Z = DECK_Z - TANK_DEPTH  # 0.7825 m

# Floating Microtube Foam Rack Initial Staging Pose
# Staged on operator left: X = 0.250, Y = 0.180
RACK_START_X = 0.250
RACK_START_Y = 0.180
RACK_H = 0.030      # 30 mm thick foam slab
RACK_W_X = 0.080    # 80 mm rack length (fits inside 150 mm tank with 35 mm clearance per side)
RACK_D_Y = 0.056    # 56 mm rack width (easily gripped by 90 mm open parallel gripper fingers)
RACK_START_Z = TABLE_SURFACE_Z + RACK_H / 2  # 0.7625 m

# Hinged Acrylic Bath Lid
# Rear hinge pivot along rear tank rim: X = 0.165, Y = -0.060, Z = DECK_Z + 0.006
LID_HINGE_X = 0.165
LID_HINGE_Y = -0.060
LID_HINGE_Z = DECK_Z + 0.006
LID_LEN = 0.180     # 180 mm flap extending forward (+X)
LID_OPEN_DEG = 70.0 # Propped open at 70 degrees

# Membrane Control Console & DEGAS Button (horizontal prismatic slider along -X)
# Sloped front console: button center resting at X = 0.405, Y = -0.045, Z = TABLE_SURFACE_Z + 0.035 m
BTN_REST_X = 0.405
BTN_Y = -0.045
BTN_REST_Z = TABLE_SURFACE_Z + 0.035
BTN_Z = BTN_REST_Z
BTN_STROKE = 0.008

# Parallel Gripper States (Authentic yam_arena URDF limits)
FINGER_OPEN = -0.04695
FINGER_CLOSED = 0.000


def _bath_body_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # 1. Main outer stainless chassis walls (hollow cavity in middle for tank)
    # Left outer wall (-Y)
    boxes.append(mk.box((BATH_W_X, (BATH_D_Y - TANK_D_Y) / 2, BATH_H),
                        at=(0.0, -(TANK_D_Y + (BATH_D_Y - TANK_D_Y) / 2) / 2, BATH_H / 2),
                        color=STEEL_BRUSHED))
    # Right outer wall (+Y)
    boxes.append(mk.box((BATH_W_X, (BATH_D_Y - TANK_D_Y) / 2, BATH_H),
                        at=(0.0, (TANK_D_Y + (BATH_D_Y - TANK_D_Y) / 2) / 2, BATH_H / 2),
                        color=STEEL_BRUSHED))
    # Rear outer wall (-X)
    rear_w = (BATH_W_X - TANK_W_X) / 2
    boxes.append(mk.box((rear_w, TANK_D_Y, BATH_H),
                        at=(-(TANK_W_X + rear_w) / 2, 0.0, BATH_H / 2),
                        color=STEEL_BRUSHED))
    # Front outer console wall (+X) - low-profile terrace for ergonomic keypad access
    # Extends forward to world X = 0.380 m, leaving 13 mm air clearance to resting button
    terrace_w = 0.055
    terrace_cx = TANK_W_X / 2 + terrace_w / 2  # 0.075 + 0.0275 = 0.1025 m -> world X = 0.3525 m
    # Terrace body (Z height 45 mm, front edge at world X = 0.380 m)
    boxes.append(mk.box((terrace_w, TANK_D_Y, 0.045),
                        at=(terrace_cx, 0.0, 0.0225),
                        color=STEEL_BRUSHED))
    # Bezel around DEGAS pushbutton on front vertical face (front face at world X = 0.384 m)
    boxes.append(mk.box((0.004, 0.048, 0.034),
                        at=(terrace_cx + terrace_w / 2 + 0.002, 0.015, 0.035),
                        color=PANEL_DARK))
    # Display screen on left of front console
    boxes.append(mk.box((0.004, 0.050, 0.030),
                        at=(terrace_cx + terrace_w / 2 + 0.002, -0.035, 0.035),
                        color=DISPLAY_GREEN))
    # 2. Tank floor (stainless floor inside well)
    boxes.append(mk.box((TANK_W_X, TANK_D_Y, 0.006),
                        at=(0.0, 0.0, (BATH_H - TANK_DEPTH) + 0.003),
                        color=STEEL_TANK))
    # 3. Water bath fluid layer (translucent blue resting fluid plane, 24.5 mm depth, fluid surface at world Z = 0.813 m)
    # Foam rack (height 0.030 m) resting on fluid surface has center Z = 0.813 + 0.015 = 0.828 m
    boxes.append(mk.box((TANK_W_X - 0.006, TANK_D_Y - 0.006, 0.0245),
                        at=(0.0, 0.0, (BATH_H - TANK_DEPTH) + 0.006 + 0.01225),
                        color=WATER_BLUE))
    return boxes


def _lid_boxes() -> list[mk.BoxSpec]:
    """Hinged acrylic safety cover. Origin at rear hinge pivot."""
    boxes = []
    # Transparent acrylic cover plate extending forward (+X from pivot)
    boxes.append(mk.box((LID_LEN, TANK_D_Y + 0.020, 0.008),
                        at=(LID_LEN / 2, 0.0, 0.004),
                        color=LID_ACRYLIC))
    # Front ergonomic lift handle bar
    boxes.append(mk.box((0.022, 0.060, 0.018),
                        at=(LID_LEN + 0.010, 0.0, 0.012),
                        color=LID_HANDLE))
    return boxes


def _degas_button_boxes() -> list[mk.BoxSpec]:
    """Momentary DEGAS pushbutton on front console. 48x36 mm tactile pad."""
    boxes = []
    boxes.append(mk.box((0.024, 0.048, 0.036), at=(0.0, 0.0, 0.0), color=BUTTON_TEAL))
    return boxes


def _foam_rack_boxes() -> list[mk.BoxSpec]:
    """Floating microtube foam rack with 8 microcentrifuge tubes in 2x4 array."""
    boxes = []
    # 1. Main buoyant closed-cell EVA foam slab
    boxes.append(mk.box((RACK_W_X, RACK_D_Y, RACK_H), at=(0.0, 0.0, 0.0), color=FOAM_BLUE))
    # 2. 8 Microcentrifuge tubes (2x4 array)
    xs = [-0.025, -0.008, 0.008, 0.025]
    ys = [-0.014, 0.014]
    for x in xs:
        for y in ys:
            # Polypropylene tube body collar proud of foam deck
            boxes.append(mk.box((0.011, 0.011, 0.010), at=(x, y, RACK_H / 2 + 0.005), color=TUBE_POLYPROPYLENE))
            # Attached red snap cap
            boxes.append(mk.box((0.013, 0.013, 0.004), at=(x, y, RACK_H / 2 + 0.012), color=TUBE_CAP_RED))
    return boxes


kit = mk.Kit()

# Register Bath Chassis
kit.part("bath_body", (BATH_X, BATH_Y, TABLE_SURFACE_Z),
         _bath_body_boxes(), mass=None, color=STEEL_BRUSHED)

# Hinged Acrylic Cover: revolute hinge propped open at 70 deg with counterbalance spring & Coulomb friction
# Equilibrium at 70 deg: stiffness=0.10, rest=70 deg, friction=0.08 N*m
kit.part("bath_lid", (LID_HINGE_X, LID_HINGE_Y, LID_HINGE_Z), _lid_boxes(),
         mass=0.22, color=LID_ACRYLIC, solver_position_iterations=16)
kit.hinge("lid_pivot", "bath_body", "bath_lid",
          anchor=(LID_HINGE_X, LID_HINGE_Y, LID_HINGE_Z), axis=(0, -1, 0),
          lower_deg=0.0, upper_deg=LID_OPEN_DEG, start_deg=LID_OPEN_DEG,
          stiffness=0.10, rest_deg=LID_OPEN_DEG, damping=0.02, friction=0.08)

# Tactile DEGAS Button on Front Console: prismatic slider along (-1, 0, 0) with return spring
kit.part("degas_button", (BTN_REST_X, BTN_Y, BTN_Z), _degas_button_boxes(),
         mass=0.05, color=BUTTON_TEAL, solver_position_iterations=16)
kit.slider("button_slider", "bath_body", "degas_button",
           axis=(-1, 0, 0), lower=0.000, upper=0.008, start=0.000,
           stiffness=20.0, rest=0.000, damping=1.0, friction=0.01)

# Floating Microtube Foam Rack: dynamic free body staged at (RACK_START_X, RACK_START_Y, RACK_START_Z)
kit.part("foam_rack", (RACK_START_X, RACK_START_Y, RACK_START_Z), _foam_rack_boxes(),
         mass=0.09, color=FOAM_BLUE, solver_position_iterations=16)

OBJECTS = kit.objects()


def _step_rules(objects: Dict[str, Any]):
    """Monitor live instantaneous physics, causal gates, and catastrophic defect."""
    step_id = mk._EP.get("step", 0)

    # 1. Measure Foam Rack Pose & Tilt
    rack_obj = objects.get("foam_rack")
    rack_pos = np.array([0.0, 0.0, 0.0])
    rack_tilt_deg = 0.0
    if rack_obj is not None and hasattr(rack_obj, "pos"):
        rack_pos = np.array([float(rack_obj.pos[0]), float(rack_obj.pos[1]), float(rack_obj.pos[2])])
        if hasattr(rack_obj, "rot"):
            rot = rack_obj.rot
            if len(rot) == 4:
                w, x, y, z = rot[0], rot[1], rot[2], rot[3]
                up_z = 1.0 - 2.0 * (x * x + y * y)
                rack_tilt_deg = math.degrees(math.acos(max(min(up_z, 1.0), -1.0)))

    mk.MEMORY["rack_pos"] = rack_pos
    mk.MEMORY["rack_tilt_deg"] = rack_tilt_deg

    # Causal Catastrophic Defect: excessive tilt (> 15 deg) during transfer or handling causes fluid spill/flood
    if rack_tilt_deg > 15.0:
        mk.MEMORY["rack_flooded_defect"] = True

    # Lift detection: rack bottom lifted clear of table staging plane (Z >= RACK_START_Z + 0.020 m)
    if rack_pos[2] >= (RACK_START_Z + 0.020):
        mk.MEMORY["rack_lifted_from_stage"] = True

    # Check whether rack is within tank chamber perimeter (|dx| <= 38 mm, |dy| <= 35 mm)
    dx = abs(rack_pos[0] - TANK_CENTER_X)
    dy = abs(rack_pos[1] - TANK_CENTER_Y)
    rack_in_tank_xy = (dx <= 0.038 and dy <= 0.035)

    # Target resting elevation on tank floor / water bed:
    # Resting center settled at Z ~ 0.828 m
    target_rack_z = 0.828
    dz = abs(rack_pos[2] - target_rack_z)
    rack_seated_level = (rack_in_tank_xy and dz <= 0.012 and rack_tilt_deg <= 8.0)
    mk.MEMORY["rack_seated_current"] = rack_seated_level

    # 2. Measure Acrylic Lid Angle (closed flush over rack/tube tops is <= 14.0 deg)
    try:
        lid_angle_deg = kit.q_deg(objects, "lid_pivot")
    except Exception:
        lid_angle_deg = LID_OPEN_DEG

    lid_closed = (lid_angle_deg <= 14.0)
    mk.MEMORY["lid_closed_current"] = lid_closed

    # 3. Measure DEGAS Button Depression Travel
    try:
        btn_travel_m = kit.q(objects, "button_slider")
    except Exception:
        btn_travel_m = 0.0

    btn_pressed = (btn_travel_m >= 0.003)
    mk.MEMORY["button_pressed_current"] = btn_pressed

    # Premature DEGAS Defect: pressing DEGAS without rack seated level in tank trips cavitation dry-run error
    if btn_pressed and not rack_seated_level:
        mk.MEMORY["dry_run_cavitation_defect"] = True

    # Degas cycle triggered ONLY when rack is seated in tank, lid is closed, and button is actively depressed
    if rack_seated_level and lid_closed and btn_pressed and not mk.MEMORY.get("rack_flooded_defect") and not mk.MEMORY.get("dry_run_cavitation_defect"):
        mk.MEMORY["degas_triggered"] = True

    # 4. Check Hands Retraction Clearance
    tcp_list = mk.tcps() if hasattr(mk, "tcps") else []
    lid_obj = objects.get("bath_lid")
    lid_tip_pos = None
    if lid_obj is not None:
        q_rad = math.radians(lid_angle_deg)
        lid_tip_pos = np.array([
            LID_HINGE_X + LID_LEN * math.cos(q_rad),
            LID_HINGE_Y,
            LID_HINGE_Z + LID_LEN * math.sin(q_rad),
        ])

    btn_obj = objects.get("degas_button")
    btn_pos = None
    if btn_obj is not None and hasattr(btn_obj, "pos"):
        btn_pos = np.array([float(btn_obj.pos[0]), float(btn_obj.pos[1]), float(btn_obj.pos[2])])

    if tcp_list:
        rack_clear = not any(np.linalg.norm(np.array(t) - rack_pos) < 0.060 for t in tcp_list)
        lid_clear = True
        if lid_tip_pos is not None:
            lid_clear = not any(np.linalg.norm(np.array(t) - lid_tip_pos) < 0.060 for t in tcp_list)
        btn_clear = True
        if btn_pos is not None:
            btn_clear = not any(np.linalg.norm(np.array(t) - btn_pos) < 0.060 for t in tcp_list)
        hands_clear = rack_clear and lid_clear and btn_clear
    else:
        hands_clear = True

    # 5. Stillness Hold: degas triggered, no defects, rack seated level, lid closed, hands clear >= 20 steps
    if (mk.MEMORY.get("degas_triggered") and
            not mk.MEMORY.get("rack_flooded_defect") and
            not mk.MEMORY.get("dry_run_cavitation_defect") and
            rack_seated_level and lid_closed and hands_clear):
        held = mk.MEMORY.get("hold_steps", 0) + 1
        mk.MEMORY["hold_steps"] = held
        if held >= 20:
            mk.MEMORY["hold_passed"] = True
    else:
        mk.MEMORY["hold_steps"] = 0

    if step_id % 20 == 0 or step_id < 10:
        print(f"[STEP {step_id:03d}] score={score(objects):.2f} rack_xyz=({rack_pos[0]:.3f},{rack_pos[1]:.3f},{rack_pos[2]:.3f}) tilt={rack_tilt_deg:.1f} lid_deg={lid_angle_deg:.1f} btn_mm={btn_travel_m*1e3:.1f} seated={rack_seated_level} lid_closed={lid_closed} hands_clear={hands_clear} hold={mk.MEMORY.get('hold_steps', 0)}", flush=True)


kit.on_step(_step_rules)


def failed(objects: Dict[str, Any]) -> bool:
    return bool(mk.MEMORY.get("rack_flooded_defect", False) or mk.MEMORY.get("dry_run_cavitation_defect", False))


def score(objects: Dict[str, Any]) -> float:
    """Instantaneous live physical scoring (Strict Directive #856, Zero Permanent Positive Latches):
    0.00: Base / idle / catastrophic defect triggered.
    0.25: Foam rack lifted and transferred inside bath chamber perimeter.
    0.50: Foam rack fully seated level in tank chamber well (tilt <= 8 deg, dz <= 12 mm).
    0.75: Acrylic lid closed flush (<= 14 deg) AND DEGAS button actively depressed (or degas registered with lid closed).
    1.00: DEGAS cycle triggered, rack seated level right now, lid closed flush right now, hands retracted clear >= 50 mm, held 20 steps (1.0 s).
    """
    if failed(objects):
        return 0.00

    # 1.00 Stage: degas registered, rack seated level, lid closed flush, stillness hold passed
    if (mk.MEMORY.get("hold_passed", False) and
            mk.MEMORY.get("rack_seated_current", False) and
            mk.MEMORY.get("lid_closed_current", False)):
        return 1.00

    # 0.75 Stage: degas triggered while rack seated and lid closed
    if (mk.MEMORY.get("degas_triggered", False) and
            mk.MEMORY.get("rack_seated_current", False) and
            mk.MEMORY.get("lid_closed_current", False)):
        return 0.75

    # 0.50 Stage: rack actively seated level in tank
    if mk.MEMORY.get("rack_seated_current", False):
        return 0.50

    # 0.25 Stage: rack lifted clear of staging plane AND transferred inside chamber perimeter (|dx| <= 38 mm, |dy| <= 35 mm)
    rack_pos = mk.MEMORY.get("rack_pos", np.array([0.0, 0.0, 0.0]))
    dx = abs(rack_pos[0] - TANK_CENTER_X)
    dy = abs(rack_pos[1] - TANK_CENTER_Y)
    if mk.MEMORY.get("rack_lifted_from_stage", False) and dx <= 0.038 and dy <= 0.035 and rack_pos[2] < (RACK_START_Z + 0.080):
        return 0.25

    return 0.00


def success(objects: Dict[str, Any]) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# -- Scripted Policy Waypoints -----------------------------------------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    variant = os.environ.get("DEMO_VARIANT", "")
    plans: dict[str, list[Waypoint]] = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}

    LEFT_PARK = (0.160, 0.280, 0.930)
    RIGHT_PARK = (0.160, -0.280, 0.930)

    def add(arm: str, dt: float, pos, finger: float, label: str = "", yaw: float = 0.0) -> None:
        plans[arm].append(Waypoint(float(dt), tuple(float(v) for v in pos), float(finger), label, float(yaw)))
        clock[arm] += float(dt)

    def wait_until(arm: str, t: float, label: str = "wait") -> None:
        dt = t - clock[arm]
        if dt > 1e-6:
            pos = plans[arm][-1].pos if plans[arm] else (LEFT_PARK if arm == "left" else RIGHT_PARK)
            finger = plans[arm][-1].finger if plans[arm] else FINGER_OPEN
            yaw = plans[arm][-1].yaw if plans[arm] else 0.0
            add(arm, dt, pos, finger, label, yaw)

    # Variant: premature_degas (Depress DEGAS button while bath is empty -> cavitation defect 0.0)
    if variant == "premature_degas":
        wait_until("left", 8.0, "idle")
        add("right", 1.2, (0.435, BTN_Y, BTN_Z), FINGER_CLOSED, "approach DEGAS button")
        add("right", 0.8, (BTN_REST_X + 0.015, BTN_Y, BTN_Z), FINGER_CLOSED, "align with button face")
        add("right", 0.8, (BTN_REST_X - 0.007, BTN_Y, BTN_Z), FINGER_CLOSED, "press DEGAS button on empty tank")
        add("right", 1.0, (BTN_REST_X - 0.007, BTN_Y, BTN_Z), FINGER_CLOSED, "hold button in defect pose")
        return plans

    # Phase 1: Left arm retrieves floating microtube rack from staging area
    # Staging pose: X = 0.250, Y = 0.180, Z = 0.7625 m
    add("left", 1.2, (RACK_START_X, RACK_START_Y, 0.880), FINGER_OPEN, "hover above foam rack")
    add("left", 0.8, (RACK_START_X, RACK_START_Y, 0.780), FINGER_OPEN, "descend onto rack grip perimeter")
    add("left", 0.6, (RACK_START_X, RACK_START_Y, 0.780), FINGER_CLOSED, "grasp foam rack securely")
    add("left", 0.4, (RACK_START_X, RACK_START_Y, 0.780), FINGER_CLOSED, "settle grasp")

    # Lift vertically to clear staging perimeter
    add("left", 0.8, (RACK_START_X, RACK_START_Y, 0.890), FINGER_CLOSED, "lift foam rack clear")

    # Transfer laterally across Y axis into ultrasonic bath tank center
    # Tank center: X = 0.250, Y = -0.060
    add("left", 1.2, (TANK_CENTER_X, 0.060, 0.890), FINGER_CLOSED, "transfer across Y midpoint")
    add("left", 1.0, (TANK_CENTER_X, TANK_CENTER_Y, 0.890), FINGER_CLOSED, "align over tank center")

    # Descend level into tank cavity onto resting water bed
    # Target rack Z center is 0.828 m -> TCP grip at 0.835 m
    add("left", 1.0, (TANK_CENTER_X, TANK_CENTER_Y, 0.835), FINGER_CLOSED, "seat foam rack inside tank")
    add("left", 0.5, (TANK_CENTER_X, TANK_CENTER_Y, 0.835), FINGER_CLOSED, "settle rack on bath floor")

    # Release rack
    add("left", 0.5, (TANK_CENTER_X, TANK_CENTER_Y, 0.835), FINGER_OPEN, "release foam rack")
    add("left", 0.8, (TANK_CENTER_X, TANK_CENTER_Y, 0.920), FINGER_OPEN, "retract left arm vertically")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Phase 2: Right arm closes hinged acrylic bath lid
    # Lid hinge at (LID_HINGE_X, LID_HINGE_Y, LID_HINGE_Z). Propped open at 70 deg.
    # Front handle tip is at X ~ 0.227, Y = -0.060, Z ~ 1.003 m
    wait_until("right", clock["left"] - 2.0, "wait until left arm clears tank well")
    add("right", 1.2, (0.227, LID_HINGE_Y, 1.030), FINGER_OPEN, "approach above raised lid handle")
    add("right", 0.8, (0.227, LID_HINGE_Y, 1.000), FINGER_OPEN, "align with handle bar")
    add("right", 0.6, (0.227, LID_HINGE_Y, 1.000), FINGER_CLOSED, "grasp lid handle")

    # Lower lid through arc to seat flush over tank rim
    add("right", 0.6, (0.260, LID_HINGE_Y, 0.975), FINGER_CLOSED, "lower lid arc step 1 (50 deg)")
    add("right", 0.6, (0.300, LID_HINGE_Y, 0.935), FINGER_CLOSED, "lower lid arc step 2 (30 deg)")
    add("right", 0.6, (0.330, LID_HINGE_Y, 0.885), FINGER_CLOSED, "lower lid arc step 3 (15 deg)")
    add("right", 0.6, (0.345, LID_HINGE_Y, 0.840), FINGER_CLOSED, "seat lid flush onto tank rim")
    add("right", 0.4, (0.345, LID_HINGE_Y, 0.840), FINGER_OPEN, "release lid handle")

    # Phase 3: Right arm moves to front control panel and depresses DEGAS button inward
    add("right", 1.2, (0.440, BTN_Y, BTN_Z), FINGER_CLOSED, "approach DEGAS button")
    add("right", 0.8, (BTN_REST_X + 0.015, BTN_Y, BTN_Z), FINGER_CLOSED, "align with button face")
    add("right", 0.8, (BTN_REST_X - 0.016, BTN_Y, BTN_Z), FINGER_CLOSED, "depress DEGAS button inward")
    add("right", 0.8, (BTN_REST_X - 0.016, BTN_Y, BTN_Z), FINGER_CLOSED, "hold DEGAS button depressed")

    # Variant: no_release (keep holding button)
    if variant == "no_release":
        end_t = max(clock.values()) + 3.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "hold unreleased")
        return plans

    # Phase 4: Release DEGAS button and park right arm clear
    add("right", 0.6, (0.450, BTN_Y, BTN_Z), FINGER_OPEN, "retract from DEGAS button")
    add("right", 0.6, (0.450, BTN_Y, 0.950), FINGER_OPEN, "elevate right arm clear of lid")
    add("right", 1.0, RIGHT_PARK, FINGER_OPEN, "park right arm")

    # Stillness hold dwell (20 steps = 1.0 s)
    end_t = max(clock.values()) + 2.0
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness dwell")

    return plans
