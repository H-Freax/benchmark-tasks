"""Clamp a split-core current transformer around an insulated conductor and verify latch lock.

On the station table: an industrial power submetering installation setup features a heavy 350 kcmil
insulated copper feeder cable (22 mm OD, 260 mm length, oriented along the Y axis) rigidly mounted
on two steel terminal standoffs (30 x 40 x 78 mm) at y = -0.110 m and y = +0.110 m, centered at
x 32 cm, z 82.75 cm (80 mm above tabletop). A standard commercial split-core current transformer
(Magnelab SCT / CR Magnetics style, outer envelope 56 x 60 x 24 mm with a 24 x 24 mm central aperture)
rests positioned on the feeder cable at the center witness mark (y = 0.000 m). Its upper core keeper
is held open at 45 degrees by an internal torsion spring, allowing free positioning along the cable.

The robot steadies the lower U-core housing around the conductor, compresses the hinged upper keeper
down flush against the lower core faces (air gap < 0.5 mm) until the mechanical retention catch clicks
locked, and clears all tools. The workstation then applies an active 15 N verification tug along the
cable for 0.5 s; a properly latched CT maintains core closure and mechanical retention without unseating.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/clamp-split-core-current-transformer/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 --max-steps 600 \
  --instruction "Clamp the split-core current transformer around the insulated conductor until the hinged keeper clicks shut and locked, then clear both arms." \
  --scorer task_score ...
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import mechkit as mk
from yam_arena.demo import Waypoint
from yam_arena.task_file import GREY, TABLE_TOP, Box, ObjectState

TAG = "clamp_split_core_current_transformer"
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Clamp the split-core current transformer around the insulated conductor"
    " until the hinged keeper clicks shut and locked, then clear both arms."
)

# Colors
COPPER_INSULATED = (0.15, 0.16, 0.18)     # 350 kcmil black THHN insulation
STANDOFF_STEEL = (0.65, 0.67, 0.70)       # zinc-plated steel terminal standoffs
CT_BODY_DARK = (0.22, 0.23, 0.25)         # industrial ABS dark charcoal housing
KEEPER_DARK = (0.28, 0.29, 0.32)          # hinged upper core keeper
LATCH_YELLOW = (0.92, 0.75, 0.15)         # high-visibility snap catch tongue
ZONE_MARKER = (0.85, 0.85, 0.30)          # alignment witness mark on cable

# -- Geometry Definitions --------------------------------------------------------------------------------------

# Cable: 350 kcmil THHN (OD = 22 mm), 260 mm length along Y
CABLE_D = 0.022
CABLE_L = 0.260
CABLE_X = 0.320
CABLE_Z = TOP + 0.080  # 80 mm above tabletop (z = 0.8275 m)

# Standoffs supporting cable ends
STANDOFF_W = 0.030
STANDOFF_L = 0.040
STANDOFF_H = 0.078
STANDOFF_Y_L = -0.110
STANDOFF_Y_R = 0.110

# Commercial Split-Core Current Transformer:
# Overall envelope: 56 mm wide (X: -28 to +28 mm), 24 mm thick (Y: -12 to +12 mm), 62 mm high (Z: -35 to +27 mm)
# Window aperture: 24 x 24 mm square aperture enclosing the 22 mm OD cable
CT_WIDTH = 0.056
CT_THICK = 0.024
CT_HEIGHT = 0.062

CT_INIT_Y = 0.000  # positioned at witness mark on cable
CT_GOAL_Y = 0.000   # center measurement position

kit = mk.Kit()

# 1. Feeder Cable & Standoffs (rigidly fixed to table)
kit.part("standoff_left", (CABLE_X, STANDOFF_Y_L, TOP + STANDOFF_H / 2), [
    mk.box((STANDOFF_W, STANDOFF_L, STANDOFF_H), color=STANDOFF_STEEL)
], mass=None, color=STANDOFF_STEEL)

kit.part("standoff_right", (CABLE_X, STANDOFF_Y_R, TOP + STANDOFF_H / 2), [
    mk.box((STANDOFF_W, STANDOFF_L, STANDOFF_H), color=STANDOFF_STEEL)
], mass=None, color=STANDOFF_STEEL)

kit.part("feeder_cable", (CABLE_X, 0.000, CABLE_Z), [
    mk.box((CABLE_D, CABLE_L, CABLE_D), color=COPPER_INSULATED),
    # Yellow witness target mark at cable center (width 12 mm)
    mk.box((CABLE_D + 0.002, 0.012, CABLE_D + 0.002), at=(0.0, 0.0, 0.0), color=ZONE_MARKER)
], mass=None, color=COPPER_INSULATED)

# 2. Lower U-Core Body (`ct_base`): slides along cable in Y (-0.080 to +0.080 m)
# Geometry forms a U-cradle under the cable:
BASE_SPINE_Z = -0.023
LEG_Z = -0.002
LEG_H = 0.028
LEG_T = 0.012

kit.part("ct_base", (CABLE_X, 0.000, CABLE_Z), [
    # Bottom spine bridging both legs
    mk.box((CT_WIDTH, CT_THICK, 0.016), at=(0.0, 0.0, BASE_SPINE_Z), color=CT_BODY_DARK),
    # Rear leg (hinge anchor): at x = -0.020
    mk.box((LEG_T, CT_THICK, LEG_H), at=(-0.020, 0.0, LEG_Z), color=CT_BODY_DARK),
    # Front leg (latch receiver): at x = +0.020, with keeper snap tongue
    mk.box((LEG_T, CT_THICK, LEG_H), at=(0.020, 0.0, LEG_Z), color=CT_BODY_DARK),
    mk.box((0.006, 0.016, 0.008), at=(0.026, 0.0, LEG_Z + LEG_H / 2 + 0.004), color=LATCH_YELLOW),
], mass=0.180, color=CT_BODY_DARK)

# Prismatic slider along Y representing sliding fit along the cable
kit.slider("ct_slide", "feeder_cable", "ct_base", axis=(0, 1, 0),
           lower=-0.080, upper=0.080, start=CT_INIT_Y, damping=0.5, friction=0.2)

# 3. Upper Hinged Core Keeper (`ct_keeper`)
# Revolute hinge on rear leg top (x = -0.020, z = +0.014).
# 0 deg = closed horizontal; -45 deg = hinged open upward/backward.
KEEPER_HINGE_AT = (CABLE_X - 0.020, 0.000, CABLE_Z + 0.014)

kit.part("ct_keeper", (CABLE_X, 0.000, CABLE_Z + 0.014), [
    # Top bridging keeper bar
    mk.box((0.046, CT_THICK, 0.012), at=(0.017, 0.0, 0.006), color=KEEPER_DARK),
    # Snap catch engagement tab on front end
    mk.box((0.010, 0.020, 0.016), at=(0.038, 0.0, 0.004), color=LATCH_YELLOW),
], mass=0.070, color=KEEPER_DARK)

kit.hinge("core_hinge", "ct_base", "ct_keeper",
          anchor=KEEPER_HINGE_AT, axis=(0, 1, 0),
          lower_deg=-45.0, upper_deg=0.0, start_deg=-45.0,
          stiffness=0.0, rest_deg=-45.0, damping=0.1, friction=0.060)

# Positive retention catch holding keeper locked closed once engaged
# _REF = kit._fk(q={"core_hinge": 0.0})

OBJECTS = [*kit.objects()]

# -- Thresholds and Tolerances ---------------------------------------------------------------------------------
ALIGN_Y_TOL = 0.020     # 20 mm tolerance to witness mark at cable center (y = 0.000)
KEEPER_CLOSED_DEG = 5.0 # <= 5.0 deg from fully seated contact
PROOF_FORCE_N = 15.0    # 15 N verification tug along cable axis
PROOF_STEPS = 10        # 0.5 s at 20 Hz
CLEAR_MARGIN = 0.020    # 20 mm clearance to all robot collision hulls
STILL_SPEED = 0.040     # 4 cm/s max speed for settled CT
STILL_SPIN = 1.50       # 1.5 rad/s max spin

_TEST_ROBOT_BOXES = None


def _finite(st) -> bool:
    try:
        p = np.asarray(st.pos, dtype=float)
        q = np.asarray(st.quat_xyzw, dtype=float)
        return p.shape == (3,) and q.shape == (4,) and np.isfinite(p).all() and np.isfinite(q).all()
    except Exception:
        return False


def ct_y_displacement(objects) -> float:
    return float(kit.q(objects, "ct_slide"))


def keeper_angle_deg(objects) -> float:
    # lower is -45, 0 is closed flat
    return abs(float(kit.q_deg(objects, "core_hinge")))


def ct_aligned(objects) -> bool:
    if not _finite(objects["ct_base"]):
        return False
    return abs(ct_y_displacement(objects) - CT_GOAL_Y) <= ALIGN_Y_TOL


def core_closed(objects) -> bool:
    if not _finite(objects["ct_keeper"]):
        return False
    return keeper_angle_deg(objects) <= KEEPER_CLOSED_DEG


def hands_off(objects) -> bool:
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES
    for name in ("ct_base", "ct_keeper", "feeder_cable"):
        st = objects[name]
        lo_s, hi_s = np.asarray(st.low), np.asarray(st.high)
        for lo_r, hi_r in boxes:
            lo_r, hi_r = np.asarray(lo_r), np.asarray(hi_r)
            d = np.maximum(np.maximum(lo_s - hi_r, lo_r - hi_s), 0.0)
            if float(np.linalg.norm(d)) < CLEAR_MARGIN:
                return False
    return True


def _rule(objects):
    M = mk.MEMORY
    M["n"] = M.get("n", 0) + 1

    st_base = objects["ct_base"]
    if _finite(st_base) and M["n"] % 20 == 1:
        y_pos = ct_y_displacement(objects)
        deg = keeper_angle_deg(objects)
        print(f"[_rule n={M['n']}] ct_y={y_pos*1000:.1f}mm keeper={deg:.1f}deg still_n={M.get('still_n')} proof_n={M.get('proof_n')} passed={M.get('proof_passed')}", flush=True)

    # Mechanical retention catch:
    # Requires keeper driven shut flush (<= 5.0 deg) by active robot contact to trip the retention latch
    if core_closed(objects) and not hands_off(objects) and not M.get("latch_engaged"):
        M["latch_engaged"] = True
        kit.drive("core_hinge", 0.0)

    # Active 15 N verification tug along cable axis:
    # When clamped and arms are clear, apply 15 N proof tug to verify the clamped CT maintains mechanical integrity
    if M.get("latch_engaged") and hands_off(objects) and core_closed(objects):
        still_n = M.get("still_n", 0) + 1
        M["still_n"] = still_n
        if still_n >= 6:  # 0.3 s settled
            proof_n = M.get("proof_n", 0) + 1
            M["proof_n"] = proof_n
            if proof_n <= PROOF_STEPS:
                kit.pull("ct_base", force=(0.0, PROOF_FORCE_N, 0.0), at=(CABLE_X, 0.000, CABLE_Z))
            else:
                kit.pull("ct_base")
                # Retention proof passes if CT maintains closure under the proof load
                if core_closed(objects) and not M.get("fell"):
                    M["proof_passed"] = True
    else:
        if M.get("proof_n"):
            kit.pull("ct_base")
        M["still_n"] = 0

    # Fall off bench check
    if min(float(objects[p].pos[2]) for p in ("ct_base", "ct_keeper")) < TOP - 0.050:
        M["fell"] = True


kit.on_step(_rule)


def score(objects) -> float:
    M = mk.MEMORY
    if M.get("fell"):
        return 0.0

    st_base = objects["ct_base"]
    base_v = float(np.linalg.norm(st_base.lin_vel))
    still = (base_v <= STILL_SPEED)

    # Stage 4 (1.00): Core clamped, proof pull passed, arms clear >= 20 mm, held at rest
    if (M.get("proof_passed") and M.get("latch_engaged")
            and core_closed(objects) and still and hands_off(objects)):
        return 1.00

    # Stage 3 (0.75): Retention catch engaged and locked
    if M.get("latch_engaged") and core_closed(objects) and ct_aligned(objects):
        return 0.75

    # Stage 2 (0.50): Upper keeper swung shut flush against lower core face (<= 5.0 deg)
    if core_closed(objects) and ct_aligned(objects):
        return 0.50

    # Stage 1 (0.25): Keeper partially depressed past halfway mark (<= 30.0 deg)
    if keeper_angle_deg(objects) <= 30.0 and ct_aligned(objects):
        return 0.25

    return 0.00


def success(objects) -> bool:
    return score(objects) >= 1.00 - 1e-6


# -- Demo trajectory plans -------------------------------------------------------------------------------------

PARK_L = (0.150, 0.250, TOP + 0.150)
PARK_R = (0.150, -0.250, TOP + 0.150)

FINGER_OPEN = 0.040
FINGER_CLOSED = 0.004


def demo():
    kind = os.environ.get("CT_DEMO", "")

    if kind == "noop":
        return {
            "left": [
                Waypoint(1.0, PARK_L, FINGER_OPEN, "idle_park_left"),
                Waypoint(14.0, PARK_L, FINGER_OPEN, "idle_park_left")
            ],
            "right": [
                Waypoint(1.0, PARK_R, FINGER_OPEN, "idle_park_right"),
                Waypoint(14.0, PARK_R, FINGER_OPEN, "idle_park_right")
            ]
        }

    if kind == "slide_only":
        # Negative control: steady base only, no keeper press
        return {
            "right": [
                Waypoint(1.5, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "hover_beside_base"),
                Waypoint(1.0, (CABLE_X, -0.030, CABLE_Z), FINGER_OPEN, "descend_beside_base"),
                Waypoint(0.5, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "contact_side_flange"),
                Waypoint(4.0, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "steady_base"),
                Waypoint(1.0, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "retract_side"),
                Waypoint(1.5, PARK_R, FINGER_OPEN, "clear_right"),
                Waypoint(5.5, PARK_R, FINGER_OPEN, "park_right")
            ],
            "left": [
                Waypoint(1.0, PARK_L, FINGER_OPEN, "idle_left"),
                Waypoint(14.0, PARK_L, FINGER_OPEN, "park_left")
            ]
        }

    if kind == "close_no_latch":
        # Negative control: shallow keeper press (25 deg) yielding 0.25 partial stage credit
        return {
            "right": [
                Waypoint(1.5, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "hover_beside_base"),
                Waypoint(1.0, (CABLE_X, -0.030, CABLE_Z), FINGER_OPEN, "descend_beside_base"),
                Waypoint(0.5, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "contact_side_flange"),
                Waypoint(5.0, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "steady_base"),
                Waypoint(1.0, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "retract_side"),
                Waypoint(1.5, PARK_R, FINGER_OPEN, "clear_right"),
                Waypoint(4.5, PARK_R, FINGER_OPEN, "park_right")
            ],
            "left": [
                Waypoint(2.0, (0.335, 0.000, 0.880), FINGER_OPEN, "approach_above_keeper"),
                Waypoint(0.5, (0.335, 0.000, 0.880), FINGER_CLOSED, "close_gripper_pad"),
                Waypoint(2.0, (0.340, 0.000, 0.865), FINGER_CLOSED, "shallow_press"),
                Waypoint(1.0, (0.335, 0.000, 0.880), FINGER_OPEN, "retract_up"),
                Waypoint(1.5, PARK_L, FINGER_OPEN, "clear_left"),
                Waypoint(8.0, PARK_L, FINGER_OPEN, "park_left")
            ]
        }

    # Coordinated bimanual operation:
    # Phase 1 (0-3.0s): Right arm descends beside CT base from -Y; Left arm descends above keeper from +Z.
    # Phase 2 (3.0-8.0s): Right arm steadies base; Left arm compresses keeper flush shut until catch clicks locked.
    # Phase 3 (8.0-15.0s): Both arms retract to park poses (>= 20 mm clear) and hold for active 15 N verification tug.
    right_plan = [
        Waypoint(1.5, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "hover_beside_base"),
        Waypoint(1.0, (CABLE_X, -0.030, CABLE_Z), FINGER_OPEN, "descend_beside_base"),
        Waypoint(0.5, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "contact_side_flange"),
        Waypoint(5.0, (CABLE_X, -0.022, CABLE_Z), FINGER_CLOSED, "steady_base_during_clamping"),
        Waypoint(1.0, (CABLE_X, -0.040, CABLE_Z + 0.040), FINGER_OPEN, "retract_side"),
        Waypoint(1.5, PARK_R, FINGER_OPEN, "clear_right"),
        Waypoint(4.5, PARK_R, FINGER_OPEN, "proof_hold_right")
    ]

    left_plan = [
        Waypoint(2.0, (0.335, 0.000, 0.880), FINGER_OPEN, "approach_above_keeper"),
        Waypoint(0.5, (0.335, 0.000, 0.880), FINGER_CLOSED, "close_gripper_pad"),
        Waypoint(2.5, (0.355, 0.000, 0.770), FINGER_CLOSED, "press_keeper_shut"),
        Waypoint(1.5, (0.355, 0.000, 0.770), FINGER_CLOSED, "hold_keeper_shut"),
        Waypoint(1.0, (0.335, 0.000, 0.880), FINGER_OPEN, "retract_up"),
        Waypoint(1.5, PARK_L, FINGER_OPEN, "clear_left"),
        Waypoint(6.0, PARK_L, FINGER_OPEN, "proof_hold_left")
    ]

    return {"left": left_plan, "right": right_plan}


# -- Offline Self-Test -----------------------------------------------------------------------------------------

def self_test():
    global _TEST_ROBOT_BOXES

    far_boxes = [((-1.0, -1.0, -1.0), (-0.9, -0.9, -0.9))]
    near_boxes = [((CABLE_X - 0.010, -0.010, CABLE_Z - 0.010), (CABLE_X + 0.010, 0.010, CABLE_Z + 0.010))]

    st_init = kit.fake_states(q={"ct_slide": CT_INIT_Y, "core_hinge": -45.0}, deg=True)
    st_half = kit.fake_states(q={"ct_slide": CT_GOAL_Y, "core_hinge": -25.0}, deg=True)
    st_closed = kit.fake_states(q={"ct_slide": CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
    st_fell = kit.fake_states(
        q={"ct_slide": CT_GOAL_Y, "core_hinge": 0.0}, deg=True,
        moved={"ct_base": ((CABLE_X, 0.0, TOP - 0.100), (0, 0, 0, 1))}
    )

    cases = {
        "initial_state": (st_init, far_boxes, {}, 0.00),
        "keeper_half_depressed": (st_half, far_boxes, {}, 0.25),
        "core_closed": (st_closed, far_boxes, {}, 0.50),
        "latch_engaged": (st_closed, far_boxes, {"latch_engaged": True}, 0.75),
        "full_success": (st_closed, far_boxes, {"latch_engaged": True, "proof_passed": True}, 1.00),
        "hand_intrusion": (st_closed, near_boxes, {"latch_engaged": True, "proof_passed": True}, 0.75),
        "part_fell": (st_fell, far_boxes, {"fell": True}, 0.00),
    }

    for name, (states, boxes, mem_overrides, expected) in cases.items():
        _TEST_ROBOT_BOXES = boxes
        mk.MEMORY.clear()
        mk.MEMORY.update(mem_overrides)
        s = score(states)
        assert abs(s - expected) < 1e-6, f"self_test case '{name}' failed: expected {expected}, got {s}"

    print(f"Self-test: {len(cases)} scoring cases passed successfully!")

    # Verify plan durations
    for variant in ("demo", "slide_only", "close_no_latch", "noop"):
        os.environ["CT_DEMO"] = variant if variant != "demo" else ""
        plan = demo()
        t_l = sum(w.duration for w in plan["left"])
        t_r = sum(w.duration for w in plan["right"])
        assert abs(t_l - t_r) < 1e-4, f"Variant '{variant}' duration mismatch: left {t_l}s != right {t_r}s"


if __name__ == "__main__":
    self_test()
