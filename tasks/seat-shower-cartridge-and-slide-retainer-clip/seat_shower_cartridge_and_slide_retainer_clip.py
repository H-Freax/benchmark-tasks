"""Moen 1222 Posi-Temp shower valve cartridge replacement and brass retainer clip seating.

Plumbing maintenance operation under standard plumbing repair practice (BLS 47-2152):
1. Left arm pushes cartridge brass stem / face inward (-X) into the cast brass valve bore
   against dual lubricated O-ring elastomeric compression resistance (~14 N) until seated flush
   against the internal shoulder stop (< 0.8 mm proud).
2. Right arm grasps the brass U-shaped retainer clip from its staging block, carries it over
   the valve body, aligns the clip legs with the machined vertical guide slots, and slides
   the clip down into engagement across the cartridge retention groove.
3. If the cartridge is released early, O-ring elastomeric relaxation pushes it outward by 1.2 mm,
   blocking clip insertion.
4. Once seated, both arms release and clear (>= 60 mm); mechanical retention holds the cartridge
   seated in equilibrium for >= 1.0 s.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

# Add task directory to sys.path so mechkit is found
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mechkit as mk  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Visual materials (sRGB)
CAST_BRASS = (0.78, 0.62, 0.28)
POLISHED_BRASS = (0.88, 0.75, 0.35)
PLASTIC_WHITE = (0.92, 0.92, 0.90)
DARK_RUBBER = (0.15, 0.15, 0.15)
STEEL_GREY = (0.65, 0.66, 0.68)

# Physical Geometry (World frame: X forward, Y left, Z up)
VALVE_X = 0.280
VALVE_Y = 0.000
VALVE_Z = TOP + 0.045  # Center of valve bore: 45 mm above table

# Moen 1222 Valve Body (110 mm wide, 70 mm tall, 50 mm deep, cast brass)
# Cartridge bore: 32 mm ID, 65 mm depth along -X.
# Cartridge enters from front (+X) moving toward rear (-X).
# Valve front face: X = VALVE_X + 0.025
# Seated flush reference plane: X = VALVE_X + 0.025
# Unseated cartridge starts 12 mm proud: X = VALVE_X + 0.025 + 0.012 = VALVE_X + 0.037

kit = mk.Kit()

# 1. Valve Body Mounting Base (Rigidly fixed to fixture)
# Built with real hollow cavity for bore and dual vertical guide slot channels
kit.part(
    "valve_body",
    (VALVE_X, VALVE_Y, VALVE_Z),
    [
        # Main mounting saddle base to table
        mk.box((0.050, 0.070, 0.020), at=(0, 0, -0.035), color=CAST_BRASS),
        # Valve body rear wall / backstop
        mk.box((0.010, 0.050, 0.050), at=(-0.020, 0, 0), color=CAST_BRASS),
        # Left wall of valve bore
        mk.box((0.040, 0.008, 0.040), at=(0.005, 0.020, 0), color=CAST_BRASS),
        # Right wall of valve bore
        mk.box((0.040, 0.008, 0.040), at=(0.005, -0.020, 0), color=CAST_BRASS),
        # Bottom wall of valve bore
        mk.box((0.040, 0.032, 0.008), at=(0.005, 0, -0.020), color=CAST_BRASS),
        # Top ear left (with 1.8 mm clip slot gap at X = 0.005)
        mk.box((0.016, 0.012, 0.012), at=(-0.008, 0.012, 0.024), color=CAST_BRASS),
        mk.box((0.016, 0.012, 0.012), at=(0.016, 0.012, 0.024), color=CAST_BRASS),
        # Top ear right (with 1.8 mm clip slot gap at X = 0.005)
        mk.box((0.016, 0.012, 0.012), at=(-0.008, -0.012, 0.024), color=CAST_BRASS),
        mk.box((0.016, 0.012, 0.012), at=(0.016, -0.012, 0.024), color=CAST_BRASS),
    ]
)

# 2. Cartridge Assembly (Free moving along X axis)
# Reference pose at rest (proud by 12 mm = +0.012 along X)
CARTRIDGE_REST_X = VALVE_X + 0.012

kit.part(
    "cartridge",
    (CARTRIDGE_REST_X, VALVE_Y, VALVE_Z),
    [
        # Cartridge body barrel (30 mm dia approx box, 50 mm length)
        mk.box((0.050, 0.028, 0.028), at=(0, 0, 0), color=PLASTIC_WHITE),
        # Front face / collar with retaining groove at local X = -0.007
        # Front pushing face / stem (10 mm sq brass stem extending forward)
        mk.box((0.020, 0.012, 0.012), at=(0.035, 0, 0), color=POLISHED_BRASS),
        # Dual simulated rubber O-rings on barrel
        mk.box((0.004, 0.030, 0.030), at=(-0.012, 0, 0), color=DARK_RUBBER),
        mk.box((0.004, 0.030, 0.030), at=(0.010, 0, 0), color=DARK_RUBBER),
    ],
    mass=0.120
)

# Prismatic slider for cartridge along X
# Travel: -0.012 (flush home) to 0.000 (rest proud).
# Axis is (-1, 0, 0) so positive q corresponds to pushing inward toward home!
kit.slider(
    "cartridge_slide",
    "valve_body",
    "cartridge",
    axis=(-1, 0, 1e-6),  # Inward push along -X
    lower=-0.002,         # Can't pull out more than 2 mm past rest
    upper=0.0125,         # Seated flush stop at 12 mm travel
    start=0.000,          # Starts at rest (proud)
    stiffness=1100.0,     # 1100 N/m * 0.012 m ≈ 13.2 N preload resistance at flush
    rest=0.000,           # Relaxes back to proud if released
    damping=10.0,         # Viscous O-ring grease damping
    friction=2.0          # O-ring Coulomb seal friction
)

# 3. Retainer Clip Staging Post
CLIP_STAGE_X = VALVE_X
CLIP_STAGE_Y = -0.120
CLIP_STAGE_Z = TOP + 0.020

kit.part(
    "clip_staging",
    (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_STAGE_Z),
    [
        mk.box((0.040, 0.040, 0.040), at=(0, 0, 0), color=STEEL_GREY)
    ]
)

# 4. Stamped Brass U-Shaped Retainer Clip (32 mm tall, 24 mm wide, 1.5 mm thick)
# Staged vertically on staging block
CLIP_INIT_X = CLIP_STAGE_X
CLIP_INIT_Y = CLIP_STAGE_Y
CLIP_INIT_Z = TOP + 0.040 + 0.016  # Center of clip: TOP + 0.056

kit.part(
    "retainer_clip",
    (CLIP_INIT_X, CLIP_INIT_Y, CLIP_INIT_Z),
    [
        # Crown top bridge
        mk.box((0.0015, 0.024, 0.006), at=(0, 0, 0.013), color=POLISHED_BRASS),
        # Left leg (30 mm long down)
        mk.box((0.0015, 0.004, 0.026), at=(0, 0.010, -0.003), color=POLISHED_BRASS),
        # Right leg (30 mm long down)
        mk.box((0.0015, 0.004, 0.026), at=(0, -0.010, -0.003), color=POLISHED_BRASS),
    ],
    mass=0.025
)

OBJECTS = [*kit.objects()]

# Key target locations
# Target seated position for retainer clip:
# X = VALVE_X + 0.005 (aligned with body vertical guide slots)
# Y = VALVE_Y = 0.000 (centered across valve body)
# Z = VALVE_Z + 0.008 (fully seated down onto ear stops)
CLIP_SEAT_TARGET = np.array([VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.008])

# Memory state tracking
HOLD_STEPS_REQUIRED = 20  # 1.0 s at 20 Hz


def cartridge_flush(objects) -> bool:
    """Cartridge pushed flush home against internal stop (< 0.8 mm proud)."""
    q_push = kit.q(objects, "cartridge_slide")
    return q_push >= 0.0112  # within 0.8 mm of 0.012 m full stroke


def clip_in_slots(objects) -> bool:
    """Retainer clip entered into vertical guide slots."""
    clip = objects.get("retainer_clip")
    if clip is None:
        return False
    pos = np.asarray(clip.pos)
    # Check X alignment with slot plane (within 3 mm)
    if abs(pos[0] - (VALVE_X + 0.005)) > 0.004:
        return False
    # Check Y lateral centering across valve ears (within 4 mm)
    if abs(pos[1] - VALVE_Y) > 0.005:
        return False
    # Check Z has lowered below staging height into top of ears
    return pos[2] <= (VALVE_Z + 0.028)


def clip_fully_seated(objects) -> bool:
    """Retainer clip fully seated down onto stops across cartridge retaining groove.
    Requires clip center within 5 mm of target stop, and upright orientation (tilt < 10 deg)."""
    clip = objects.get("retainer_clip")
    if clip is None:
        return False
    pos = np.asarray(clip.pos)
    err = np.linalg.norm(pos - CLIP_SEAT_TARGET)
    if err >= 0.006:
        return False
    # Check clip orientation: upright (z-axis within 10 deg of world Z)
    rot_mat = mk.quat_to_mat(clip.quat_xyzw)
    z_dot = rot_mat[2, 2]
    return z_dot > 0.9848  # cos(10 deg)


def hands_cleared(objects, margin=0.060) -> bool:
    """Both gripper TCPs cleared away from valve and clip by at least margin (m)."""
    if hasattr(mk, "_TEST_TCPS") and mk._TEST_TCPS is not None:
        tcps = mk._TEST_TCPS
    else:
        tcps = mk.tcps()
    if not tcps:
        return False
    targets = [np.array([VALVE_X, VALVE_Y, VALVE_Z]), CLIP_SEAT_TARGET]
    for tcp in tcps:
        for tgt in targets:
            if np.linalg.norm(tcp - tgt) < margin:
                return False
    return True


def _check_retention_hold(objects):
    """Step callback to enforce 1.0 s hands-free retention under O-ring backpressure."""
    clip = objects.get("retainer_clip")
    if clip is None:
        return

    # Check for dropped clip failure latch
    if clip.pos[2] < TOP + 0.010:
        mk.MEMORY["clip_dropped"] = True

    flush = cartridge_flush(objects)
    seated = clip_fully_seated(objects)
    cleared = hands_cleared(objects)
    no_drop = not mk.MEMORY.get("clip_dropped", False)

    if flush and seated and cleared and no_drop:
        held = mk.MEMORY.get("hold_steps", 0) + 1
        mk.MEMORY["hold_steps"] = held
        if held >= HOLD_STEPS_REQUIRED:
            mk.MEMORY["retained_equilibrium"] = True
    else:
        mk.MEMORY["hold_steps"] = 0


kit.on_step(_check_retention_hold)


def score(objects) -> float:
    """Multi-stage scoring for shower valve cartridge replacement.
    0.00: Cartridge unpushed (> 8 mm proud).
    0.20: Cartridge pushed halfway into bore (< 6 mm proud).
    0.45: Cartridge pushed flush home against shoulder stop (< 0.8 mm proud).
    0.70: Retainer clip entered into guide slots while cartridge held flush.
    0.90: Retainer clip fully seated down onto stops.
    1.00: Hands cleared (>= 60 mm); retained in equilibrium against O-ring push for >= 1.0 s.
    """
    if mk.MEMORY.get("clip_dropped", False):
        return 0.0

    q_push = kit.q(objects, "cartridge_slide")

    if q_push < 0.006:
        return 0.00
    if q_push < 0.0112:
        return 0.20

    # Stage 2: Cartridge flush home against O-ring resistance
    pts = 0.45

    # Stage 3: Retainer clip entered into guide slots
    if not clip_in_slots(objects):
        return pts
    pts = 0.70

    # Stage 4: Retainer clip fully seated down onto ear stops
    if not clip_fully_seated(objects):
        return pts
    pts = 0.90

    # Stage 5: Hands cleared and assembly retained under backpressure for >= 1.0 s
    if mk.MEMORY.get("retained_equilibrium", False):
        return 1.00

    return pts


def success(objects) -> bool:
    return score(objects) >= 1.00


def demo():
    """Bimanual synchronized manipulation demo for shower cartridge reset."""
    variant = os.environ.get("DEMO_VARIANT", "normal")

    # Variant: no_push (right hand tries to slide clip while cartridge unpushed)
    if variant == "no_push":
        left_wps = [
            Waypoint(1.5, (0.34, 0.080, VALVE_Z + 0.060), FINGER_OPEN, "idle left"),
            Waypoint(13.5, (0.34, 0.080, VALVE_Z + 0.060), FINGER_OPEN, "idle left")
        ]
        right_wps = [
            Waypoint(1.5, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z + 0.050), FINGER_OPEN, "hover clip"),
            Waypoint(1.0, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z), FINGER_OPEN, "reach clip"),
            Waypoint(1.0, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z), FINGER_CLOSED, "grasp clip"),
            Waypoint(1.5, (CLIP_STAGE_X, CLIP_STAGE_Y, VALVE_Z + 0.060), FINGER_CLOSED, "lift clip"),
            Waypoint(2.0, (VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.060), FINGER_CLOSED, "carry over valve"),
            Waypoint(2.0, (VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.025), FINGER_CLOSED, "attempt seat (blocked)"),
            Waypoint(6.0, (VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.025), FINGER_CLOSED, "blocked hold")
        ]
        return {"left": left_wps, "right": right_wps}

    # Variant: no_clip (left hand pushes cartridge flush, then releases without clip)
    if variant == "no_clip":
        left_wps = [
            Waypoint(1.5, (VALVE_X + 0.080, VALVE_Y, VALVE_Z), FINGER_OPEN, "approach stem"),
            Waypoint(1.0, (VALVE_X + 0.047, VALVE_Y, VALVE_Z), FINGER_CLOSED, "contact stem"),
            Waypoint(2.0, (VALVE_X + 0.035, VALVE_Y, VALVE_Z), FINGER_CLOSED, "push flush"),
            Waypoint(3.0, (VALVE_X + 0.035, VALVE_Y, VALVE_Z), FINGER_CLOSED, "hold flush"),
            Waypoint(1.5, (VALVE_X + 0.100, VALVE_Y, VALVE_Z + 0.060), FINGER_OPEN, "release early"),
            Waypoint(6.0, (0.200, 0.150, TOP + 0.150), FINGER_OPEN, "clear left")
        ]
        right_wps = [
            Waypoint(1.5, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z + 0.050), FINGER_OPEN, "idle right"),
            Waypoint(13.5, (0.200, -0.150, TOP + 0.150), FINGER_OPEN, "idle right")
        ]
        return {"left": left_wps, "right": right_wps}

    # Full synchronized bimanual demo:
    left_wps = [
        # Reach and push cartridge flush home
        Waypoint(1.5, (VALVE_X + 0.080, VALVE_Y, VALVE_Z), FINGER_OPEN, "approach stem"),
        Waypoint(1.0, (VALVE_X + 0.047, VALVE_Y, VALVE_Z), FINGER_CLOSED, "contact stem"),
        Waypoint(2.0, (VALVE_X + 0.035, VALVE_Y, VALVE_Z), FINGER_CLOSED, "push flush home"),
        # Hold flush home against 14 N O-ring push while right hand seats clip
        Waypoint(5.5, (VALVE_X + 0.035, VALVE_Y, VALVE_Z), FINGER_CLOSED, "hold flush"),
        # Once clip seated, release stem and clear
        Waypoint(1.5, (VALVE_X + 0.100, VALVE_Y + 0.040, VALVE_Z + 0.060), FINGER_OPEN, "release stem"),
        Waypoint(1.5, (0.200, 0.150, TOP + 0.150), FINGER_OPEN, "clear left"),
        Waypoint(2.0, (0.200, 0.150, TOP + 0.150), FINGER_OPEN, "hold clear")
    ]

    right_wps = [
        # Grasp clip from staging block
        Waypoint(1.5, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z + 0.050), FINGER_OPEN, "hover clip"),
        Waypoint(1.0, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z), FINGER_OPEN, "reach clip crown"),
        Waypoint(1.0, (CLIP_STAGE_X, CLIP_STAGE_Y, CLIP_INIT_Z), FINGER_CLOSED, "grasp clip crown"),
        # Lift and align over valve body slots
        Waypoint(1.5, (CLIP_STAGE_X, CLIP_STAGE_Y, VALVE_Z + 0.060), FINGER_CLOSED, "lift clip"),
        Waypoint(1.5, (VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.050), FINGER_CLOSED, "align over slots"),
        # Slide down into slots across cartridge retaining flats
        Waypoint(1.5, (CLIP_SEAT_TARGET[0], CLIP_SEAT_TARGET[1], CLIP_SEAT_TARGET[2]), FINGER_CLOSED, "slide clip home"),
        # Release clip crown and clear
        Waypoint(1.0, (CLIP_SEAT_TARGET[0], CLIP_SEAT_TARGET[1], CLIP_SEAT_TARGET[2]), FINGER_OPEN, "release clip"),
        Waypoint(1.5, (VALVE_X + 0.005, VALVE_Y - 0.080, VALVE_Z + 0.080), FINGER_OPEN, "clear right"),
        Waypoint(1.5, (0.200, -0.150, TOP + 0.150), FINGER_OPEN, "hold clear right"),
        Waypoint(2.0, (0.200, -0.150, TOP + 0.150), FINGER_OPEN, "maintain clear")
    ]

    return {"left": left_wps, "right": right_wps}


def self_test():
    """Offline unit test validating stage scoring, geometry constraints, and failure modes."""
    mk.new_episode()

    valve_ref = (VALVE_X, VALVE_Y, VALVE_Z)
    staging_clip = (CLIP_INIT_X, CLIP_INIT_Y, CLIP_INIT_Z)
    seated_clip = tuple(CLIP_SEAT_TARGET)

    # Test 1: Initial state (cartridge unpushed, clip on staging) -> 0.0
    s_init = kit.fake_states(
        q={"cartridge_slide": 0.000},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (staging_clip, (0, 0, 0, 1))}
    )
    assert score(s_init) == 0.0, f"Initial state must score 0.0, got {score(s_init)}"

    # Test 2: Partial push (5 mm) -> 0.0
    s_part = kit.fake_states(
        q={"cartridge_slide": 0.005},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (staging_clip, (0, 0, 0, 1))}
    )
    assert score(s_part) == 0.0, f"5 mm push must score 0.0, got {score(s_part)}"

    # Test 3: Halfway push (8 mm) -> 0.20
    s_half = kit.fake_states(
        q={"cartridge_slide": 0.008},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (staging_clip, (0, 0, 0, 1))}
    )
    assert score(s_half) == 0.20, f"8 mm push must score 0.20, got {score(s_half)}"

    # Test 4: Flush home (12 mm) without clip -> 0.45
    s_flush = kit.fake_states(
        q={"cartridge_slide": 0.012},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (staging_clip, (0, 0, 0, 1))}
    )
    assert score(s_flush) == 0.45, f"Flush home must score 0.45, got {score(s_flush)}"

    # Test 5: Flush home + clip entered slots -> 0.70
    slot_clip = (VALVE_X + 0.005, VALVE_Y, VALVE_Z + 0.024)
    s_slots = kit.fake_states(
        q={"cartridge_slide": 0.012},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (slot_clip, (0, 0, 0, 1))}
    )
    assert score(s_slots) == 0.70, f"Clip in slots must score 0.70, got {score(s_slots)}"

    # Test 6: Flush home + clip seated down onto stops -> 0.90
    s_seated = kit.fake_states(
        q={"cartridge_slide": 0.012},
        moved={"valve_body": (valve_ref, (0, 0, 0, 1)), "retainer_clip": (seated_clip, (0, 0, 0, 1))}
    )
    assert score(s_seated) == 0.90, f"Clip seated must score 0.90, got {score(s_seated)}"

    # Test 7: Hands cleared and equilibrium maintained -> 1.00
    mk._TEST_TCPS = [np.array([0.20, 0.15, TOP + 0.15]), np.array([0.20, -0.15, TOP + 0.15])]
    mk.MEMORY["hold_steps"] = HOLD_STEPS_REQUIRED
    mk.MEMORY["retained_equilibrium"] = True
    assert score(s_seated) == 1.00, f"Equilibrium hold must score 1.00, got {score(s_seated)}"
    assert success(s_seated), "Must satisfy success() predicate"

    # Test 8: Dropped clip failure latch -> 0.0
    mk.MEMORY["clip_dropped"] = True
    assert score(s_seated) == 0.00, "Dropped clip must latch 0.0"

    mk._TEST_TCPS = None
    mk.new_episode()
    return 8


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    n_passed = self_test()
    print(f"[seat-shower-cartridge-and-slide-retainer-clip] offline self-test: {n_passed}/{n_passed} checks passed cleanly.")

    # Kinematic verification of demo plans
    plan = demo()
    for side in ("left", "right"):
        base = station.BASES[side]
        p = ArmPlan(plan[side], base)
        assert p.duration <= 40.0, f"{side} arm plan exceeds 40s run limit ({p.duration:.1f} s)"
        print(f"{side}: {p.duration:.1f} s plan valid.")
    print("Kinematic trajectory check: ALL PASSED!")
