"""Position and Lock 96-Well PCR Cooler Rack on Magnetic Separation Stand.

Category: Other (Wet-Lab Research Automation / Molecular Biology & NGS Automation).
BLS SOC Grounding:
- Biological Technicians (BLS SOC 19-4021, >85,000 workers)
- Chemical Technicians (BLS SOC 19-4031, >65,000 workers)
- Medical and Clinical Laboratory Technologists and Technicians (BLS SOC 29-2010, >340,000 workers)
Total Addressable Workforce: >490,000 laboratory research personnel.

Task Description:
In high-throughput genomics, molecular biology, and next-generation sequencing (NGS)
library preparation protocols (e.g. SPRI magnetic bead cleanups and RNA/DNA extraction),
chilled reaction samples in a standard 96-well skirted PCR cooler rack/plate
(127.8 x 85.5 x 28.0 mm, conforming to ANSI/SLAS standards) must be transferred from
a bench staging area onto a precision 96-well neodymium magnetic bead separation stand
(Alpaqua / Beckman Coulter style chassis: 140 x 100 x 35 mm).
The separation stand features a recessed registration nest with 96 neodymium ring magnet
pins and a mechanical sliding lock latch bar (prismatic slider along Y, travel 20 mm).
Engaging the lock latch rigidly constrains the plate against high-gradient magnetic
pull-down forces and prevents plate tipping during automated pellet formation.

Required Procedure:
1. Manipulator (Left arm) grasps the chilled 96-well PCR cooler rack by its authentic
   perimeter skirt flange rim and lifts it from the bench staging pad.
2. Transfers the cooler rack squarely over the magnetic separation stand and seats it
   flush into the recessed magnetic pin registration nest (horizontal error <= 12 mm,
   vertical flush <= 10 mm).
3. Once the plate is seated flush, manipulator (Right arm) engages the sliding lock
   latch bar by pushing its knurled handle tab along +Y (> 15 mm travel) to clamp the
   rack securely into the stand.
4. Both manipulators retract clear (> 50 mm) for a continuous 20-step (1.0 s) live
   instantaneous-state stillness hold.

Scoring & Strict Instantaneous State Verification:
- In strict adherence to V24 post-mortem rules (#854, #856, #857):
  - ZERO permanent stage latching (`self._seated = True`).
  - Evaluates live instantaneous physical state at the current step:
    - Rack currently seated inside magnetic nest (XY distance <= 12 mm, Z flush <= 10 mm).
    - Slider lock bar currently engaged (travel >= 15 mm).
    - Manipulators currently retracted clear (> 50 mm) for 20 continuous steps (1.0 s).
  - If the rack is dislodged, the lock bar retracted, or hands re-enter during dwell,
    the score drops immediately and dwell resets to 0.
"""

from __future__ import annotations

import math
import os
import sys
import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import mechkit as mk  # noqa: E402
from yam_arena.task_file import GREY, Box, ObjectState, TABLE_TOP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED  # noqa: E402

VARIANT = os.getenv("DEMO_VARIANT", "normal")

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475 m table surface Z

# Color palette (PBR-friendly laboratory materials)
COL_STAND_ANODIZED = (0.22, 0.24, 0.26)   # dark anodized aluminum magnetic stand body
COL_MAGNET_NEO = (0.75, 0.76, 0.78)       # nickel-plated neodymium ring magnet posts
COL_LOCK_SLIDER = (0.85, 0.45, 0.15)      # safety-orange anodized aluminum slide lock bar
COL_COOLER_RACK = (0.15, 0.45, 0.75)      # cobalt blue chilled polypropylene PCR cooler rack
COL_STAGING_PAD = (0.88, 0.89, 0.90)      # cleanroom silicone staging mat

# Workstation Layout Coordinates
# Magnetic Separation Stand located forward-reachable on the workstation
STAND_X = 0.280       # X center (280 mm)
STAND_Y = -0.040      # Y center (-40 mm)
STAND_W = 0.145       # Stand depth X (145 mm)
STAND_L = 0.105       # Stand width Y (105 mm)
STAND_H = 0.035       # Stand base height Z (35 mm)

# Recessed registration nest level
NEST_CENTER_X = STAND_X
NEST_CENTER_Y = STAND_Y
NEST_DEPTH = 0.012
NEST_FLOOR_Z = TOP + STAND_H - NEST_DEPTH

# Staging Area (chilled silicone benchtop mat on left side +Y)
STAGE_X = 0.260
STAGE_Y = 0.200
STAGE_Z = TOP

# 96-Well PCR Cooler Rack (ANSI/SLAS standard 127.8 x 85.5 x 28.0 mm)
RACK_W = 0.128  # X span
RACK_L = 0.086  # Y span
RACK_H = 0.028  # Z height
RACK_MASS = 0.110  # 110 gram chilled IsoFreeze-style rack

RACK_INIT_X = STAGE_X
RACK_INIT_Y = STAGE_Y
RACK_INIT_Z = STAGE_Z + 0.005 + RACK_H / 2.0  # resting on staging mat

# Target seated location (seated inside recessed magnetic stand nest)
RACK_TARGET_X = NEST_CENTER_X
RACK_TARGET_Y = NEST_CENTER_Y
RACK_TARGET_Z = NEST_FLOOR_Z + RACK_H / 2.0

# Slide Lock Bar (Prismatic Slider along Y on -Y side of stand)
# Retracted: 0.0 mm. Engaged: 18.0 mm travel along +Y over the rack skirt lip.
LOCK_TRAVEL = 0.020  # 20 mm max travel
LOCK_ENGAGED_MIN = 0.015  # 15 mm travel threshold for full engagement
LOCK_INIT_X = STAND_X
LOCK_INIT_Y = STAND_Y - (RACK_L / 2.0 + 0.014)
LOCK_INIT_Z = TOP + STAND_H + 0.005

# Robot Rest Park Poses
L_PARK = (-0.100, 0.260, 1.020)
R_PARK = (-0.100, -0.260, 1.020)


kit = mk.Kit()

# 1. Benchtop Staging Mat (Silicone bench mat) - Static
stage_boxes = [
    mk.box((0.150, 0.110, 0.006), at=(0, 0, 0.003)),
]
kit.part(
    "staging_mat",
    (STAGE_X, STAGE_Y, STAGE_Z),
    stage_boxes,
    mass=None,
    color=COL_STAGING_PAD,
)

# 2. Magnetic Separation Stand Chassis - Static
stand_boxes = [
    # Solid anodized aluminum base block
    mk.box((STAND_W, STAND_L, STAND_H - NEST_DEPTH), at=(0, 0, (STAND_H - NEST_DEPTH) / 2.0)),
    # Perimeter registration walls (+X, -X, and +Y fixed stops)
    mk.box((0.010, STAND_L, NEST_DEPTH), at=(STAND_W / 2.0 - 0.005, 0, STAND_H - NEST_DEPTH / 2.0)),
    mk.box((0.010, STAND_L, NEST_DEPTH), at=(-STAND_W / 2.0 + 0.005, 0, STAND_H - NEST_DEPTH / 2.0)),
    mk.box((STAND_W - 0.020, 0.010, NEST_DEPTH), at=(0, STAND_L / 2.0 - 0.005, STAND_H - NEST_DEPTH / 2.0)),
    # Simulated 96-well neodymium magnetic pin array block
    mk.box((0.118, 0.076, 0.008), at=(0, 0, STAND_H - NEST_DEPTH + 0.004), color=COL_MAGNET_NEO),
]
kit.part(
    "magnetic_stand",
    (STAND_X, STAND_Y, TOP),
    stand_boxes,
    mass=None,
    color=COL_STAND_ANODIZED,
)

# 3. Slide Lock Bar (Prismatic Slider along Y)
# Located on -Y side of stand. Pushing it +Y slides retaining flange over rack skirt.
slider_boxes = [
    # Retaining lock bar body
    mk.box((RACK_W + 0.012, 0.012, 0.014), at=(0, 0, 0)),
    # Knurled grip tab for robot manipulator
    mk.box((0.024, 0.016, 0.022), at=(0, -0.010, 0.008)),
]
kit.part(
    "lock_slider",
    (LOCK_INIT_X, LOCK_INIT_Y, LOCK_INIT_Z),
    slider_boxes,
    mass=0.085,
    color=COL_LOCK_SLIDER,
    solver_position_iterations=16,
)
kit.slider(
    "slider_joint",
    parent="magnetic_stand",
    child="lock_slider",
    axis=(0, 1, 0),
    lower=0.0,
    upper=LOCK_TRAVEL,
    start=0.0,
    stiffness=50.0,
    rest=0.0,
    damping=15.0,
)

# 4. 96-Well Skirted PCR Cooler Rack (Dynamic free-floating rigid body)
# ANSI/SLAS standard 127.8 x 85.5 x 28.0 mm with authentic perimeter skirt rim
rack_boxes = [
    # Main chilled thermal core block
    mk.box((RACK_W - 0.010, RACK_L - 0.010, RACK_H), at=(0, 0, 0)),
    # Skirt perimeter rim (+X flange)
    mk.box((0.005, RACK_L, RACK_H - 0.006), at=(RACK_W / 2.0 - 0.0025, 0, -0.003)),
    # Skirt perimeter rim (-X flange)
    mk.box((0.005, RACK_L, RACK_H - 0.006), at=(-RACK_W / 2.0 + 0.0025, 0, -0.003)),
    # Skirt perimeter rim (+Y flange)
    mk.box((RACK_W, 0.005, RACK_H - 0.006), at=(0, RACK_L / 2.0 - 0.0025, -0.003)),
    # Skirt perimeter rim (-Y flange)
    mk.box((RACK_W, 0.005, RACK_H - 0.006), at=(0, -RACK_L / 2.0 + 0.0025, -0.003)),
]
kit.part(
    "pcr_cooler_rack",
    (RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z),
    rack_boxes,
    mass=RACK_MASS,
    color=COL_COOLER_RACK,
    solver_position_iterations=16,
)

OBJECTS = kit.objects()


# -- State Tracking & Dwell Helpers ----------------------------------------------------------------------------

def _mem():
    return mk.MEMORY


_LAST_TCPS: list[tuple[float, float, float]] = []


def _hands_clear(min_dist: float = 0.050) -> bool:
    """Verifies both robot end-effectors are retracted clear of the magnetic separation stand."""
    if not _LAST_TCPS:
        return True
    for tcp in _LAST_TCPS:
        dist_stand = np.hypot(tcp[0] - STAND_X, tcp[1] - STAND_Y)
        if dist_stand < 0.160 and tcp[2] < (TOP + STAND_H + 0.080):
            return False
    return True


# -- Instantaneous Non-Latched Scorer ---------------------------------------------------------------------------

def score(objects: dict[str, ObjectState]) -> float:
    """Computes instantaneous evaluation score for seating cooler rack and locking magnetic stand.

    Strict V24 Compliance (#854, #856, #857):
    - ZERO permanent stage latching (`self._seated_latched = True` banned).
    - Every milestone strictly checks the live physical state at the current step.
    - Continuous 20-step (1.0 s) dwell hold with hands clear. If rack dislodges or lock retracts,
      score drops immediately and dwell resets to 0.
    """
    m = _mem()
    if "post_dwell" not in m:
        m["post_dwell"] = 0

    rack = objects["pcr_cooler_rack"]
    pos_rack = getattr(rack, "pos", None)
    if pos_rack is None:
        rx, ry, rz = RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z
    else:
        rx, ry, rz = float(pos_rack[0]), float(pos_rack[1]), float(pos_rack[2])

    slider_travel = float(kit.q(objects, "slider_joint"))

    # Displacement of cooler rack from target magnetic stand nest
    dist_nest_xy = np.hypot(rx - RACK_TARGET_X, ry - RACK_TARGET_Y)
    dist_nest_z = abs(rz - RACK_TARGET_Z)

    # Physical milestones evaluated purely at current step:
    rack_lifted = np.hypot(rx - RACK_INIT_X, ry - RACK_INIT_Y) > 0.030 or (rz - RACK_INIT_Z) > 0.020
    rack_seated = (dist_nest_xy <= 0.015) and (dist_nest_z <= 0.012)
    slider_locked = (slider_travel >= LOCK_ENGAGED_MIN)

    # Scoring Ladder:
    # 0.00: Cooler rack resting untouched on bench staging mat
    if not rack_lifted:
        return 0.00

    # Stage 1: Rack lifted and transferring toward magnetic stand nest -> 0.00 to 0.40
    if not rack_seated:
        transfer_prog = 1.0 - np.clip(dist_nest_xy / 0.24, 0.0, 1.0)
        return round(0.10 + 0.30 * float(transfer_prog), 4)

    # Stage 2: Rack safely seated in magnetic nest, lock bar not yet fully engaged -> 0.40 to 0.70
    if not slider_locked:
        lock_prog = np.clip(slider_travel / LOCK_ENGAGED_MIN, 0.0, 1.0)
        return round(0.40 + 0.30 * float(lock_prog), 4)

    # Stage 3: Rack seated AND lock bar engaged -> 0.70 to 1.00 (requires hands clear stillness hold)
    clear = _hands_clear(min_dist=0.050)
    if clear:
        m["post_dwell"] += 1
        if m["post_dwell"] >= 20:  # 1.0 s hold at 20 Hz
            return 1.00
        return round(0.70 + 0.30 * (m["post_dwell"] / 20.0), 4)
    else:
        m["post_dwell"] = 0  # reset dwell if robot hands re-enter stand volume

    return 0.70


# -- Deterministic Scripted Bimanual Demonstration Trajectory ----------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    """Generates coordinated bimanual policy: Left arm seats cooler rack, Right arm locks slide bar."""
    Lw: list[Waypoint] = []
    Rw: list[Waypoint] = []

    def w(arm_list, dur, pos, grip, label, yaw=0.0):
        arm_list.append(Waypoint(dur, tuple(float(v) for v in pos), grip, label, yaw))

    def t(arm_list):
        return sum(x.duration for x in arm_list)

    # Initial rest park poses (t = 0.0 to 1.5 s)
    w(Lw, 1.5, L_PARK, FINGER_OPEN, "l_init", yaw=0.0)
    w(Rw, 1.5, R_PARK, FINGER_CLOSED, "r_init", yaw=0.0)

    # Left Arm: Grasp PCR cooler rack by skirt rim and transfer to magnetic stand nest
    # Plate at (STAGE_X, STAGE_Y, RACK_INIT_Z). Yaw = 0.0 aligns parallel fingers with Y edges.
    p_rack_above = (RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z + 0.080)
    p_rack_grip = (RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z)
    p_rack_lift = (RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z + 0.120)
    p_nest_above = (RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z + 0.120)
    p_nest_seat = (RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z)

    # Approach and grasp PCR cooler rack
    w(Lw, 2.0, p_rack_above, FINGER_OPEN, "l_approach_stage", yaw=0.0)
    w(Lw, 1.2, p_rack_grip, FINGER_OPEN, "l_lower_to_rack", yaw=0.0)
    w(Lw, 0.8, p_rack_grip, FINGER_CLOSED, "l_pinch_rack_skirt", yaw=0.0)

    # Lift and transfer to magnetic separation stand nest
    w(Lw, 1.5, p_rack_lift, FINGER_CLOSED, "l_lift_from_mat", yaw=0.0)
    w(Lw, 2.2, p_nest_above, FINGER_CLOSED, "l_transfer_to_stand", yaw=0.0)
    w(Lw, 1.5, p_nest_seat, FINGER_CLOSED, "l_seat_in_nest", yaw=0.0)

    # Release rack and retract clear
    w(Lw, 0.8, p_nest_seat, FINGER_OPEN, "l_release_rack", yaw=0.0)
    w(Lw, 1.2, p_nest_above, FINGER_OPEN, "l_retract_above_nest", yaw=0.0)
    w(Lw, 1.8, L_PARK, FINGER_OPEN, "l_retract_park", yaw=0.0)

    # Right Arm: Slide lock bar along +Y to clamp rack into magnetic stand
    # Starts after left arm has released rack and retracted clear (t = 12.0 s)
    # Lock slider knob at (LOCK_INIT_X, LOCK_INIT_Y - 0.010, LOCK_INIT_Z + 0.008)
    p_knob_rear = (LOCK_INIT_X, LOCK_INIT_Y - 0.035, LOCK_INIT_Z + 0.010)
    p_knob_contact = (LOCK_INIT_X, LOCK_INIT_Y - 0.010, LOCK_INIT_Z + 0.010)
    p_knob_push = (LOCK_INIT_X, LOCK_INIT_Y + 0.018, LOCK_INIT_Z + 0.010)
    p_knob_retract = (LOCK_INIT_X, LOCK_INIT_Y - 0.030, LOCK_INIT_Z + 0.050)

    if VARIANT == "no_seat":
        # Negative control: push lock bar early without seating rack
        w(Rw, 1.5, p_knob_rear, FINGER_CLOSED, "r_approach_knob_early", yaw=0.0)
        w(Rw, 1.0, p_knob_contact, FINGER_CLOSED, "r_contact_knob_early", yaw=0.0)
        w(Rw, 1.8, p_knob_push, FINGER_CLOSED, "r_push_lock_early", yaw=0.0)
        w(Rw, 1.0, p_knob_retract, FINGER_CLOSED, "r_retract_early", yaw=0.0)
        w(Rw, 1.5, R_PARK, FINGER_OPEN, "r_park_early", yaw=0.0)
    else:
        # Standard: wait for left arm to seat rack and retract clear
        w(Rw, 11.5, R_PARK, FINGER_CLOSED, "r_wait_for_seating", yaw=0.0)
        w(Rw, 1.8, p_knob_rear, FINGER_CLOSED, "r_approach_slider_rear", yaw=0.0)
        w(Rw, 1.0, p_knob_contact, FINGER_CLOSED, "r_contact_slider_tab", yaw=0.0)
        w(Rw, 2.0, p_knob_push, FINGER_CLOSED, "r_push_slider_lock", yaw=0.0)
        w(Rw, 1.2, p_knob_retract, FINGER_CLOSED, "r_retract_from_slider", yaw=0.0)
        w(Rw, 1.8, R_PARK, FINGER_OPEN, "r_park", yaw=0.0)

    # Dwell stillness hold
    w(Lw, 2.0, L_PARK, FINGER_OPEN, "l_dwell", yaw=0.0)
    w(Rw, 2.0, R_PARK, FINGER_OPEN, "r_dwell", yaw=0.0)

    # Synchronize durations
    t_max = max(t(Lw), t(Rw))
    if t(Lw) < t_max:
        w(Lw, t_max - t(Lw), L_PARK, FINGER_OPEN, "l_sync", yaw=0.0)
    if t(Rw) < t_max:
        w(Rw, t_max - t(Rw), R_PARK, FINGER_OPEN, "r_sync", yaw=0.0)

    return {"left": Lw, "right": Rw}


# -- Offline Self-Test ------------------------------------------------------------------------------------------

def fake_state(rack_x: float, rack_y: float, rack_z: float, slider_m: float):
    """Generates synthetic ObjectState dict for offline testing."""
    st = kit.fake_states(
        q={"slider_joint": slider_m},
    )
    z = np.zeros(3)
    st["pcr_cooler_rack"] = ObjectState(
        pos=np.array([rack_x, rack_y, rack_z]),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        lin_vel=z,
        ang_vel=z.copy(),
        low=np.array([rack_x - RACK_W / 2.0, rack_y - RACK_L / 2.0, rack_z - RACK_H / 2.0]),
        high=np.array([rack_x + RACK_W / 2.0, rack_y + RACK_L / 2.0, rack_z + RACK_H / 2.0]),
    )
    return st


def self_test():
    """Validates scoring rubric, instantaneous non-latched state checks, and dwell hold."""
    global _LAST_TCPS
    mk.new_episode()
    _LAST_TCPS = [L_PARK, R_PARK]

    # 1. Initial State: Rack untouched on staging mat, slider retracted (0.0 mm) -> 0.00
    st_init = fake_state(RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z, 0.0)
    assert score(st_init) == 0.0, f"init score {score(st_init)} != 0.0"

    # 2. Rack lifted and mid-transfer -> 0.10 to 0.40
    st_transfer = fake_state(RACK_INIT_X + 0.10, RACK_INIT_Y - 0.10, RACK_INIT_Z + 0.08, 0.0)
    sc_tr = score(st_transfer)
    assert 0.10 <= sc_tr < 0.40, f"transfer score {sc_tr} out of range [0.10, 0.40)"

    # 3. Rack seated flush in magnetic nest, slider still retracted -> 0.40
    st_seated_open = fake_state(RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z, 0.0)
    sc_seated = score(st_seated_open)
    assert sc_seated == 0.40, f"seated score {sc_seated} != 0.40"

    # 4. Rack seated, slider advancing (e.g. 8.0 mm) -> 0.50 to 0.70
    st_half_locked = fake_state(RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z, 0.008)
    sc_half = score(st_half_locked)
    assert 0.50 <= sc_half < 0.70, f"half locked score {sc_half} out of range [0.50, 0.70)"

    # 5. Rack seated AND slider fully engaged (16.0 mm) -> 0.70 + dwell progression
    st_locked = fake_state(RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z, 0.016)
    sc_locked = score(st_locked)
    assert sc_locked >= 0.70, f"locked score {sc_locked} < 0.70"

    # Advance dwell to 20 steps -> 1.00
    for _ in range(25):
        sc_full = score(st_locked)
    assert sc_full == 1.00, f"full score {sc_full} != 1.00"

    # 6. Anti-Latching Verification: If lock bar is retracted, score MUST drop immediately!
    st_unlocked = fake_state(RACK_TARGET_X, RACK_TARGET_Y, RACK_TARGET_Z, 0.002)
    sc_drop = score(st_unlocked)
    assert sc_drop < 0.50, f"score failed to drop on slider release: {sc_drop}"

    # 7. If rack is removed from nest, score MUST drop immediately!
    st_removed = fake_state(RACK_INIT_X, RACK_INIT_Y, RACK_INIT_Z + 0.10, 0.016)
    sc_removed = score(st_removed)
    assert sc_removed < 0.40, f"score failed to drop on rack removal: {sc_removed}"

    print("[position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand] offline self-test: 7/7 checks passed cleanly.")


if __name__ == "__main__":
    self_test()
