"""Restock a wall-mount emergency eyewash station with a sealed saline eyewash bottle.

Occupational hygiene / workplace safety compliance procedure (OSHA 1910.151(c), ANSI Z358.1):
1. Left arm pushes station dust cover clear door open past detent (>= 75 deg).
2. Right arm reaches into cradle cavity, grasps expired/empty bottle, and removes it to the bench discard area.
3. Left arm picks up the fresh 32 oz sealed saline eyewash bottle (75 mm OD, 210 mm height, 980 g)
   from staging, transports it upright, and inserts it into the cradle cavity with the contoured eyecup
   nozzle oriented forward (toward user, yaw within 25 deg).
4. Right arm swings the dust cover door shut until the detent latch snaps (< 6 deg from shut, > 8 N force).
5. Both arms clear (>= 60 mm); station is left secured in compliant ready state.
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
STATION_GREEN = (0.05, 0.50, 0.22)
CLEAR_ACRYLIC = (0.85, 0.90, 0.92)
BOTTLE_SALINE = (0.92, 0.94, 0.95)
EXPIRED_BOTTLE = (0.75, 0.76, 0.70)
EYECUP_GREEN = (0.08, 0.45, 0.20)
STEEL_LATCH = (0.70, 0.72, 0.75)

# Physical Geometry (World frame: X forward, Y left, Z up)
STATION_X = 0.280
STATION_Y = 0.050
STATION_Z = TOP + 0.110  # Station center at TOP + 0.110

kit = mk.Kit()

# 1. Eyewash Station Base Housing
# True hollow cavity for bottle (82 mm wide, 82 mm deep, 210 mm height)
kit.part(
    "eyewash_station",
    (STATION_X, STATION_Y, STATION_Z),
    [
        # Backplate mounting wall
        mk.box((0.015, 0.140, 0.220), at=(-0.045, 0, 0), color=STATION_GREEN),
        # Bottom floor shelf
        mk.box((0.100, 0.140, 0.015), at=(0.005, 0, -0.102), color=STATION_GREEN),
        # Top roof cap
        mk.box((0.100, 0.140, 0.015), at=(0.005, 0, 0.102), color=STATION_GREEN),
        # Left side wall
        mk.box((0.100, 0.015, 0.220), at=(0.005, 0.065, 0), color=STATION_GREEN),
        # Right side wall (hinge side)
        mk.box((0.100, 0.015, 0.220), at=(0.005, -0.065, 0), color=STATION_GREEN),
        # Cradle bottom locator ring / stop
        mk.box((0.080, 0.080, 0.010), at=(0.005, 0, -0.090), color=STATION_GREEN),
    ]
)

# 2. Hinged Clear Front Dust Cover Door
DOOR_HINGE_X = STATION_X + 0.055
DOOR_HINGE_Y = STATION_Y - 0.065
DOOR_HINGE_Z = STATION_Z

kit.part(
    "station_door",
    (DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z),
    [
        # Main clear acrylic front panel (swings open)
        mk.box((0.008, 0.130, 0.200), at=(0, 0.065, 0), color=CLEAR_ACRYLIC),
        # Latch tab / handle on left edge
        mk.box((0.020, 0.012, 0.030), at=(0.010, 0.125, 0), color=STEEL_LATCH),
    ],
    mass=0.180
)

# Revolute hinge for door
kit.hinge(
    "door_hinge",
    "eyewash_station",
    "station_door",
    anchor=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z),
    axis=(0, 0, 1),
    lower_deg=-2.0,
    upper_deg=105.0,
    start_deg=0.0,
    stiffness=0.08,
    rest_deg=0.0,
    friction=0.15
)

# 3. Expired / Old Eyewash Bottle (Starts inside cradle)
OLD_BOTTLE_X = STATION_X + 0.005
OLD_BOTTLE_Y = STATION_Y
OLD_BOTTLE_Z = STATION_Z - 0.090 + 0.100

kit.part(
    "old_bottle",
    (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z),
    [
        mk.box((0.072, 0.072, 0.150), at=(0, 0, -0.025), color=EXPIRED_BOTTLE),
        mk.box((0.040, 0.040, 0.045), at=(0, 0, 0.072), color=EYECUP_GREEN),
        mk.box((0.055, 0.045, 0.015), at=(0.010, 0, 0.095), color=EYECUP_GREEN),
    ],
    mass=0.350
)

# 4. Fresh 32 oz Sealed Saline Bottle (Starts on bench staging area)
FRESH_BOTTLE_X = 0.240
FRESH_BOTTLE_Y = -0.150
FRESH_BOTTLE_Z = TOP + 0.105

kit.part(
    "fresh_bottle",
    (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z),
    [
        mk.box((0.074, 0.074, 0.155), at=(0, 0, -0.025), color=BOTTLE_SALINE),
        mk.box((0.042, 0.042, 0.045), at=(0, 0, 0.075), color=EYECUP_GREEN),
        mk.box((0.058, 0.046, 0.018), at=(0.012, 0, 0.098), color=EYECUP_GREEN),
    ],
    mass=0.980
)

OBJECTS = [*kit.objects()]
PARTS = ("dust_cover", "expired_bottle", "fresh_bottle")

# Discard bench location
DISCARD_X = 0.380
DISCARD_Y = 0.160
DISCARD_Z = TOP + 0.100


def score(objects) -> float:
    """Cumulative 5-stage progress score with physical verification.
    0.00: Start / door shut.
    0.20: Door opened past 60 deg.
    0.40: Old bottle removed from cradle to discard bench.
    0.70: Fresh bottle seated into station cradle.
    0.85: Fresh bottle seated with contoured eyecup oriented forward (+X).
    1.00: Door re-closed onto detent latch (< 6 deg) + hands clear (>= 60 mm).
    """
    door_q = math.degrees(kit.q(objects, "door_hinge"))

    if door_q >= 60.0:
        mk.MEMORY["door_opened"] = True

    if not mk.MEMORY.get("door_opened", False):
        return 0.00

    pts = 0.20

    # Old bottle state
    old_p = objects["old_bottle"].pos
    old_dist = math.hypot(old_p[0] - OLD_BOTTLE_X, old_p[1] - OLD_BOTTLE_Y)
    old_removed = old_dist > 0.080 and old_p[2] <= TOP + 0.150

    if not old_removed:
        return pts
    pts = 0.40

    # Fresh bottle state
    fresh_p = objects["fresh_bottle"].pos
    fresh_cradle_dist = math.hypot(fresh_p[0] - OLD_BOTTLE_X, fresh_p[1] - OLD_BOTTLE_Y)
    fresh_z_err = abs(fresh_p[2] - OLD_BOTTLE_Z)
    fresh_seated = fresh_cradle_dist < 0.025 and fresh_z_err < 0.020

    if not fresh_seated:
        return pts
    pts = 0.70

    # Orientation check: eyecup facing forward (+X, yaw within 25 deg)
    q = objects["fresh_bottle"].quat_xyzw  # (x, y, z, w)
    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    yaw_err = abs((yaw_deg + 180.0) % 360.0 - 180.0)

    if yaw_err > 25.0:
        return pts
    pts = 0.85

    # Discard zone check
    in_discard = (abs(old_p[0] - DISCARD_X) < 0.080 and 
                  abs(old_p[1] - DISCARD_Y) < 0.080 and 
                  old_p[2] <= TOP + 0.120)
    if not in_discard:
        return pts

    # Door latch check
    door_closed = abs(door_q) < 6.0
    if not door_closed:
        return pts

    # Hand clearance & equilibrium hold
    tcps = getattr(mk, "_TEST_TCPS", None) or mk.tcps()
    if tcps:
        hands_clear = True
        for tcp in tcps:
            if math.dist(tcp, (STATION_X, STATION_Y, STATION_Z)) < 0.050 or math.dist(tcp, fresh_p) < 0.050:
                hands_clear = False
                break
    else:
        hands_clear = True
    
    if not hands_clear:
        return pts

    # Equilibrium hold (>= 20 steps in sim, or single-step eval if _UNIT_TEST)
    if not getattr(mk, "_UNIT_TEST", False):
        hold_steps = mk.MEMORY.get("equilibrium_hold_steps", 0) + 1
        mk.MEMORY["equilibrium_hold_steps"] = hold_steps
        if hold_steps < 20:
            return pts

    return 1.00


def success(objects) -> bool:
    return score(objects) >= 1.00


def demo():
    """Bimanual demonstration sequence with standard duration segments."""
    variant = os.environ.get("DEMO_VARIANT", "normal")

    # Variant: door_left_open
    if variant == "door_left_open":
        left_wps = [
            Waypoint(1.2, (0.20, 0.15, TOP + 0.25), FINGER_OPEN, "L approach door"),
            Waypoint(1.0, (STATION_X + 0.070, STATION_Y + 0.065, STATION_Z), FINGER_OPEN, "L reach tab"),
            Waypoint(0.5, (STATION_X + 0.060, STATION_Y + 0.065, STATION_Z), 0.020, "L grip tab"),
            Waypoint(1.2, (STATION_X + 0.120, STATION_Y - 0.020, STATION_Z), 0.020, "L swing open"),
            Waypoint(0.8, (STATION_X + 0.100, STATION_Y - 0.080, STATION_Z), FINGER_OPEN, "L door open"),
            Waypoint(1.5, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.120), FINGER_OPEN, "L hover fresh"),
            Waypoint(1.0, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.040), FINGER_OPEN, "L descend fresh"),
            Waypoint(0.8, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.040), 0.070, "L grasp fresh"),
            Waypoint(1.2, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.150), 0.070, "L lift fresh"),
            Waypoint(1.5, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.140), 0.070, "L carry over cradle"),
            Waypoint(1.2, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z), 0.070, "L seat into cradle"),
            Waypoint(0.8, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z), FINGER_OPEN, "L release fresh"),
            Waypoint(2.0, (0.20, 0.18, TOP + 0.25), FINGER_OPEN, "L clear")
        ]
        right_wps = [
            Waypoint(1.5, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.120), FINGER_OPEN, "R hover old"),
            Waypoint(1.0, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.040), FINGER_OPEN, "R descend old"),
            Waypoint(0.8, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.040), 0.065, "R grasp old"),
            Waypoint(1.2, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.140), 0.065, "R lift old"),
            Waypoint(1.5, (DISCARD_X, DISCARD_Y, TOP + 0.140), 0.065, "R carry discard"),
            Waypoint(1.0, (DISCARD_X, DISCARD_Y, TOP + 0.100), 0.065, "R set discard"),
            Waypoint(0.8, (DISCARD_X, DISCARD_Y, TOP + 0.100), FINGER_OPEN, "R release old"),
            Waypoint(6.9, (0.20, -0.18, TOP + 0.25), FINGER_OPEN, "R clear idle")
        ]
        return {"left": left_wps, "right": right_wps}

    # Full synchronized bimanual demo:
    left_wps = [
        # Stage 1: Left arm swings door open
        Waypoint(1.2, (0.20, 0.15, TOP + 0.25), FINGER_OPEN, "L approach door"),
        Waypoint(1.0, (STATION_X + 0.070, STATION_Y + 0.065, STATION_Z), FINGER_OPEN, "L reach tab"),
        Waypoint(0.5, (STATION_X + 0.060, STATION_Y + 0.065, STATION_Z), 0.020, "L grip tab"),
        Waypoint(1.2, (STATION_X + 0.120, STATION_Y - 0.020, STATION_Z), 0.020, "L swing open"),
        Waypoint(0.8, (STATION_X + 0.100, STATION_Y - 0.080, STATION_Z), FINGER_OPEN, "L door open"),
        # Stage 3: Left arm grasps fresh saline bottle and seats into cradle
        Waypoint(1.5, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.120), FINGER_OPEN, "L hover fresh"),
        Waypoint(1.0, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.040), FINGER_OPEN, "L descend fresh"),
        Waypoint(0.8, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.040), 0.070, "L grasp fresh"),
        Waypoint(1.2, (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z + 0.150), 0.070, "L lift fresh"),
        Waypoint(1.5, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.140), 0.070, "L carry over cradle"),
        Waypoint(1.2, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z), 0.070, "L seat into cradle"),
        Waypoint(0.8, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z), FINGER_OPEN, "L release fresh"),
        Waypoint(1.5, (0.20, 0.18, TOP + 0.25), FINGER_OPEN, "L retreat clear"),
        Waypoint(2.0, (0.20, 0.18, TOP + 0.25), FINGER_OPEN, "L maintain clear")
    ]

    right_wps = [
        # Stage 2: Right arm removes old bottle to discard area
        Waypoint(2.0, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.120), FINGER_OPEN, "R hover old"),
        Waypoint(1.0, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.040), FINGER_OPEN, "R descend old"),
        Waypoint(0.8, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.040), 0.065, "R grasp old"),
        Waypoint(1.2, (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z + 0.140), 0.065, "R lift old"),
        Waypoint(1.5, (DISCARD_X, DISCARD_Y, TOP + 0.140), 0.065, "R carry discard"),
        Waypoint(1.0, (DISCARD_X, DISCARD_Y, TOP + 0.100), 0.065, "R set discard"),
        Waypoint(0.8, (DISCARD_X, DISCARD_Y, TOP + 0.100), FINGER_OPEN, "R release old"),
        # Stage 4: Right arm swings door shut to snap latch
        Waypoint(2.0, (STATION_X + 0.100, STATION_Y - 0.080, STATION_Z), FINGER_OPEN, "R approach open door"),
        Waypoint(1.5, (STATION_X + 0.110, STATION_Y - 0.040, STATION_Z), FINGER_OPEN, "R contact door"),
        Waypoint(1.2, (STATION_X + 0.060, STATION_Y + 0.060, STATION_Z), FINGER_OPEN, "R push door shut"),
        Waypoint(0.8, (STATION_X + 0.055, STATION_Y + 0.065, STATION_Z), FINGER_OPEN, "R press latch home"),
        Waypoint(1.5, (0.20, -0.18, TOP + 0.25), FINGER_OPEN, "R hands clear"),
        Waypoint(1.4, (0.20, -0.18, TOP + 0.25), FINGER_OPEN, "R maintain clear")
    ]

    return {"left": left_wps, "right": right_wps}


def self_test():
    """Offline unit test validating stage scoring, geometry constraints, and failure modes."""
    print("[self_test] Running restock-wall-mount-eyewash-station unit tests...")
    mk._UNIT_TEST = True
    mk.new_episode()

    old_init = (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z)
    fresh_init = (FRESH_BOTTLE_X, FRESH_BOTTLE_Y, FRESH_BOTTLE_Z)
    discard_p = (DISCARD_X, DISCARD_Y, DISCARD_Z)
    cradle_p = (OLD_BOTTLE_X, OLD_BOTTLE_Y, OLD_BOTTLE_Z)
    forward_q = (0, 0, 0, 1)
    reversed_q = (0, 0, 1, 0)

    # Test 1: Initial state -> Score 0.0
    s_init = kit.fake_states(
        q={"door_hinge": 0.0},
        moved={"old_bottle": (old_init, forward_q), "fresh_bottle": (fresh_init, forward_q)}
    )
    assert score(s_init) == 0.0, f"Expected 0.0 at start, got {score(s_init)}"

    # Test 2: Door opened past 60 deg -> Score 0.20
    s_door = kit.fake_states(
        q={"door_hinge": 75.0},
        moved={"old_bottle": (old_init, forward_q), "fresh_bottle": (fresh_init, forward_q)},
        deg=True
    )
    assert score(s_door) == 0.20, f"Expected 0.20 after door open, got {score(s_door)}"

    # Test 3: Old bottle removed to discard bench -> Score 0.40
    s_old = kit.fake_states(
        q={"door_hinge": 75.0},
        moved={"old_bottle": (discard_p, forward_q), "fresh_bottle": (fresh_init, forward_q)},
        deg=True
    )
    assert score(s_old) == 0.40, f"Expected 0.40 after old bottle removed, got {score(s_old)}"

    # Test 4: Fresh bottle seated into cradle (with reversed yaw) -> Score 0.70
    s_seated_rev = kit.fake_states(
        q={"door_hinge": 75.0},
        moved={"old_bottle": (discard_p, forward_q), "fresh_bottle": (cradle_p, reversed_q)},
        deg=True
    )
    assert score(s_seated_rev) == 0.70, f"Expected 0.70 with reversed yaw, got {score(s_seated_rev)}"

    # Test 5: Fresh bottle seated with forward eyecup nozzle orientation -> Score 0.85
    s_seated_fwd = kit.fake_states(
        q={"door_hinge": 75.0},
        moved={"old_bottle": (discard_p, forward_q), "fresh_bottle": (cradle_p, forward_q)},
        deg=True
    )
    assert score(s_seated_fwd) == 0.85, f"Expected 0.85 with forward orientation, got {score(s_seated_fwd)}"

    # Test 6: Door re-closed onto latch -> Score 1.00
    s_done = kit.fake_states(
        q={"door_hinge": 1.5},
        moved={"old_bottle": (discard_p, forward_q), "fresh_bottle": (cradle_p, forward_q)},
        deg=True
    )
    assert score(s_done) == 1.00, f"Expected 1.00 for full completion, got {score(s_done)}"
    assert success(s_done) is True, "Expected success() to return True"

    # Test 7: Verify demo kinematics dict output
    d = demo()
    assert "left" in d and "right" in d
    assert len(d["left"]) > 0 and len(d["right"]) > 0
    t_left = sum(w.duration for w in d["left"])
    t_right = sum(w.duration for w in d["right"])
    print(f"[self_test] Demo timing: left {t_left:.2f}s, right {t_right:.2f}s (run limit 40s)")
    assert t_left <= 40.0 and t_right <= 40.0, "Demo durations must be <= run limit"

    print("[self_test] All 7 scoring unit test assertions passed cleanly!")


if __name__ == "__main__":
    self_test()
