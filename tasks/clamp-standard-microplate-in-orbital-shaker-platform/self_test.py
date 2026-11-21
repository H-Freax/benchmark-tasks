"""Offline self-test for clamp_standard_microplate_in_orbital_shaker_platform.

Verifies:
1. Syntax and imports
2. Trajectory IK feasibility across all waypoints
3. Scorer stages, partial credit, and defect latch logic
"""

import os
import sys
import numpy as np

task_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, task_dir)
arena_dir = "/home/freax/Documents/GitHub/yam-isaac-lab-arena"
if arena_dir not in sys.path:
    sys.path.insert(0, arena_dir)

from yam_arena.demo import ArmPlan, LEFT_BASE_POS, RIGHT_BASE_POS
import mechkit as mk
import clamp_standard_microplate_in_orbital_shaker_platform as task

def test_ik_trajectories():
    print("Testing ArmPlan IK reachability across full trajectory...")
    plans = task.demo()
    lp = ArmPlan(plans["left"], LEFT_BASE_POS)
    rp = ArmPlan(plans["right"], RIGHT_BASE_POS)

    t = 0.0
    while t <= max(lp.duration, rp.duration):
        if t <= lp.duration:
            lp.action7(t)
        if t <= rp.duration:
            rp.action7(t)
        t += 0.1

    print(f"  Left arm worst IK: {lp.worst_ik_mm:.2f} mm, {lp.worst_ik_deg:.2f} deg")
    print(f"  Right arm worst IK: {rp.worst_ik_mm:.2f} mm, {rp.worst_ik_deg:.2f} deg")
    assert lp.worst_ik_mm < 5.0, f"Left arm IK error too large: {lp.worst_ik_mm} mm"
    assert rp.worst_ik_mm < 5.0, f"Right arm IK error too large: {rp.worst_ik_mm} mm"
    print("All trajectory IK solutions verified!")

def test_demo_waypoints():
    print("Testing scripted demo waypoints across variants...")
    for variant in ["", "no_seat", "no_clamp", "no_release"]:
        os.environ["DEMO_VARIANT"] = variant
        demo_dict = task.demo()
        assert "left" in demo_dict and "right" in demo_dict
        t_left = sum(w.duration for w in demo_dict['left'])
        t_right = sum(w.duration for w in demo_dict['right'])
        print(f"  Variant '{variant or 'default'}': L={len(demo_dict['left'])}({t_left:.1f}s), R={len(demo_dict['right'])}({t_right:.1f}s)")
        assert abs(t_left - t_right) < 0.1, f"Duration mismatch: {t_left}s vs {t_right}s"
    os.environ.pop("DEMO_VARIANT", None)

def test_scoring_logic():
    print("Testing score stages and anti-cheat defect latch...")
    mk.MEMORY.clear()
    assert task.score({}) == 0.0

    # Defect latch
    mk.MEMORY["defect"] = True
    assert task.score({}) == 0.0
    mk.MEMORY.clear()

    # Stage 1: plate_lifted
    mk.MEMORY["plate_lifted"] = True
    assert abs(task.score({}) - 0.15) < 1e-4

    # Stage 2: plate_seated
    mk.MEMORY["plate_seated"] = True
    assert abs(task.score({}) - 0.35) < 1e-4

    # Stage 3: plate_clamped
    mk.MEMORY["plate_clamped"] = True
    assert abs(task.score({}) - 0.50) < 1e-4

    # Stage 4: shaker_started
    mk.MEMORY["shaker_started"] = True
    assert abs(task.score({}) - 0.75) < 1e-4

    # Stage 5: hold steps
    mk.MEMORY["hold"] = task.HOLD_STEPS_REQUIRED
    assert abs(task.score({}) - 1.00) < 1e-4
    assert task.success({}) is True
    print("Scoring stages verified!")

if __name__ == "__main__":
    test_ik_trajectories()
    test_demo_waypoints()
    test_scoring_logic()
    print("\n>>> ALL OFFLINE SELF-TESTS PASSED! <<<")
