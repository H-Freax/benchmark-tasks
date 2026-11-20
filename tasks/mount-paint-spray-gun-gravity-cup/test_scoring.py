"""Offline unit tests for mount-paint-spray-gun-gravity-cup scoring and mechanics."""

from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

import numpy as np

# Add task dir to sys.path
TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))

import mount_paint_spray_gun_gravity_cup as task
import mechkit as mk


class TestMountPaintSprayGunGravityCupScoring(unittest.TestCase):

    def setUp(self):
        mk.MEMORY.clear()
        mk.MEMORY.update({
            "cup_aligned": False,
            "cup_seated": False,
            "ring_locked": False,
            "proof_state": "idle",
            "proof_timer": 0,
            "proof_slip_m": 0.0,
            "locked_joint_engaged": False,
        })

    def make_state(
        self,
        cup_pos: tuple[float, float, float] = (task.CUP_START_X, task.CUP_START_Y, task.CUP_START_Z),
        cup_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        gun_pos: tuple[float, float, float] = (task.GUN_X, task.GUN_Y, task.GUN_STAND_Z),
        ring_pos: tuple[float, float, float] = (task.INLET_X, task.INLET_Y, task.RING_Z),
        cup_aligned: bool = False,
        cup_seated: bool = False,
        ring_locked: bool = False,
        proof_state: str = "idle",
    ):
        moved = {
            "spray_gun": (np.array(gun_pos), np.array([0.0, 0.0, 0.0, 1.0])),
            "paint_cup": (np.array(cup_pos), np.array(cup_quat)),
            "locking_ring": (np.array(ring_pos), np.array([0.0, 0.0, 0.0, 1.0])),
        }
        mk.MEMORY["cup_aligned"] = cup_aligned
        mk.MEMORY["cup_seated"] = cup_seated
        mk.MEMORY["ring_locked"] = ring_locked
        mk.MEMORY["proof_state"] = proof_state
        return task.kit.fake_states(moved=moved)

    def test_start_state_scores_zero(self):
        objs = self.make_state()
        self.assertEqual(task.score(objs), 0.0)
        self.assertFalse(task.success(objs))

    def test_stage_1_cup_aligned(self):
        # Hovering 30 mm above inlet
        hover_pos = (task.INLET_X, task.INLET_Y, task.CUP_SEATED_Z + 0.030)
        objs = self.make_state(cup_pos=hover_pos, cup_aligned=True)
        self.assertEqual(task.score(objs), 0.25)
        self.assertFalse(task.success(objs))

    def test_stage_2_cup_seated(self):
        # Fully seated in inlet, but ring unlocked
        seated_pos = (task.INLET_X, task.INLET_Y, task.CUP_SEATED_Z)
        objs = self.make_state(
            cup_pos=seated_pos,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=False,
        )
        self.assertEqual(task.score(objs), 0.50)
        self.assertFalse(task.success(objs))

    def test_stage_3_ring_locked_unreleased(self):
        # Cup seated, ring locked, but proof not yet complete
        seated_pos = (task.INLET_X, task.INLET_Y, task.CUP_SEATED_Z)
        objs = self.make_state(
            cup_pos=seated_pos,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=True,
            proof_state="idle",
        )
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_stage_4_full_success(self):
        # Seated, ring locked, proof held, arms clear
        seated_pos = (task.INLET_X, task.INLET_Y, task.CUP_SEATED_Z)
        objs = self.make_state(
            cup_pos=seated_pos,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=True,
            proof_state="held",
        )
        self.assertEqual(task.score(objs), 1.00)
        self.assertTrue(task.success(objs))

    def test_dropped_off_table_scores_zero(self):
        # Cup fell below table level
        dropped_pos = (task.INLET_X, task.INLET_Y, task.TOP - 0.030)
        objs = self.make_state(
            cup_pos=dropped_pos,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=True,
            proof_state="held",
        )
        self.assertEqual(task.score(objs), 0.0)

    def test_overturned_cup_scores_zero(self):
        # Cup tilted 90 degrees (on its side)
        # 90 deg rotation about X axis: quat = [sin(45), 0, 0, cos(45)]
        tilted_quat = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
        objs = self.make_state(
            cup_quat=tilted_quat,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=True,
            proof_state="held",
        )
        self.assertEqual(task.score(objs), 0.0)

    def test_proof_failure_scores_zero(self):
        # Cup popped out during proof tug
        seated_pos = (task.INLET_X, task.INLET_Y, task.CUP_SEATED_Z)
        objs = self.make_state(
            cup_pos=seated_pos,
            cup_aligned=True,
            cup_seated=True,
            ring_locked=True,
            proof_state="failed",
        )
        self.assertEqual(task.score(objs), 0.0)

    def test_demo_waypoints_kinematics(self):
        traj = task.demo()
        self.assertIn("left", traj)
        self.assertIn("right", traj)
        self.assertGreater(len(traj["left"]), 0)
        self.assertGreater(len(traj["right"]), 0)

        # Verify all waypoint coordinates are within reachable tabletop bounds
        for arm in ["left", "right"]:
            for wp in traj[arm]:
                x, y, z = wp.xyz
                self.assertGreater(x, 0.05, f"{arm} {wp.label} x too close to base")
                self.assertLess(x, 0.50, f"{arm} {wp.label} x too far")
                self.assertGreater(y, -0.45, f"{arm} {wp.label} y too far right")
                self.assertLess(y, 0.45, f"{arm} {wp.label} y too far left")
                self.assertGreater(z, task.TOP, f"{arm} {wp.label} z penetrates table")
                self.assertLess(z, task.TOP + 0.35, f"{arm} {wp.label} z too high")


if __name__ == "__main__":
    unittest.main()
