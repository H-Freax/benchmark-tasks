"""Offline unit tests for clamp-din-rail-end-stop-with-screwdriver scoring and mechanics."""

from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

import numpy as np

# Add task dir to sys.path
TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))

import clamp_din_rail_end_stop_with_screwdriver as task
import mechkit as mk


class TestDinRailEndStopScoring(unittest.TestCase):

    def setUp(self):
        mk.MEMORY.clear()
        mk.MEMORY.update({
            "screw_advance_m": 0.0,
            "torque_Nm": 0.0,
            "driver_engaged": False,
            "clamped": False,
            "proof_state": "idle",
            "proof_timer": 0,
            "proof_slip_m": 0.0,
        })

    def make_state(
        self,
        stop_y: float = task.START_Y,
        driver_pos: tuple[float, float, float] = (0.220, 0.180, task.TOP + 0.050),
        driver_engaged: bool = False,
        clamped: bool = False,
        proof_state: str = "idle",
    ):
        moved = {
            "end_stop": (np.array([task.RAIL_X, stop_y, task.RAIL_Z]), np.array([0.0, 0.0, 0.0, 1.0])),
            "screwdriver": (np.array(driver_pos), np.array([0.0, 0.0, 0.0, 1.0])),
        }
        mk.MEMORY["driver_engaged"] = driver_engaged
        mk.MEMORY["clamped"] = clamped
        mk.MEMORY["proof_state"] = proof_state
        if clamped:
            mk.MEMORY["screw_advance_m"] = 0.0007
            mk.MEMORY["torque_Nm"] = 0.45
        return task.kit.fake_states(moved=moved)

    def test_start_state_scores_zero(self):
        objs = self.make_state()
        self.assertEqual(task.score(objs), 0.0)
        self.assertFalse(task.success(objs))

    def test_stage_1_slid_forward(self):
        objs = self.make_state(stop_y=-0.024)
        self.assertEqual(task.score(objs), 0.25)
        self.assertFalse(task.success(objs))

    def test_stage_2_flush_and_engaged(self):
        objs = self.make_state(
            stop_y=task.FLUSH_Y,
            driver_engaged=True,
            clamped=False,
        )
        self.assertEqual(task.score(objs), 0.50)
        self.assertFalse(task.success(objs))

    def test_stage_3_clamped_unreleased(self):
        # Driver at screw location, clamped, but proof not finished
        screw_pos = (task.RAIL_X - 0.010, task.FLUSH_Y, task.TOP + 0.082)
        objs = self.make_state(
            stop_y=task.FLUSH_Y,
            driver_pos=screw_pos,
            driver_engaged=True,
            clamped=True,
            proof_state="idle",
        )
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_stage_4_full_success(self):
        # Clamped, driver returned to rest, proof push held
        driver_at_rest = (0.220, 0.180, task.TOP + 0.040)
        objs = self.make_state(
            stop_y=task.FLUSH_Y,
            driver_pos=driver_at_rest,
            driver_engaged=False,
            clamped=True,
            proof_state="held",
        )
        self.assertEqual(task.score(objs), 1.00)
        self.assertTrue(task.success(objs))

    def test_derailed_scores_zero(self):
        objs = self.make_state(
            stop_y=task.FLUSH_Y,
            clamped=True,
            proof_state="held",
        )
        # Drop bracket off table
        objs = task.kit.fake_states(moved={
            "end_stop": (np.array([task.RAIL_X, task.FLUSH_Y, task.TOP - 0.020]), np.array([0.0, 0.0, 0.0, 1.0]))
        })
        self.assertEqual(task.score(objs), 0.0)

    def test_proof_push_slip_failure(self):
        driver_at_rest = (0.220, 0.180, task.TOP + 0.040)
        objs = self.make_state(
            stop_y=task.FLUSH_Y,
            driver_pos=driver_at_rest,
            clamped=True,
            proof_state="failed",
        )
        self.assertEqual(task.score(objs), 0.0)

    def test_seam_gap_exceeded(self):
        # Bracket stopped 2 mm short of terminal stack
        objs = self.make_state(
            stop_y=task.FLUSH_Y - 0.002,
            clamped=True,
            proof_state="held",
        )
        # Seam gap > 0.5 mm should deny full score (falls back to 0.25)
        self.assertEqual(task.score(objs), 0.25)

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
