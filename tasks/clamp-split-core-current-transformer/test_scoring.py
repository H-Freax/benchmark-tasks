"""Offline unit tests for clamp-split-core-current-transformer scoring and mechanics."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import unittest

import numpy as np

TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))

import clamp_split_core_current_transformer as task
import mechkit as mk


class TestSplitCoreCTScoring(unittest.TestCase):

    def setUp(self):
        mk.MEMORY.clear()
        task._TEST_ROBOT_BOXES = [((-1.0, -1.0, -1.0), (-0.9, -0.9, -0.9))]

    def test_start_state_scores_zero(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_INIT_Y, "core_hinge": -45.0}, deg=True)
        self.assertEqual(task.score(objs), 0.0)
        self.assertFalse(task.success(objs))

    def test_stage_1_partial_compression(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": -25.0}, deg=True)
        self.assertEqual(task.score(objs), 0.25)
        self.assertFalse(task.success(objs))

    def test_stage_2_core_closed(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
        self.assertEqual(task.score(objs), 0.50)
        self.assertFalse(task.success(objs))

    def test_stage_3_latch_engaged(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
        mk.MEMORY.update({"latch_engaged": True})
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_stage_4_full_success(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
        mk.MEMORY.update({"latch_engaged": True, "proof_passed": True})
        self.assertEqual(task.score(objs), 1.00)
        self.assertTrue(task.success(objs))

    def test_hand_intrusion_denies_full_success(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
        mk.MEMORY.update({"latch_engaged": True, "proof_passed": True})
        task._TEST_ROBOT_BOXES = [((task.CABLE_X - 0.010, -0.010, task.CABLE_Z - 0.010),
                                   (task.CABLE_X + 0.010, 0.010, task.CABLE_Z + 0.010))]
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_dropped_off_table_scores_zero(self):
        objs = task.kit.fake_states(
            q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True,
            moved={"ct_base": ((task.CABLE_X, 0.0, task.TOP - 0.100), (0, 0, 0, 1))}
        )
        mk.MEMORY.update({"fell": True})
        self.assertEqual(task.score(objs), 0.0)

    def test_proof_pull_unpassed_caps_at_stage_3(self):
        objs = task.kit.fake_states(q={"ct_slide": task.CT_GOAL_Y, "core_hinge": 0.0}, deg=True)
        mk.MEMORY.update({"latch_engaged": True, "proof_passed": False})
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_demo_waypoint_kinematics_and_durations(self):
        for variant in ("demo", "slide_only", "close_no_latch", "noop"):
            os.environ["CT_DEMO"] = variant if variant != "demo" else ""
            plan = task.demo()
            self.assertIn("left", plan)
            self.assertIn("right", plan)
            t_l = sum(w.duration for w in plan["left"])
            t_r = sum(w.duration for w in plan["right"])
            self.assertAlmostEqual(t_l, t_r, places=4)

            for arm in ("left", "right"):
                for wp in plan[arm]:
                    x, y, z = wp.pos
                    self.assertGreater(x, 0.05, f"{arm} {wp.label} x too close to base")
                    self.assertLess(x, 0.55, f"{arm} {wp.label} x too far")
                    self.assertGreater(y, -0.45, f"{arm} {wp.label} y too far right")
                    self.assertLess(y, 0.45, f"{arm} {wp.label} y too far left")
                    self.assertGreater(z, task.TOP - 0.10, f"{arm} {wp.label} z penetrates table")
                    self.assertLess(z, task.TOP + 0.35, f"{arm} {wp.label} z too high")


if __name__ == "__main__":
    unittest.main()
