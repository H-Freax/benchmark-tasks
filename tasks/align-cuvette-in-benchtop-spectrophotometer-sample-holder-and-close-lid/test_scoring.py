#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

task_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(task_dir))
sys.path.insert(0, "/home/freax/Documents/GitHub/yam-isaac-lab-arena")

import align_cuvette_in_benchtop_spectrophotometer_sample_holder_and_close_lid as task
import mechkit as mk


class TestScoring(unittest.TestCase):
    def setUp(self):
        mk.MEMORY.clear()

    def test_initial_state_zero(self):
        # Cuvette in rack, lid open at 65 deg
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(65.0)},
            moved={
                "optical_cuvette": (
                    (task.CUVETTE_INIT_X, task.CUVETTE_INIT_Y, task.CUVETTE_INIT_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        task._update_mechanics(states)
        self.assertEqual(task.score(states), 0.00)

    def test_cuvette_lifted_scores_stage1(self):
        # Cuvette lifted above rack floor (Z >= RACK_FLOOR_Z + 0.040)
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(65.0)},
            moved={
                "optical_cuvette": (
                    (task.RACK_X, task.RACK_Y, task.RACK_FLOOR_Z + 0.060),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        task._update_mechanics(states)
        self.assertEqual(task.score(states), 0.25)

    def test_cuvette_seated_scores_stage2(self):
        # Cuvette seated in spectrophotometer socket, but lid still open
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(65.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        task._update_mechanics(states)
        self.assertEqual(task.score(states), 0.50)

    def test_lid_closed_scores_stage3(self):
        # Cuvette seated, lid closed flush (0 deg), hold_steps < 20
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(0.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        task._update_mechanics(states)
        self.assertEqual(task.score(states), 0.75)

    def test_full_hold_scores_1_0(self):
        # 20 consecutive steps of complete physical state
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(0.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        for _ in range(20):
            task._update_mechanics(states)
        self.assertEqual(task.score(states), 1.00)
        self.assertTrue(task.success(states))

    def test_reopening_lid_drops_score(self):
        # Verify ZERO permanent positive latching:
        # After reaching 1.00, if lid is re-opened, score immediately drops to 0.50!
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(0.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        for _ in range(20):
            task._update_mechanics(states)
        self.assertEqual(task.score(states), 1.00)

        # Reopen lid
        states_open = task.kit.fake_states(
            q={"lid_hinge": math.radians(35.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
            },
        )
        task._update_mechanics(states_open)
        self.assertEqual(task.score(states_open), 0.50)
        self.assertEqual(mk.MEMORY.get("hold_steps", 0), 0)

    def test_misaligned_cuvette_drops_score(self):
        # Cuvette seated in socket and lid closed, BUT cuvette rotated 90 deg (yaw = pi/2)
        # Optical windows face +/- X instead of light beam axis (+/- Y)
        # Quat for 90 deg around Z: (0, 0, sin(45), cos(45)) = (0, 0, 0.7071068, 0.7071068)
        states = task.kit.fake_states(
            q={"lid_hinge": math.radians(0.0)},
            moved={
                "optical_cuvette": (
                    (task.HOLDER_X, task.HOLDER_Y, task.CUVETTE_SEATED_Z),
                    (0.0, 0.0, 0.70710678, 0.70710678),
                )
            },
        )
        task._update_mechanics(states)
        # Since optical_aligned is False, stage 2, 3, and 4 cannot be achieved. Score should be 0.25 (or in transit/lifted) or 0.0
        self.assertLess(task.score(states), 0.50)
        self.assertEqual(mk.MEMORY.get("hold_steps", 0), 0)


if __name__ == "__main__":
    unittest.main()
