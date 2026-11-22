import unittest
import numpy as np
import sys
from pathlib import Path
from dataclasses import dataclass

# Add task dir to sys.path
task_dir = Path(__file__).parent
sys.path.insert(0, str(task_dir))

import mechkit as mk
import position_and_clamp_steel_bar_in_benchtop_drill_press_vise as task

@dataclass
class MockState:
    pos: np.ndarray
    low: np.ndarray
    high: np.ndarray

class TestViseScoring(unittest.TestCase):
    def setUp(self):
        task.on_reset()

    def _mock_objects(self, bar_pos, jaw_x, handle_x):
        b_p = np.array(bar_pos, dtype=float)
        j_p = np.array([jaw_x, task.VISE_XY[1], task.BED_Z], dtype=float)
        h_p = np.array([handle_x, task.HANDLE_Y0, task.HANDLE_Z], dtype=float)
        return {
            "workpiece_bar": MockState(pos=b_p, low=b_p - 0.01, high=b_p + 0.01),
            "moving_jaw": MockState(pos=j_p, low=j_p - 0.01, high=j_p + 0.01),
            "vise_handle": MockState(pos=h_p, low=h_p - 0.01, high=h_p + 0.01),
        }

    def test_initial_state_zero(self):
        objs = self._mock_objects(
            bar_pos=[task.BAR_INIT_XY[0], task.BAR_INIT_XY[1], task.BAR_INIT_Z],
            jaw_x=task.MOVING_JAW_X0,
            handle_x=task.HANDLE_X0,
        )
        task._on_step(objs)
        self.assertEqual(task.score(objs), 0.0)

    def test_bar_lifted_score_020(self):
        objs = self._mock_objects(
            bar_pos=[task.BAR_INIT_XY[0], task.BAR_INIT_XY[1], task.BAR_INIT_Z + 0.050],
            jaw_x=task.MOVING_JAW_X0,
            handle_x=task.HANDLE_X0,
        )
        task._on_step(objs)
        self.assertEqual(task.score(objs), 0.20)

    def test_bar_seated_score_050(self):
        objs = self._mock_objects(
            bar_pos=[task.BAR_TARGET_X, task.BAR_TARGET_Y, task.BAR_TARGET_Z],
            jaw_x=task.MOVING_JAW_X0,
            handle_x=task.HANDLE_X0,
        )
        mk.MEMORY["bar_lifted"] = True
        task._on_step(objs)
        self.assertEqual(task.score(objs), 0.50)

    def test_bar_clamped_score_085(self):
        objs = self._mock_objects(
            bar_pos=[task.BAR_TARGET_X, task.BAR_TARGET_Y, task.BAR_TARGET_Z],
            jaw_x=task.MOVING_JAW_X0 + 0.017,
            handle_x=task.HANDLE_X0 + 0.017,
        )
        mk.MEMORY["bar_lifted"] = True
        mk.MEMORY["bar_seated"] = True
        task._on_step(objs)
        self.assertEqual(task.score(objs), 0.85)

    def test_full_hold_score_100(self):
        objs = self._mock_objects(
            bar_pos=[task.BAR_TARGET_X, task.BAR_TARGET_Y, task.BAR_TARGET_Z],
            jaw_x=task.MOVING_JAW_X0 + 0.017,
            handle_x=task.HANDLE_X0 + 0.017,
        )
        mk.MEMORY["bar_lifted"] = True
        mk.MEMORY["bar_seated"] = True
        mk.MEMORY["clamped"] = True
        mk.MEMORY["stillness_steps"] = 20
        self.assertEqual(task.score(objs), 1.00)

if __name__ == "__main__":
    unittest.main()
