"""Unit tests for drill press vise positioning and clamping scorer."""
import unittest
import numpy as np

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk
import position_and_clamp_steel_bar_in_benchtop_drill_press_vise as task

class TestViseScoring(unittest.TestCase):
    def setUp(self):
        task.on_reset()
        self.objects = {}
        # Nominal target positions
        self.bar_pos = np.array([task.BAR_TARGET_X, task.BAR_TARGET_Y, task.BAR_TARGET_Z])
        self.jaw_pos = np.array([task.MOVING_JAW_X0, task.VISE_XY[1], task.BED_Z])
        self.handle_pos = np.array([task.HANDLE_X0, task.HANDLE_Y0, task.HANDLE_Z])

    def _make_state(self, pos, low=None, high=None, quat=None):
        pos = np.array(pos, dtype=float)
        low = pos - 0.01 if low is None else np.array(low, dtype=float)
        high = pos + 0.01 if high is None else np.array(high, dtype=float)
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float) if quat is None else np.array(quat, dtype=float)
        return mk.ObjectState(
            pos=pos,
            quat_xyzw=quat,
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
            low=low,
            high=high,
        )

    def test_initial_state_zero_score(self):
        """Initial staging rack pose should score 0.0."""
        objects = {
            "workpiece_bar": self._make_state([task.BAR_INIT_XY[0], task.BAR_INIT_XY[1], task.BAR_INIT_Z]),
            "moving_jaw": self._make_state(self.jaw_pos),
            "vise_handle": self._make_state(self.handle_pos),
        }
        self.assertEqual(task.score(objects), 0.00)

    def test_lift_score(self):
        """Lifting the bar clear of rack (z >= 0.805m) should score 0.20."""
        objects = {
            "workpiece_bar": self._make_state([task.BAR_INIT_XY[0], task.BAR_INIT_XY[1], 0.810]),
            "moving_jaw": self._make_state(self.jaw_pos),
            "vise_handle": self._make_state(self.handle_pos),
        }
        self.assertEqual(task.score(objects), 0.20)

    def test_seated_on_parallels_score(self):
        """Bar seated level on parallels within tolerances should score 0.50."""
        objects = {
            "workpiece_bar": self._make_state(self.bar_pos),
            "moving_jaw": self._make_state(self.jaw_pos),
            "vise_handle": self._make_state(self.handle_pos),
        }
        self.assertEqual(task.score(objects), 0.50)

    def test_clamped_score(self):
        """Moving jaw and handle advanced >= 16.5 mm into bar should score 0.85."""
        objects = {
            "workpiece_bar": self._make_state(self.bar_pos),
            "moving_jaw": self._make_state([task.MOVING_JAW_X0 + 0.017, self.jaw_pos[1], self.jaw_pos[2]]),
            "vise_handle": self._make_state([task.HANDLE_X0 + 0.017, self.handle_pos[1], self.handle_pos[2]]),
        }
        self.assertEqual(task.score(objects), 0.85)

    def test_excessive_tilt_blocks_seated_score(self):
        """Bar tilted > 3.0 deg should not score parallel seating credit."""
        # 5 deg tilt around X axis while hovering above parallels
        angle_rad = np.radians(5.0)
        q = np.array([np.sin(angle_rad / 2), 0.0, 0.0, np.cos(angle_rad / 2)])
        objects = {
            "workpiece_bar": self._make_state([task.BAR_TARGET_X, task.BAR_TARGET_Y, 0.810], quat=q),
            "moving_jaw": self._make_state(self.jaw_pos),
            "vise_handle": self._make_state(self.handle_pos),
        }
        # Tilted above parallel tolerance scores 0.20 (lift credit) instead of 0.50 (seated credit)
        self.assertEqual(task.score(objects), 0.20)

    def test_dropped_fail(self):
        """Dropping the bar below table height fails immediately to 0.0."""
        objects = {
            "workpiece_bar": self._make_state([task.BAR_TARGET_X, task.BAR_TARGET_Y, task.TOP - 0.10]),
            "moving_jaw": self._make_state(self.jaw_pos),
            "vise_handle": self._make_state(self.handle_pos),
        }
        self.assertEqual(task.score(objects), 0.00)

if __name__ == "__main__":
    unittest.main()
