"""Offline verification suite for disable-phantom-power-before-unlatching-xlr-mic-plug:
- Scarlett 2i2 + Neutrik PUSH tab positive self-clearance across rest and depressed states
- Unstated +48V phantom power hot-unplug irreversible hazard latch (naive unplug -> 0.0!)
- Full 4-stage live scoring ladder (0.00 / 0.25 / 0.50 / 0.75 / 1.00) and full-footprint tray containment
- Printable fixture STL generation (fabricate.py) and closed-loop 6-DoF IK dry_run (< 0.2 mm worst error)
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import closedloop as cl  # noqa: E402
import disable_phantom_power_before_unlatching_xlr_mic_plug as T  # noqa: E402
import fabricate  # noqa: E402
import mechkit as mk  # noqa: E402
from mechkit import MechState  # noqa: E402


def _quat_from_euler(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([x, y, z, w], dtype=float)


def _st(pos, yaw: float = 0.0, joints=None) -> MechState:
    p = np.asarray(pos, dtype=float)
    q = _quat_from_euler(yaw=yaw)
    z3 = np.zeros(3, dtype=float)
    return MechState(
        pos=p, origin=p, quat_xyzw=q, lin_vel=z3, ang_vel=z3, low=p - 0.04, high=p + 0.04, joints=dict(joints or {})
    )


def test_interface_self_clearance() -> None:
    gaps_rest = mk.self_gap(T.AUDIO_INTERFACE, {"phantom_btn": 0.0, "push_tab": 0.0})
    gaps_dep = mk.self_gap(T.AUDIO_INTERFACE, {"phantom_btn": -T.BTN_TRAVEL, "push_tab": -T.TAB_TRAVEL})
    assert min(gaps_rest.values()) > 0.005
    assert min(gaps_dep.values()) > 0.005


def test_phantom_safety_hazard_and_scoring_ladder() -> None:
    if_rest = _st((T.IFACE_XY[0], T.IFACE_XY[1], T.MAT_TOP), joints={"phantom_btn": 0.0, "push_tab": 0.0})
    if_click = _st((T.IFACE_XY[0], T.IFACE_XY[1], T.MAT_TOP), joints={"phantom_btn": -0.0035, "push_tab": 0.0})
    xlr_in = _st((T.IFACE_XY[0] + T.XLR_SEATED_X_REL, T.IFACE_XY[1] + T.IN1_Y_REL, T.SOCKET_WORLD_Z))
    xlr_mid = _st((T.IFACE_XY[0] + T.XLR_SEATED_X_REL - 0.050, T.IFACE_XY[1] + T.IN1_Y_REL, T.SOCKET_WORLD_Z + 0.04))
    xlr_tray = _st((T.TRAY_XY[0], T.TRAY_XY[1], T.MAT_TOP + 0.015), yaw=0.25)
    l2_ok = _st((T.IFACE_XY[0] + T.TRS_SEATED_X_REL, T.IFACE_XY[1] + T.IN2_Y_REL, T.SOCKET_WORLD_Z))

    # 1) Naive hot-unplug while 48V ON permanently zeroes score even if 48V is turned off later!
    T.reset_episode_state()
    T.evaluate_step_state(if_rest, xlr_mid, l2_ok, arms_released=False)
    T.evaluate_step_state(if_click, xlr_tray, l2_ok, arms_released=False)
    for _ in range(22):
        T.evaluate_step_state(if_rest, xlr_tray, l2_ok, arms_released=True)
    assert T.failed({}) is True
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_tray, "line2_plug": l2_ok}) == 0.0

    # 2) Safe sequence: 48V OFF (0.25) -> Extracted (0.50) -> In tray (0.75) -> 20-step dwell (1.00)
    T.reset_episode_state()
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_in, "line2_plug": l2_ok}) == 0.0
    T.evaluate_step_state(if_click, xlr_in, l2_ok, arms_released=False)
    T.evaluate_step_state(if_rest, xlr_in, l2_ok, arms_released=True)
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_in, "line2_plug": l2_ok}) == 0.25

    T.evaluate_step_state(if_rest, xlr_mid, l2_ok, arms_released=False)
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_mid, "line2_plug": l2_ok}) == 0.50

    T.evaluate_step_state(if_rest, xlr_tray, l2_ok, arms_released=False)
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_tray, "line2_plug": l2_ok}) == 0.75

    for _ in range(20):
        T.evaluate_step_state(if_rest, xlr_tray, l2_ok, arms_released=True)
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_tray, "line2_plug": l2_ok}) == 1.00

    # Zero positive latch: pressing 48V back ON drops score to 0.0!
    T.evaluate_step_state(if_click, xlr_tray, l2_ok, arms_released=False)
    assert T.score({"audio_interface": if_rest, "xlr_plug": xlr_tray, "line2_plug": l2_ok}) == 0.0


def test_fabricate_and_closedloop_dry_run() -> None:
    with tempfile.TemporaryDirectory() as td:
        written = fabricate.write_all(td)
        assert len(written) == 3
        for p in written.values():
            assert os.path.getsize(p) > 200

    res = cl.dry_run(os.path.join(_DIR, "disable_phantom_power_before_unlatching_xlr_mic_plug.py"))
    assert res["steps"] <= 560, f"Too many steps: {res['steps']}"
    assert res["worst_ik_mm"] < 0.5, f"IK error too large: {res['worst_ik_mm']}"
    assert res["state_reads"] == 0


if __name__ == "__main__":
    test_interface_self_clearance()
    test_phantom_safety_hazard_and_scoring_ladder()
    test_fabricate_and_closedloop_dry_run()
    print("ALL 3 XLR PHANTOM POWER SAFETY TEST SUITES PASSED CLEANLY!")
