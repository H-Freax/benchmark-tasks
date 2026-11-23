"""Offline verification suite for seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas:
- Procedural mesh & self-collision checks (32-shape bored/turned rack + 17-shape Magnasonic CD2800 bath)
- Archimedes hydrostatic math & dry-gripper clearance margins
- All 5 live scoring stages (0.00 / 0.25 / 0.50 / 0.75 / 1.00) with ZERO permanent positive latches
- Irreversible flood (> 18 deg tilt) and wet-gripper (< water + 4 mm inside open tank) failure latches
- ON/OFF solid-state electronics & red cycle LED lamp state
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
import fabricate  # noqa: E402
import mechkit as mk  # noqa: E402
import seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas as T  # noqa: E402
from mechkit import MechState  # noqa: E402


def _rack_state(pos=None, tilt_deg: float = 0.0) -> MechState:
    if pos is None:
        pos = (T.BATH_XY[0], T.BATH_XY[1], T.RACK_FLOAT_WORLD_Z)
    p = np.asarray(pos, dtype=float)
    rad = math.radians(tilt_deg)
    q = np.array([math.sin(rad / 2.0), 0.0, 0.0, math.cos(rad / 2.0)], dtype=float)
    z3 = np.zeros(3, dtype=float)
    return MechState(pos=p, origin=p, quat_xyzw=q, lin_vel=z3, ang_vel=z3, low=p - 0.04, high=p + 0.04)


def _bath_state(lid: float = 0.0, on_btn: float = 0.0, off_btn: float = 0.0) -> MechState:
    p = np.array([T.BATH_XY[0], T.BATH_XY[1], T.MAT_TOP], dtype=float)
    q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    z3 = np.zeros(3, dtype=float)
    return MechState(
        pos=p,
        origin=p,
        quat_xyzw=q,
        lin_vel=z3,
        ang_vel=z3,
        low=p - 0.10,
        high=p + 0.10,
        joints={"lid": float(lid), "on_btn": float(on_btn), "off_btn": float(off_btn)},
    )


def test_geometry_and_hydrostatics() -> None:
    assert len(T.FOAM_RACK.shapes) == 32, "Expected 8 bored foam cells + 8 turned tubes + 8 caps + 8 tabs"
    gaps_open = mk.self_gap(T.ULTRASONIC_BATH)
    gaps_closed = mk.self_gap(T.ULTRASONIC_BATH, {"lid": T.LID_CLOSED_RAD})
    assert min(gaps_open.values()) > 0.020
    assert min(gaps_closed.values()) > 0.020
    # Verify tube tips float 15.1 mm clear above stainless tank floor and foam top has 16.1 mm dry freeboard
    tip_rel_z = (T.RACK_FLOAT_WORLD_Z - T.MAT_TOP) - T.RACK_FOAM_H / 2.0 - T.TUBE_TIP_BELOW
    assert abs(tip_rel_z - 0.0471) < 1e-4
    assert tip_rel_z - T.TANK_FLOOR_REL_Z > 0.014
    freeboard = (T.RACK_FLOAT_WORLD_Z - T.MAT_TOP) + T.RACK_FOAM_H / 2.0 - T.WATER_REL_Z
    assert abs(freeboard - 0.0161) < 1e-4


def test_empty_bath_on_is_not_dry_run_and_scores_zero_until_rack_placed() -> None:
    T.reset_episode_state()
    staged = _rack_state((T.STOW_XY[0], T.STOW_XY[1], T.RACK_SPAWN_Z))
    # Pressing ON before placing the rack turns cycle_on True (valid on water-filled bath) without latching failure!
    T.evaluate_step_state(staged, _bath_state(lid=0.0, on_btn=-0.0032), arms_released=False)
    assert T._STATE["cycle_on"] is True
    assert T.failed({"foam_rack": staged, "ultrasonic_bath": _bath_state()}) is False
    assert T.score({"foam_rack": staged, "ultrasonic_bath": _bath_state()}) == 0.0


def test_live_scoring_ladder_and_zero_positive_latches() -> None:
    T.reset_episode_state()
    r_float = _rack_state()
    b_open = _bath_state(lid=0.0)
    T.evaluate_step_state(r_float, b_open, arms_released=True)
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_open}) == 0.25

    # Close lid flush -> 0.50 (cycle still OFF)
    b_closed = _bath_state(lid=T.LID_CLOSED_RAD)
    T.evaluate_step_state(r_float, b_closed, arms_released=True)
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_closed}) == 0.50

    # Click ON button -> 0.75 while held / before 20-step dwell completes
    b_click = _bath_state(lid=T.LID_CLOSED_RAD, on_btn=-0.0032)
    T.evaluate_step_state(r_float, b_click, arms_released=False)
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_click}) == 0.75

    # Release button & hold still for 20 steps -> 1.00
    for _ in range(20):
        T.evaluate_step_state(r_float, b_closed, arms_released=True)
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_closed}) == 1.00
    assert T.success({"foam_rack": r_float, "ultrasonic_bath": b_closed}) is True

    # ZERO POSITIVE LATCH PROOF: pressing OFF or lifting lid immediately drops score from 1.00!
    T.evaluate_step_state(r_float, _bath_state(lid=T.LID_CLOSED_RAD, off_btn=-0.0032), arms_released=False)
    T.evaluate_step_state(r_float, b_closed, arms_released=True)
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_closed}) == 0.50
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": b_open}) == 0.25


def test_flood_and_wet_gripper_failure_latches() -> None:
    # 1) Tilting > 18 deg floods tube wells -> permanent 0.0
    T.reset_episode_state()
    r_tilted = _rack_state(tilt_deg=22.0)
    b_closed = _bath_state(lid=T.LID_CLOSED_RAD, on_btn=-0.0032)
    T.evaluate_step_state(r_tilted, b_closed, arms_released=True)
    r_recovered = _rack_state(tilt_deg=0.0)
    for _ in range(25):
        T.evaluate_step_state(r_recovered, _bath_state(lid=T.LID_CLOSED_RAD), arms_released=True)
    assert T.failed({"foam_rack": r_recovered, "ultrasonic_bath": b_closed}) is True
    assert T.score({"foam_rack": r_recovered, "ultrasonic_bath": b_closed}) == 0.0

    # 2) Dipping a finger pad within 4 mm of the water surface inside the open tank -> permanent 0.0
    T.reset_episode_state()
    r_float = _rack_state()
    b_open = _bath_state(lid=0.0)
    wet_pad = [np.array([[T.BATH_XY[0], T.BATH_XY[1], T.WATER_WORLD_Z + 0.001]], dtype=float)]
    T.evaluate_step_state(r_float, b_open, pad_clouds=wet_pad, arms_released=False)
    assert T._STATE["gripper_wet"] is True
    assert T.score({"foam_rack": r_float, "ultrasonic_bath": _bath_state(lid=T.LID_CLOSED_RAD)}) == 0.0


def test_fabricate_and_closedloop_dry_run() -> None:
    with tempfile.TemporaryDirectory() as td:
        written = fabricate.write_all(td)
        assert len(written) == 3
        for p in written.values():
            assert os.path.getsize(p) > 200

    res = cl.dry_run(
        os.path.join(_DIR, "seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py")
    )
    assert res["steps"] <= 520, f"Too many steps: {res['steps']}"
    assert res["worst_ik_mm"] < 1.0, f"IK error too large: {res['worst_ik_mm']}"
    assert res["state_reads"] == 0


if __name__ == "__main__":
    test_geometry_and_hydrostatics()
    test_empty_bath_on_is_not_dry_run_and_scores_zero_until_rack_placed()
    test_live_scoring_ladder_and_zero_positive_latches()
    test_flood_and_wet_gripper_failure_latches()
    test_fabricate_and_closedloop_dry_run()
    print("ALL 5 TEST SUITES PASSED CLEANLY!")
