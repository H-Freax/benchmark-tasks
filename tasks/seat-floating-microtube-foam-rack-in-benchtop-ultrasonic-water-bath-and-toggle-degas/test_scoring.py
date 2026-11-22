import pytest
import os
import sys
import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas as task_module
import mechkit as mk


class MockPart:
    def __init__(self, pos=(0, 0, 0), rot=(1, 0, 0, 0)):
        self.pos = np.array(pos, dtype=float)
        self.rot = np.array(rot, dtype=float)


@pytest.fixture(autouse=True)
def reset_mem():
    mk.MEMORY.clear()
    mk._EP["step"] = 0
    yield
    mk.MEMORY.clear()


def test_initial_state_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.RACK_START_X, task_module.RACK_START_Y, task_module.RACK_START_Z)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    assert task_module.score(objects) == 0.00
    assert not task_module.failed(objects)


def test_rack_in_tank_perimeter_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.RACK_START_Z + 0.050)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    mk.MEMORY["rack_pos"] = np.array([task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.RACK_START_Z + 0.050])
    mk.MEMORY["rack_lifted_from_stage"] = True
    assert task_module.score(objects) == 0.25


def test_rack_seated_level_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.TANK_FLOOR_Z + 0.018 + task_module.RACK_H / 2)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    mk.MEMORY["rack_seated_current"] = True
    assert task_module.score(objects) == 0.50


def test_degas_triggered_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.TANK_FLOOR_Z + 0.018 + task_module.RACK_H / 2)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    mk.MEMORY["rack_seated_current"] = True
    mk.MEMORY["lid_closed_current"] = True
    mk.MEMORY["degas_triggered"] = True
    assert task_module.score(objects) == 0.75


def test_hold_passed_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.TANK_FLOOR_Z + 0.018 + task_module.RACK_H / 2)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    mk.MEMORY["rack_seated_current"] = True
    mk.MEMORY["lid_closed_current"] = True
    mk.MEMORY["degas_triggered"] = True
    mk.MEMORY["hold_passed"] = True
    assert task_module.score(objects) == 1.00
    assert task_module.success(objects)


def test_catastrophic_tilt_defect_score():
    objects = {
        "bath_body": MockPart((task_module.BATH_X, task_module.BATH_Y, task_module.TABLE_SURFACE_Z)),
        "foam_rack": MockPart((task_module.TANK_CENTER_X, task_module.TANK_CENTER_Y, task_module.RACK_START_Z)),
        "bath_lid": MockPart((task_module.LID_HINGE_X, task_module.LID_HINGE_Y, task_module.LID_HINGE_Z)),
        "degas_button": MockPart((task_module.BTN_REST_X, task_module.BTN_Y, task_module.BTN_Z)),
    }
    mk.MEMORY["degas_triggered"] = True
    mk.MEMORY["rack_flooded_defect"] = True
    assert task_module.failed(objects)
    assert task_module.score(objects) == 0.00


def test_demo_waypoints():
    plans = task_module.demo()
    assert "left" in plans and "right" in plans
    assert len(plans["left"]) > 0
    assert len(plans["right"]) > 0
