"""Causal negative: ordinary key motions with the top stopper untouched.

Same scene, passive hinge, fluid model and grader as V7. No state setters or
score override. Root may run at most 1000 controls. Judge airlock using actual
stopper/vented/key-angle/recovery observations, not score zero alone.
"""
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from scripted import OBJECTS, SCENE, SCORE_OVER, CONTROL_HZ, SUCCESS_HOLD_S, score, success, demo
import scripted

scripted.CTRL.sequence = [
    ('settle', 25), ('key_hover', 45), ('key_approach', 35),
    ('key_close', 30), ('key_open', 50), ('no_vent_hold', 300),
    ('key_close_valve', 45), ('key_closed_settle', 60),
    ('key_release', 100), ('key_withdraw', 40), ('right_retreat', 40),
    ('final_settle', 40),
]
