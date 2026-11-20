"""Yam arena task file redirect for mount-paint-spray-gun-gravity-cup."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mount_paint_spray_gun_gravity_cup import (
    INSTRUCTION,
    OBJECTS,
    SCENE,
    SCORE_OVER,
    SUCCESS_HOLD_S,
    demo,
    score,
    success,
)

__all__ = [
    "INSTRUCTION",
    "OBJECTS",
    "SCENE",
    "SCORE_OVER",
    "SUCCESS_HOLD_S",
    "demo",
    "score",
    "success",
]
