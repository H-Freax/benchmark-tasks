"""Yam arena task file redirect for clamp-din-rail-end-stop-with-screwdriver."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from clamp_din_rail_end_stop_with_screwdriver import (
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
