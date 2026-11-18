"""Drain water from beneath oil, stop at the visible interface and leave safe."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import native as rt
OBJECTS=rt.OBJECTS
SCENE='station'
SCORE_OVER='end'
CONTROL_HZ=20.
SUCCESS_HOLD_S=1.

def score(objects):return rt.LAST_ROW.get('score',0.)if rt.LAST_ROW else 0.
def success(objects):return score(objects)>=1.
