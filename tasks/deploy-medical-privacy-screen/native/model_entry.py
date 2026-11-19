"""Standalone modelscene/scorer entry. Never imports thereferencepolicy."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
