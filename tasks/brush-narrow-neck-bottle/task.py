"""Task entry for ordinary model policies; reference controller is separate."""
from pathlib import Path
import runpy
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'native'))
globals().update({k:v for k,v in runpy.run_path(str(HERE/'native/task.py')).items()if not k.startswith('__')})
