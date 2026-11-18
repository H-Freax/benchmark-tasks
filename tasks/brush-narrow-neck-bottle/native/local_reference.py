"""Cloud cooking backend only; same full physical brush/bottle reference."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from reference import *
import task as rt
from cooking_local import configure
_previous=rt.ng.pk._Finder.targets['yam_arena.scene']
def patch_scene(mod):
 configure(rt.TRACE);_previous(mod)
rt.ng.pk._Finder.targets['yam_arena.scene']=patch_scene
