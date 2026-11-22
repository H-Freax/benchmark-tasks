import sys
from pathlib import Path
_DIR = Path(__file__).resolve().parent
if str(_DIR) not in sys.path:
    sys.path.insert(0, str(_DIR))

from seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas import demo

if __name__ == "__main__":
    plans = demo()
    print("Demo plans generated:", {k: len(v) for k, v in plans.items()})
