# Lift a small cabinet door off its slip-joint hinges and stand it on a bench pad

Instruction given to the policy: `Remove the open cabinet door and stand its bottom edge on the green rubber pad. Let go and move both hands clear.`

Files: `task.json`, `task.py` (scene declaration, `Scorer`, `score()`, `success()`, and scripted `demo()`), `bridge.py` (deferred USD spawning, filtered door-to-pad contact sensor, and 6-DoF waypoint interpolation), `geometry.py` (authoritative metre-scale cut-list and deterministic USDA generator), `assets/*.usda`, `inventory.json`, `SOURCES.md`, `LICENSE`, `check_assets.py`, `run_native.py`, `test_task.py`, `README.md`.

## What is modelled

- A real-size small storage-cabinet door, 350 mm wide x 400 mm tall x 18 mm thick (poplar plywood, 1.267 kg including a 202 mm horizontal stainless pull bar and two moving hinge halves), starting 90 degrees open on a fixed 240 x 391 x 420 mm plywood cabinet carcase clamped to the bench.
- Two Sugatsune NH-40L/SS-class 304-stainless lift-off slip-joint hinges (catalogue outer envelope 40 x 30 x 7 mm each, vertical axes, shoulder planes 260 mm apart at 50 mm and 310 mm above the door bottom). Each fixed lower half has a 7.0 mm shoulder and an upward 3.0 mm diameter x 18 mm solid cylindrical pin; each moving upper half has a 20 mm hollow annular knuckle (3.6 mm bore, 0.30 mm radial running clearance, built from 24 convex wedge colliders so convex cooking keeps the bore open).
- Gravity seats the knuckles on the shoulders (reset spawns the door 0.2 mm above the shoulder plane so it settles cleanly under gravity without startup narrow-phase artefacts). Pulling horizontally while seated jams the knuckles against the vertical pins; both arms must lift the door straight up clear of both 18 mm pins, carry it clear of the carcase, and stand its bottom edge upright on the flat 140 x 410 x 4 mm green nonslip rubber pad (`(0.250, 0, 0.7495) m`, no slot, cradle, or guide). No joint, weld, proximity release, or object pose write is used.
- Internal pin/bore machining dimensions (3.0 mm pin, 3.6 mm bore, 18 mm engagement, 20 mm knuckle halves) and friction coefficients (pad 0.8/0.64, wood/pull 0.5/0.4, fixed hinge/carcase 0.2/0.16) are disclosed engineering approximations; see `SOURCES.md` and `inventory.json` for full procurement references and cut-list coordinates.

## Scoring

`SCORE_OVER = 'end'` (current physical progress; dropping the door off the bench or re-engaging a pin loses earlier credit; invalid/non-finite readings return `0.0`):

- `0.00`: either hinge pin still occupies either moving knuckle, door off the bench, or invalid measurement.
- `0.25`: both hinge pins completely clear of both moving knuckles above the bench footprint.
- `0.50`: detached and door origin moved at least 80 mm horizontally away from its seated hinge origin.
- `0.75`: additionally safely supported (on the bare bench within 1 mm of the tabletop without overlapping the pad, or on the pad within 3 mm with upward pad contact force >= 85% of door weight = 10.56 N), released (every URDF collision hull of both robots >= 30 mm clear of the door AABB), and still (linear speed < 10 mm/s, angular speed < 0.05 rad/s, max vertex displacement <= 2 mm) continuously for 1.0 s (>= 21 samples at 20 Hz).
- `1.00` (`success`): the `0.75` window satisfied with the complete door footprint inside the 140 x 410 mm pad, door upright within 5 degrees of vertical, all four panel bottom corners within `[-1 mm, +3 mm]` of the pad top, and upward door-to-pad contact force >= 85% of door weight throughout the 1.0 s window.

## Environment

Isaac Lab-Arena 0.3.1 with `yam-arena` and `inspect-robots`.

## Offline checks, no GPU

```bash
PYTHONDONTWRITEBYTECODE=1 python -B tasks/lift-off-cabinet-door-to-pad/geometry.py
PYTHONDONTWRITEBYTECODE=1 python -B -m pytest -p no:cacheprovider tasks/lift-off-cabinet-door-to-pad/test_task.py -q
PYTHONDONTWRITEBYTECODE=1 python -B tasks/lift-off-cabinet-door-to-pad/run_native.py --cpu-check
uv run --python 3.12 --with usd-core==26.11 --no-project python -B tasks/lift-off-cabinet-door-to-pad/check_assets.py
```

## Run and score one trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lift-off-cabinet-door-to-pad/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Remove the open cabinet door and stand its bottom edge on the green rubber pad. Let go and move both hands clear." \
  --scorer task_score --max-steps 800 --max-action-delta 0.12 --seed 0 --no-prompt --store-frames --log-dir runs/lift-off-door
```

For Dream-1 use `--policy dream` with the same arguments. Set `LIFT_OFF_CABINET_DEMO=noop|pull_only|hover|held|off_pad` with `--policy yam_scripted` to run the comparison controls.
