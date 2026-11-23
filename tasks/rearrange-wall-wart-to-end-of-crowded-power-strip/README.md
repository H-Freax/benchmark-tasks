# Move a bulky wall-wart power brick to the end outlet of a crowded power strip so a fourth cord can plug in

`rearrange_wall_wart_to_end_of_crowded_power_strip.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A free-standing `4-outlet`
NEMA `5-15R` power strip (`222 x 54 x 30 mm`, `360 g`, unwired) rests on a desk mat at `(24.5, -2.0) cm` with two
already-connected appliance plugs in Outlets `1` and `2` and a bulky white `AC` wall-wart adapter brick (`36 x 54 x 40 mm`,
`105 g`) in Outlet `3` whose `44 mm` `-y` body overhang completely covers end Outlet `4`. On the right at `(24.5, -18.5) cm`,
a loose black appliance cord's `3-prong` NEMA `5-15P` plug (`36 x 24 x 38 mm`, `64 g`) rests prongs-down on a teal desktop
cord cradle (`2.5 mm` ground-pin clearance above the mat). Given only the natural instruction *"Plug the black cord on the
right into the power strip, leaving everything already on the strip plugged in"*, the left arm braces the free-standing
power strip while the right arm pulls the bulky wall-wart out of Outlet `3`, moves it to end Outlet `4` (uncovering Outlet
`3`), and seats the black cord's `3-prong` plug flush into Outlet `3`.

Files:
- `rearrange_wall_wart_to_end_of_crowded_power_strip.py` (scene, 4 x 3-cavity NEMA `5-15R` strip, chamfered `3-prong` NEMA `5-15P` plugs, asymmetric overhanging wall-wart, 5-tier live scorer, closed-loop 6-DoF bimanual demo)
- `test_scoring.py` (offline 3-prong socket clearance vs `180 deg` ground-pin rejection, wall-wart overhang collision in Outlet `3` vs clearance in Outlet `4`, 5-tier scorer, failure latches, and 6-DoF IK dry-run checks)
- `mechkit.py`, `closedloop.py`, `belimits.py`, `gripkit.py` (procedural convex-hull geometry, rubber pad friction, and 6-DoF arm control utilities)

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rearrange-wall-wart-to-end-of-crowded-power-strip/rearrange_wall_wart_to_end_of_crowded_power_strip.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Plug the black cord on the right into the power strip, leaving everything already on the strip plugged in." \
  --scorer task_score --max-steps 600 --seed 0 --store-frames --log-dir logs/crowded-power-strip-demo
```

- Model run (Dream-1): the same command with `--policy dream`.
- Offline test suite: `python3 tasks/rearrange-wall-wart-to-end-of-crowded-power-strip/test_scoring.py`.
- Scorer checks (`DEMO_VARIANT`):
  - `DEMO_VARIANT=unplug_existing` pulls the pre-seated blue appliance plug out of Outlet `1` (`0.00`, `existing_unplugged` latch).
  - `DEMO_VARIANT=move_wart_only` moves the bulky wall-wart from Outlet `3` to end Outlet `4` without plugging in the black cord (`0.50`).

## Results

- Scripted demo (`yam_scripted`): `1.0` (`seed=0` in `442` steps, `seed=1`: `1.0`).
- Model run (`dream`, `seed=0`): `0.0` (`600` steps — reaches directly toward the black plug on the right cradle without moving the overhanging wall-wart from Outlet `3` to Outlet `4`).
- Scorer checks: `unplug_existing` `0.00`, `move_wart_only` `0.50`.

## Why it matters (`Societal relatability`: everyday crowded power-strip rearrangement)

Rearranging a bulky router, charger, or speaker wall-wart to the end socket of a power strip so its overhanging housing
stops blocking an adjacent socket—without unplugging devices that are already running or dragging the light strip across
the desk—is an everyday household and office manipulation puzzle combining unstated spatial reasoning, bimanual
stabilization, and `3-prong` plug insertion.

## Simulation, NEMA 5-15 geometry, and retail parts

| Part | Value | Source, or setup check |
|---|---|---|
| `4-outlet` power strip | `222 x 54 x 30 mm` unwired NEMA `5-15R` strip (`360 +/- 5 g`, `34.0 mm` outlet pitch) | Woods / GE `4-outlet` strip (`$8.99`, unwired); `3-cavity` polarized + ground slots per outlet |
| Bulky `AC` wall-wart | `36 x 54 x 40 mm` (`105 +/- 3 g`), prongs `10 mm` from `+y` end (`44 mm` `-y` overhang) | TP-Link `12V` adapter housing (unwired); covers Outlet `4` when in `3`, clears Outlet `3` by `24 mm` when in `4` |
| Molded `NEMA 5-15P` plugs | `30 x 25 x 28 mm` (`48 g`, blue & grey) and `36 x 24 x 38 mm` (`64 g`, black), `14.5 mm` blades + `16.0 mm` ground pin | Blue (`Outlet 1`), grey (`Outlet 2`), and black loose cord (`<= 5.0 mm` seated face-gap check) |
| Desktop cord cradle | Fixed teal cradle (`46 x 44 x 18.5 mm` side ledges) at `(24.5, -18.5) cm` | Holds black plug prongs-down `2.5 mm` clear of mat |

## Runtime and seeds

`object_jitter=0` fixed layout; run limit `600` control steps (`30.0 s` at `20 Hz`, ~75 s wall-clock per rollout on a
single RTX 3090 GPU). Verified with `--seed 0` and `--seed 1`.

## Sources and licenses

- `rearrange_wall_wart_to_end_of_crowded_power_strip.py` and `test_scoring.py` were authored by this task's team.
- `mechkit.py`, `closedloop.py`, `belimits.py`, and `gripkit.py` are adapted from MIT-licensed utilities in `robocurve/benchmark-tasks` (`tasks/remove-windscreen-before-lighting-canister-stove` and `tasks/fit-humidifier-onto-oxygen-flowmeter`).
- Robot link hulls load at runtime from `yam_arena` and are not redistributed; no third-party binary meshes, images, or videos are committed to git.
