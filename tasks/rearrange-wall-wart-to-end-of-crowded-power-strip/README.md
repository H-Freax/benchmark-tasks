# Move a bulky wall-wart adapter to the end outlet of a crowded power strip so a fourth cord can plug in

`rearrange_wall_wart_to_end_of_crowded_power_strip.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. An unwired `4-outlet`
longitudinal NEMA `5-15R` power strip (**CyberPower CSB404**, `214 x 50 x 35 mm`, `380 g`, `32.0 mm` outlet pitch) rests
inside a `4.5 mm` alignment frame (`strip_cleat_frame.stl`) at `(24.5, -2.0) cm` with two connected `1.8 m` (`6 ft`)
appliance cords in Outlets `1` (blue) and `2` (grey) that must remain connected throughout, plus a white `2-prong` Class
II `AC` wall-wart adapter (**TP-Link T120150-2B1** `12V 1.5A`, `34 x 62 x 38 mm`, `105 g`) in Outlet `3` whose `48 mm`
`-y` body overhang completely covers end Outlet `4`. On the right at `(24.5, -18.5) cm`, a loose black `1.8 m` `18 AWG
SVT` appliance cord's `3-prong` NEMA `5-15P` plug (`34 x 24 x 38 mm`, `64 g`) rests prongs-down on `plug_cord_cradle.stl`
with all five cords routed forward (`+x`) through desk clips off the front desk edge.

The left arm steadies the free-standing strip while the right arm temporarily unplugs the white adapter from Outlet `3`,
reseats it flush (`<= 1.5 mm` face gap, zero exposed blade metal) into end Outlet `4` (where its overhang extends `22 mm`
past the `-86 mm` strip end, uncovering Outlet `3`), and seats the black cord's `3-prong` plug flush (`<= 1.5 mm`, zero
exposed blades) into Outlet `3`.

Files:
- `rearrange_wall_wart_to_end_of_crowded_power_strip.py` (scene, longitudinal `4 x 3-cavity` NEMA `5-15R` strip, `3-prong` NEMA `5-15P` & `2-prong` NEMA `1-15P` plugs, routed `+x` cord guides, `<= 1.5 mm` zero-exposed-blade live scorer, closed-loop 6-DoF bimanual demo)
- `fabricate.py` (`strip_cleat_frame.stl`, `plug_cord_cradle.stl`, and `1.5 mm` no-go `feeler_gauge_1p5mm.stl`)
- `test_scoring.py` (offline longitudinal `3-prong` socket clearance vs `180 deg` ground-pin rejection, wall-wart overhang block in Outlet `3` vs clearance in Outlet `4`, `<= 1.5 mm` exposed-blade rejection, 5-tier scorer, failure latches, STL generation, and 6-DoF IK dry-run checks)
- `mechkit.py`, `closedloop.py`, `belimits.py`, `gripkit.py` (procedural convex-hull geometry, rubber pad friction, and 6-DoF arm control utilities)

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rearrange-wall-wart-to-end-of-crowded-power-strip/rearrange_wall_wart_to_end_of_crowded_power_strip.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Plug the black cord on the right into the power strip so all four cords end up plugged in; keep the blue and grey plugs connected throughout, though you may unplug and move the white adapter." \
  --scorer task_score --max-steps 600 --seed 0 --store-frames --log-dir logs/crowded-power-strip-demo
```

- Model run (Dream-1): the same command with `--policy dream`.
- Printable trial fixtures & `1.5 mm` no-go feeler gauge: `python3 tasks/rearrange-wall-wart-to-end-of-crowded-power-strip/fabricate.py fixtures/`.
- Offline test suite: `python3 tasks/rearrange-wall-wart-to-end-of-crowded-power-strip/test_scoring.py`.
- Scorer checks (`DEMO_VARIANT`):
  - `DEMO_VARIANT=unplug_existing` lifts the connected blue appliance plug out of Outlet `1` (`0.00`, `existing_unplugged` latch).
  - `DEMO_VARIANT=unplug_wart_only` braces the strip and pulls the white adapter clear of Outlet `3` onto the desk without reseating it (`0.25`).
  - `DEMO_VARIANT=move_wart_only` moves the white adapter flush into end Outlet `4` without plugging in the black cord (`0.50`).

## Results

- Scripted demo (`yam_scripted`): `1.0` (`seed=0` in `469` steps, `seed=1`: `1.0`; all four plugs seated dead flush at `0.0 mm` gap, passing the `<= 1.5 mm` zero-exposed-blade check).
- Model run (`dream`, `seed=0`): `0.0` (`600` steps — reaches right arm toward the black plug on the right cradle and hovers beside it without moving the white adapter in Outlet `3`).
- Scorer checks: `unplug_existing` `0.00`, `unplug_wart_only` `0.25`, `move_wart_only` `0.50`.

## Simulation, NEMA 5-15 geometry, cords, and retail parts

| Part | Value | Source, routing, or setup check |
|---|---|---|
| `4-outlet` power strip | **CyberPower CSB404** (`214 x 50 x 35 mm`, `380 +/- 10 g`, `32.0 mm` longitudinal pitch) | `$7.99` (`ASIN B004K1YG22`); `1.2 m` `14 AWG SJT` cord routed `+x` via `(35.0, +11.4)` clip to unplugged floor drop |
| White `AC` wall-wart | **TP-Link T120150-2B1** `12V 1.5A` (`34 x 62 x 38 mm`, `105 +/- 5 g`, `2-prong` NEMA `1-15P`, `48 mm` `-y` overhang) | `$9.99`; `1.5 m` `22 AWG` DC cable in `22 cm` `+x` slack loop via `(36.0, -8.6)` clip |
| Connected cords (`Outlets 1 & 2`) | **Monoprice 107673** (blue) & **StarTech PXT101** (grey) `1.8 m` `18 AWG` NEMA `5-15P` to `IEC C13` (`48 g` plugs) | Seated `<= 1.5 mm`; routed `+x` via `(36.0, 0.0)` dual clip (`15 cm` slack, `C13` ends taped to front leg) |
| Staged black cord | **Monoprice 105278** `1.8 m` `18 AWG SVT` NEMA `5-15P` to `IEC C13` (`34 x 24 x 38 mm`, `64 g` plug) | Staged prongs-down on `plug_cord_cradle.stl` at `(24.5, -18.5)`; `35 cm` `+x` slack via `(38.0, -18.0)` clip |
| Printable fixtures & gauge | `strip_cleat_frame.stl`, `plug_cord_cradle.stl`, `feeler_gauge_1p5mm.stl` (`1.5 mm` no-go blade) | Enforces identical `<= 1.5 mm` zero-exposed-blade check in real trials and simulation |

## Runtime and seeds

`object_jitter=0` fixed layout; run limit `600` control steps (`30.0 s` at `20 Hz`, ~75 s wall-clock per rollout on a
single RTX 3090 GPU). Verified with `--seed 0` and `--seed 1`.

## Sources and licenses

- `rearrange_wall_wart_to_end_of_crowded_power_strip.py`, `fabricate.py`, and `test_scoring.py` were authored by this task's team.
- `mechkit.py`, `closedloop.py`, `belimits.py`, and `gripkit.py` are adapted from MIT-licensed utilities in `robocurve/benchmark-tasks` (`tasks/remove-windscreen-before-lighting-canister-stove` and `tasks/fit-humidifier-onto-oxygen-flowmeter`).
- Robot link hulls load at runtime from `yam_arena` and are not redistributed; no third-party binary meshes, images, or videos are committed to git.
