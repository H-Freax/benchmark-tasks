# Float a sample-loaded microtube rack in a benchtop ultrasonic bath, close its lid and press ON

`seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py` for the `yam_arena`
embodiment of [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A Magnasonic
CD2800 600 mL benchtop ultrasonic cleaner sits on a charcoal silicone mat in front of the robot, filled with 480 mL of
tap water to its `MAX` line with its rear-hinged smoked-blue domed cover propped open against its rear stop. To the
robot's left on an olive-slate mat, a Heathrow Scientific HS29041B 8-well EVA foam floating rack loaded with eight
capped 1.5 mL Eppendorf tubes (1.00 mL water per tube; 22.0 g total loaded mass) rests on an open-well 3D-printed
staging cradle so its 19 mm conical tube tips hang clear of the bench. The robot pinches the dry upper foam margin,
floats the rack level in the water bath without wetting either gripper, closes the domed cover flush on the tank rim,
presses `ON` to start sonication, and lets go.

Files:
- `seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py` (procedural 3D meshes, Archimedes hydrostatics, live scorer, closed-loop 6-DoF demo with `state_reads = 0`)
- `fabricate.py` (writes the three printable PETG bench fixtures: `staging_cradle.stl`, `button_actuator_cap.stl`, `freeboard_gauge.stl`)
- `test_scoring.py` (offline hull clearance, Archimedes freeboard, live scoring ladder, failure latches, STL generation, and 6-DoF IK dry-run checks)
- `mechkit.py`, `closedloop.py`, `belimits.py`, `gripkit.py` (spawning, hull geometry, rubber pad friction, and 6-DoF arm control utilities)

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas/seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Float the sample-loaded microtube rack level in the ultrasonic water bath, close the hinged cover flush, press ON to start sonication, and let go without wetting the grippers." \
  --scorer task_score --max-steps 600 --seed 0 --store-frames --log-dir logs/ultrasonic-demo
```

- Model run (Dream-1): the same command with `--policy dream`.
- Offline test suite: `python3 tasks/seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas/test_scoring.py`.
- Scorer checks (`DEMO_VARIANT`):
  - `DEMO_VARIANT=rack_only` floats the rack level and retracts while leaving the cover open (`0.25`).
  - `DEMO_VARIANT=no_sonicate` floats the rack and closes the cover flush without pressing `ON` (`0.50`).
  - `DEMO_VARIANT=no_release` floats the rack, closes the cover, and presses `ON` while holding the finger on the switch (`0.75`).
  - `DEMO_VARIANT=tilt_flood` tips the rack past 18 degrees during transfer, latching `rack_flooded` (`0.00`).

## Results

- Scripted demo (`yam_scripted`, seeds `0` and `1`): `1.0` (`479` steps; rack floating level inside the `MAX`-filled bath, smoked cover closed flush on the rim, red `ON` LED latched active, both arms returned to `READY` `>= 3.0 cm` clear with `1.0 s` stillness, zero state reads).
- Model run (`dream`, seed `0`, `600` steps): `0.0` (the left arm reaches over the staging cradle and nudges the near-left corner of the blue foam rack without grasping or lifting it out of the cradle, while the right arm stays near home and the hinged bath cover remains propped open).
- Scorer checks (`DEMO_VARIANT`): `rack_only` `0.25` (`314` steps), `no_sonicate` `0.50` (`426` steps), `no_release` `0.75` (`425` steps), `tilt_flood` `0.00` (`193` steps).

## Why it matters (`Other`: wet-lab sample preparation)

Biological, chemical, and clinical laboratory technicians (over 559,000 US workers across BLS Occupational Employment
and Wage Statistics SOC `19-4021` Biological Technicians, `19-4031` Chemical Technicians, `19-1021` Biochemists and
Biophysicists, and `29-2010` Clinical Laboratory Technologists and Technicians) routinely sonicate capped 1.5 mL
microcentrifuge tubes in compact benchtop ultrasonic water baths to disperse cell pellets, degas buffers, and assist
lysis. Conical tube bottoms must hang immersed in the water bath while snap caps and shared robot grippers stay dry,
and the hinged acoustic cover must be closed before starting ultrasonic cavitation to contain aerosolized mist.

## Simulation, Archimedes hydrostatics, and physical sizes

- **Archimedes equilibrium (`22.0 g` total mass).** Eight 19.0 mm conical polypropylene tube tips (`r = 2.0..5.3 mm`)
  submerged below the foam underside displace `7.30 mL` of water (`7.30 g`). The remaining `14.70 g` across the
  `38.1 cm^2` net EVA foam underside (`5.4 x 8.2 cm` minus eight `11.0 mm` bored wells) produces a **`3.86 mm` foam
  draft**. With the `480 mL` `MAX` water surface at `+70.0 mm` above the bench mat (`38.0 mm` depth above the `+32.0 mm`
  tank floor, `14.0 mm` below the `+84.0 mm` tank rim), the resting foam centre sits at **`+76.1 mm` above the mat**,
  leaving **`16.1 mm` of dry foam freeboard** above water and **`15.1 mm` of clearance** under the tube tips above the
  stainless tank floor.
- **Rigid-body water approximation.** Bath water is rendered as a non-colliding blue plane at `+70.0 mm` above the mat,
  while vertical buoyancy at equilibrium is provided by an invisible low-friction hydrostatic support plane at
  `+47.1 mm` under the tube tips. All lateral tank walls, bored foam wells, turned conical tubes, the over-centre
  gravity-hinged domed cover (`hold = 0.0`, resting against its `102 deg` open stop or `0 deg` rim stop under gravity),
  and the `900 N/m` spring-return `ON` / `OFF` tactile buttons resolve through PhysX convex-hull contact.
- **Solid-state switch electronics.** Pressing `ON` past `2.5 mm` (`2.25 N`) latches `cycle_on = True` and illuminates
  the red `cycle_led`; pressing `OFF` past `2.5 mm` clears both. Operating a water-filled bath without samples is **not**
  treated as a dry-run fault (it simply scores `0.0` until the rack is floating inside).

| Part | Value | Source, or setup check |
|---|---|---|
| Ultrasonic cleaner | Magnasonic CD2800 (`$44.99`, ASIN `B007Q2661E`), 600 mL stainless tank `156 x 96 x 52 mm`, exterior `200 x 140 x 84 mm` (to tank rim) | Magnasonic CD2800 product specification & user manual |
| Water fill (`MAX` line) | `480 mL` tap water (`38.0 mm` depth; surface `+70.0 mm` above mat, `14.0 mm` below `+84.0 mm` rim) | CD2800 `MAX` fill specification; verified each trial with `freeboard_gauge.stl` |
| Hinged cover | Rear-hinged smoked-blue domed polycarbonate lid (`16 mm` interior crown), `102 deg` open rest to `0 deg` rim rest | Built-in CD2800 flip cover; check `<= 6 deg` (`<= 11 mm` front lip gap) when closed |
| Front `ON` / `OFF` pads | `14 x 22 x 6 mm` PETG actuator caps (`button_actuator_cap.stl`) on front oval switches (`4.0 mm` travel, `2.5 mm` click) | `python3 fabricate.py fixtures/`; check click and red `ON` LED |
| Floating foam rack | Heathrow Scientific HS29041B (`$14.50`), `54 x 82 x 20 mm` closed-cell EVA foam, `2 x 4` `11.0 mm` wells, `6.0 g` | Heathrow Scientific HS29041B catalog dimensions; check dry mass `6.0 g` |
| Sample tubes (x8) | Eppendorf `0030120086` Safe-Lock `1.5 mL` PP tubes filled with `1.00 mL` tap water (`2.0 g` each; `22.0 g` loaded rack) | Check on `0.1 g` scale: `22.0 +/- 0.5 g` loaded rack mass before each trial |
| Staging cradle | `74 x 102 x 26 mm` PETG stand (`staging_cradle.stl`), `+23.0 mm` ledge, `45 x 75 x 20 mm` inner tube-tip clearance well | `python3 fabricate.py fixtures/`; check `4.0 mm` clearance under `19 mm` tube tips |

## The demo

The demo (`demo_script`) uses fixed waypoints for the `object_jitter=0` benchtop layout and reads **zero** privileged
object or simulator states (`state_reads = 0` verified in `test_scoring.py`). The left arm aligns its parallel jaws
across the 54 mm rack width (`yaw = -pi/2`), pinches only the top 8 mm dry foam band, lifts straight out of the cradle,
lowers the rack level onto the bath water (`pad tips >= 8.1 mm` above water), opens, and returns to `READY`. The right
arm closes its fingers, sweeps the open cover forward over-centre onto the tank rim, presses the front teal `ON` switch
down `3.8 mm` to latch sonication on, releases, and returns to `READY` for a `1.6 s` stillness dwell.

## Real-world build and protecting the grippers

- **Fixtures.** Print `staging_cradle.stl`, two `button_actuator_cap.stl`, and `freeboard_gauge.stl` in PETG (`python3 fabricate.py fixtures/`, ~42 g filament total).
- **Protecting the shared grippers.** Slip disposable powder-free nitrile finger cots over all four parallel jaw pads with a `4 mm` white depth-reference band `4 mm` above each pad tip. Because the rack floats with `16.1 mm` of dry foam above the waterline (`14.0 mm` below the tank rim, i.e., `2.1 mm` proud of the rim), the robot grasps and releases the upper foam margin without entering the water. Video crossing of the `+4 mm` band to or below the `MAX` waterline, or any wet spot when blotting the four cotted pads on dry lint-free tissue, scores `0`.
- **Between trials (< 45 s).** Press `OFF` if running, prop the cover open against its `102 deg` stop, blot the rack underside and tube exteriors dry, verify `22.0 +/- 0.5 g` on a `0.1 g` scale, set the rack on `staging_cradle.stl`, check the `14.0 mm` below-rim water level with `freeboard_gauge.stl`, and confirm all four nitrile cots are dry.

## Runtime and seeds

`object_jitter=0` fixed layout; run limit `600` control steps (`30.0 s` at `20 Hz`, ~85 s wall-clock per rollout on a single RTX 3090 GPU). Verified with `--seed 0` and `--seed 1`.

## Sources and licenses

- `seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py`, `fabricate.py`, and `test_scoring.py` were authored by this task's team.
- `mechkit.py`, `closedloop.py`, `belimits.py`, and `gripkit.py` are adapted from MIT-licensed utilities in `robocurve/benchmark-tasks` (`tasks/remove-windscreen-before-lighting-canister-stove` and `tasks/fit-humidifier-onto-oxygen-flowmeter`).
- Robot link hulls load at runtime from `yam_arena` and are not redistributed; no third-party binary meshes, images, or videos are committed to git.
