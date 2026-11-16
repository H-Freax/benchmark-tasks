# Move a mortar and its loose pestle

Move the granite mortar to the green work area and leave it upright with the pestle resting inside. The long loose pestle initially rests between the bowl floor and rim. Temporary tabletop placement, separate support, and other safe ways to move the pair are allowed.

The mortar is 140 mm across and 80 mm high, with a 110 mm cavity, 18 mm floor, two integral 30 × 36 × 18 mm ears, and an estimated mass of 1.85 kg. Its independent 180 mm pestle has a 48 mm-diameter, 50 mm-long head and a 28 mm-diameter, 130 mm-long shaft, mass 0.48 kg. Both are granite engineering equivalents. The cavity has 32 convex wall sectors. There are no grasp welds, pose writers or active mechanisms. Fracture and grinding are outside this operation; surface friction uses the stock arena material (static 1.2, dynamic 1.0) and is not granite-calibrated.

Coordinates are metres from the arm-base midpoint: +x forward, +y left, z up. Bases are (0, ±0.305, 0.750); the wood table is 1.10 × 1.30 m, centred at x=0.36, top z=0.7475. The mortar bottom starts at (0.240, 0, 0.7475), target (0.300, −0.160, 0.7475). The pestle leans 52.95° toward +x from vertical; its exact initial origin and xyzw quaternion appear in `task.json` and the task source. Four green border strips, each 1 mm high, mark a 210 × 240 mm area outside the target mortar footprint.

`score()` evaluates the end state. Full placement requires a mortar bottom within 25 mm horizontally and 4 mm vertically of target, with tilt ≤5°. Containment checks the actual pestle-head vertices against the cavity and floor with 3 mm contact tolerance; the head centre must be at least 10 mm below the rim and 3 mm inside the cavity wall. Both objects must move slower than 10 mm/s and 0.17 rad/s. Release checks all 630 robot collision pieces with a conservative 20 mm margin; missing or invalid numeric readouts fail closed.

| Outcome | Score |
|---|---:|
| Neither placement nor the approach condition below | 0 |
| Mortar within 90 mm of target, pestle contained, both stable | 0.25 |
| Target mortar without contained pestle, held or moving | 0.25 |
| Target mortar without contained pestle, stable and released | 0.50 |
| Target mortar with contained pestle, held or moving | 0.75 |
| Target mortar and contained pestle, stable and released | 1.0 |

The exact branch order is in `task.json`: the 90 mm approach tier applies only before full mortar placement. Either body's lowest point ever more than 50 mm below the tabletop latches score 0; a current point more than 25 mm below it also scores 0. Full success must hold for one second.

Run from the repository root with IsaacLab, `yam_arena` and Inspect Robots installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/move-mortar-and-pestle/move_mortar_and_pestle.py \
  -E object_jitter=0 -E control_hz=20 --scorer task_score \
  --max-steps 1350 --seed 0 --no-prompt --store-frames \
  --instruction "Move the granite mortar to the green work area and leave it upright with the pestle resting inside." \
  --log-dir runs/mortar-demo
```

Use `--policy dream` for the model run and the instruction from `task.json`. The limit is 1,350 steps, 67.5 simulated seconds at 20 Hz; the default scripted reference lasts 63 seconds. Wall runtime depends on the machine. One fixed starting layout is used, seed 0.

Two controls change only ordinary demo actions, while retaining the same scene and scorer:

```sh
DEMO_VARIANT=mortar_only inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/move-mortar-and-pestle/move_mortar_and_pestle.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 1350 --seed 0 --no-prompt --log-dir runs/mortar-only
DEMO_VARIANT=keep_held inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/move-mortar-and-pestle/move_mortar_and_pestle.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 1350 --seed 0 --no-prompt --log-dir runs/mortar-held
```

`mortar_only` leaves the pestle staged on the table after relocating and releasing the mortar; expected score 0.50. `keep_held` returns the pestle into the bowl but keeps the real gripper closed; expected score at most 0.75. Empty `DEMO_VARIANT` selects the full reference. These controls use no object-state feedback.

Reset the mortar and pestle to the exact starting transforms and both arms to READY with open grippers. Nothing is consumed. Recorded demo scores, control scores, Dream result and uploaded media are listed in `task.json`.


Validation on the three Python source hashes recorded in `validation.json`:

| Trial | Controls | Score |
|---|---:|---:|
| Full reference, first trial | 1,203 | 1.0 |
| Full reference, repeat | 1,203 | 1.0 |
| Mortar-only control | 941 | 0.50 |
| Pestle still held control | 1,301 | 0.75 |
| Dream-1 | 1,350 | 0.0 |

All five trials ended normally with no errored trials and unchanged source. Full references took about 160 seconds wall time each on the recorded machine; the Dream trial took about 191 seconds. Recorded top-camera observations are 640 × 352 at 20 Hz. The six scoring fixtures, actual 630-piece release readout fixture, and ten invalid-input/finite-clearance checks also passed. The metadata results and media ID are filled after the trials; the scene, scorer and ordinary reference code are identical to the validated hashes.
