# Dose a front-loader's dispenser drawer: both tablets in the main-wash compartment, drawer shut

Instruction given to the policy: `Dose this wash with the two detergent tablets; it is a normal cycle with no pre-wash.`

Files: `task.json`, `detergent_drawer_main_wash.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (adds the drawer slide to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/detergent-drawer-main-wash/detergent_drawer_main_wash.py
```

It asserts the scoring rules on hand-built states and replays both arms' demo plans through the kinematics at
20 Hz (every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 51.3 s plan, worst IK 0.091 mm / 0.013 deg
right: 2.0 s plan, worst IK 0.060 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/detergent-drawer-main-wash/detergent_drawer_main_wash.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Dose this wash with the two detergent tablets; it is a normal cycle with no pre-wash." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/detergent-drawer-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step. The CLI prints
`task_score: <value>`.

## Run and score the full-score demo

Same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/detergent-drawer-demo`. The demo is `demo()` in
the task file: open loop and deterministic. The left arm grips the pull tab and draws the drawer to its stop, sets
each tablet on the main-wash floor, then grips the tab again and pushes the drawer shut (51 s of plan, about 1,050
steps with the success hold, inside the 1,400-step limit). It scores 1.0. Two scripted variants were checked: both
tablets put in the pre-wash compartment and the drawer shut scores 0; both in main wash with the drawer left out
scores 0.6.

## Videos and screenshot

```bash
for log in ~/yam-eval/detergent-drawer-*/*.json; do inspect-robots video "$log"; done
```

The screenshot is the first top-camera frame of a 2-step run at `-E cam_width=1280 -E cam_height=704`.

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: a few minutes of wall clock, plus Isaac Sim warm-up on the first run.

## Reset between trials

Push the drawer fully shut and set both tablets flat on their marks; replace a tablet that has chipped or got damp.
