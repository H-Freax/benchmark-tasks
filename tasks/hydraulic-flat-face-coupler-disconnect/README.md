# Relieve, disconnect and cap a hydraulic flat-face quick coupler

Instruction given to the policy: `Disconnect the hydraulic coupler and cap both ends.`

The instruction does not mention the trapped pressure or how the coupler lets go. The scored steps it leaves
unstated: turn the relief lever into its relieving detent until the gauge reads zero (until then the male is held in
by the pressure), then hold the female's sleeve pushed back with one hand while the other draws the male out, then
cap both open ends: the dust plug from the tray into the female, and the male's nose into the dust cup in its holder.

Files: `task.json`, `hydraulic_flat_face_coupler_disconnect.py` (scene, `success()`, partial-credit `score()`, the
scripted `demo()` and an offline self-test), `mechkit.py` (joints, sliders, releasable bonds and snap welds added to
the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/hydraulic-flat-face-coupler-disconnect/hydraulic_flat_face_coupler_disconnect.py
```

It asserts the scoring rules on hand-built states and replays both arms' demo plans through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
left: 46.7 s plan, worst IK 0.098 mm / 0.013 deg
right: 41.1 s plan, worst IK 0.096 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hydraulic-flat-face-coupler-disconnect/hydraulic_flat_face_coupler_disconnect.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Disconnect the hydraulic coupler and cap both ends." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hydraulic-coupler-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/hydraulic-flat-face-coupler-disconnect/hydraulic_flat_face_coupler_disconnect.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Disconnect the hydraulic coupler and cap both ends." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hydraulic-coupler-demo
```

The demo is `demo()` in the task file: open loop and deterministic, about 900 steps to success, inside the run
limit of 1,600. The left arm sweeps the relief lever into its relieving detent and waits over the coupler; the right
arm grips the steel line over the male's centre of mass. The left arm then holds the sleeve down on its stop (500 N/m spring, 5 N there) with
its fingers either side of the male's body while the right arm draws the male out; the left lets the sleeve up, opens
and backs out from round the male before the right arm carries it to the cup holder, pushes the nose into the dust cup
and leaves it standing upright with its line on the rest. The left arm takes the dust plug from the tray and pushes it
into the female's mouth.

Drawing the nose out of the bore turns the male about 2.2 deg in the two-pad pinch on the line (`SLIP_DEG`, measured
in sim), so the carry turns the wrist back by that much; the remaining aim offset (`CUP_AIM`, 4 mm / 1.5 mm) is the
right arm's tracking lag coming down with the male held 15 cm off the wrist.

Degenerate checks (same command, variant in the environment), both score 0:

- `DEMO_VARIANT=no_relief`: the same moves without relieving the line. The sleeve goes back, but the male stays held,
  so nothing comes out and neither end can be capped.
- `DEMO_VARIANT=relieve_only`: relieves the line and stops. Relief only counts with the male out and parked upright
  on its holder and rest, so it earns nothing on its own.

## Videos and screenshot

```bash
for log in ~/yam-eval/hydraulic-coupler-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/hydraulic-coupler-demo/frames/<run>/~f1~scene-0-e0~top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,600 steps (80 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Turn the relief lever back to closed (the gauge back to 150 bar: the trapped pressure is a number in the sim), pull
both caps off, push the male back into the female with the sleeve held back and its line on the stand, stand the plug
head up in the tray and the cup open side up in its holder.
