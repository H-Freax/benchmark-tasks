# Take a wireless mouse's USB receiver out of its slot and plug it into a laptop

Instruction given to the policy: `Plug the mouse's USB receiver into the laptop.`

Files: `task.json`, `office_mouse_receiver_transfer.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (multi-box parts and the breakable detent, added to the stock runtime
in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/office-mouse-receiver-transfer/office_mouse_receiver_transfer.py
```

It asserts the scoring rules on hand-built states and replays both arms' demo plans through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 10 cases pass
left: 18.0 s plan, worst IK 0.073 mm / 0.013 deg
right: 23.2 s plan, worst IK 0.093 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/office-mouse-receiver-transfer/office_mouse_receiver_transfer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the mouse's USB receiver into the laptop." \
  --scorer task_score --max-steps 1000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/mouse-receiver-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/office-mouse-receiver-transfer/office_mouse_receiver_transfer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the mouse's USB receiver into the laptop." \
  --scorer task_score --max-steps 1000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/mouse-receiver-demo
```

The demo is `demo()` in the task file: open loop and deterministic. The left arm presses the mouse down while the right
arm grips the receiver's body, pulls it out, carries it to the laptop and pushes the plug into the port. It reaches
success after about 22 s (about 440 steps), inside the run limit of 1,000 steps. Without the left arm's hold the pull
drags the mouse along and the trial scores 0; pulling the receiver out and leaving it on the desk scores 0.3.

## Videos and screenshot

```bash
for log in ~/yam-eval/mouse-receiver-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/mouse-receiver-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,000 steps (50 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Push the receiver back into the mouse's slot (the detent re-engages) and put the mouse on its mark; the laptop does
not move.
