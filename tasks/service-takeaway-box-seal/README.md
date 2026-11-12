# Match and seal the correct takeaway box

Instruction given to the policy: `Seal the order with the tamper label.`

Files: `task.json`, `service_takeaway_box_seal.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (multi-box rigid parts and per-step hooks for the stock runtime),
`README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/service-takeaway-box-seal/service_takeaway_box_seal.py
```

It asserts the scoring rules on hand-built states (including the wrong-box latch) and replays both arms' demo
plans through the kinematics at 20 Hz (joints inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 10 cases pass
left: 13.3 s plan, ...
right: 29.5 s plan, ...
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/service-takeaway-box-seal/service_takeaway_box_seal.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seal the order with the tamper label." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/service-takeaway-box-seal-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step. The CLI prints
`task_score: <value>`.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (and `--log-dir ~/yam-eval/service-takeaway-box-seal-demo`). The demo is
`demo()` in the task file: open loop and deterministic. The left arm lifts the napkin off the seal and lays it on
the counter; the right arm pinches the seal's pull tab, peels it off the backing and lays it on the B24 box's panel.
About 30 s of plan (about 620 steps with the success hold), inside the run limit.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. The demo is deterministic.
- Runtime per trial on an L40S: about 1 minute per 400 steps, plus Isaac Sim warm-up on the first run.

## Reset between trials

Put a fresh seal on its backing at the mark, tab toward the robot, and lay the folded napkin over its printed
code; the boxes, backing and mat do not move.
