# Dry-fit and peg a mortise-and-tenon frame corner

Instruction given to the policy: `Assemble the frame corner and peg it.`

Files: `task.json`, `frame_corner_pegged_tenon.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (multi-box parts for the hollow mortise and holes), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Offline check, no GPU

```bash
python tasks/frame-corner-pegged-tenon/frame_corner_pegged_tenon.py
```

It checks the clearances, asserts the scoring rules on hand-built states and replays both arms' demo plans through
the kinematics at 20 Hz (every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 37.0 s plan, worst IK 0.085 mm / 0.013 deg
right: 19.7 s plan, worst IK 0.093 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/frame-corner-pegged-tenon/frame_corner_pegged_tenon.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the frame corner and peg it." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir logs/frame-corner-pegged-tenon-dream
```

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/frame-corner-pegged-tenon/frame_corner_pegged_tenon.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the frame corner and peg it." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir logs/frame-corner-pegged-tenon-demo
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step. The demo is open
loop and deterministic: the right arm lines the rail up on the mortise and pushes the tenon home (about 20 s), then
the left arm takes the peg from its block, drops it through the stile and the tenon and taps it down (about 37 s in
all, about 750 steps; run limit 1,200). It scored 1.0 with seed 0.

Shortcuts checked in sim (both score 0): pegging first and then pushing the rail (the peg stops the tenon 18 mm in);
pushing the rail without lining it up on the mortise (the tenon's end meets the stile's edge and the peg drops
into the empty mortise).

Runtime per trial: about 2 minutes on an L40S including startup. Seed 0, `object_jitter=0`.
