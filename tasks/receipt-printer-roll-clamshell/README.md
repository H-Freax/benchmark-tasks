# Load a new paper roll into a clamshell receipt printer

A drop-in receipt printer stands on the bench with its lid latched over an empty core. The new 80 mm roll lies on
the bench in front of it. The instruction gives only the goal. To load the roll, press the release button so the
lid's spring lifts it, then take the old core out. Drop the roll into the cradle so the paper comes off the bottom
of the roll toward the tear bar, and bring the leader over the platen and out past the slot. Then press the lid
shut until the latch pawl snaps, which takes about 10 N. Closing the lid over a reversed roll, or with the leader
left inside (less than 15 mm out of the slot), scores 0.

Files: `task.json`, `receipt_printer_roll_clamshell.py` (the scene, `success()`, the partial-credit `score()`, the
scripted `demo()` with its degenerate variants, and an offline self-test) and `mechkit.py` (joints, sliders and
catches added to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/receipt-printer-roll-clamshell/receipt_printer_roll_clamshell.py
```

This asserts the scoring rules on hand-built states. It also replays every demo variant through the kinematics at
20 Hz and checks the joint limits and an IK error under 0.5 mm.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/receipt-printer-roll-clamshell/receipt_printer_roll_clamshell.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The receipt printer is out of paper. Load the new roll." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/receipt-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Use the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/receipt-demo`. The demo is open loop
and deterministic. The right arm presses the release button and the lid springs open. The left arm lifts the old
core out and lays it on the bench. The right arm lifts the new roll with its leader hanging and carries it in over
the tear bar, so the leader drags over the platen and hangs out the front. It then lowers the roll into the
cradle. The left arm sweeps the lid over from behind and presses it home until it latches.

Degenerate checks use the same command with a variable set in the environment. Each one scores 0:

- `DEMO_VARIANT=close_only` opens the lid and presses it shut again over the old core. Nothing has changed, and
  the lid points need a seated new roll.
- `DEMO_VARIANT=reversed` turns the roll end for end (143 deg, set down, regrasp, 37 deg more) before loading it,
  so the paper would leave the bottom away from the slot. Closing the lid on it latches the jam.
- `DEMO_VARIANT=tucked` loads the roll as the demo does, lifts it again, draws it 18 mm rearward to pull the
  leader in, then sets it back low into the cradle. Closing the lid with less than 15 mm out latches the jam.

`RECEIPT_DEBUG=1` prints the state every 20 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/receipt-demo/*.json
ffmpeg -y -i ~/yam-eval/receipt-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 s at 20 Hz). The demo plan is about 47 s long.
- There is one fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is
  deterministic. Dream-1 is hosted and not seeded.
- A trial takes roughly 6 to 8 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Press the release button, take the roll out and lay it back at its mark with the leader flat toward the right arm,
put the old core back in the cradle and press the lid shut.
