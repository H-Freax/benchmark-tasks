# Restock a twin jumbo-roll tissue dispenser

Instruction given to the policy: `This dispenser is almost empty. Restock it with the new roll so the old one gets used up first, and lock it.`

The instruction leaves the order and the handling unstated. The stub has to come off the primary spindle and go
onto the reserve spindle, which a slide gate blocks until it is turned clear. The new roll then goes onto the
primary. Each spindle's sprung split hub lets a core on or off only while the release tab beside it is held down,
so one hand holds the tab while the other slides the core along the spindle. Last, the cover's stay is released
with its button; the closing spring and damper lower the cover until the latch clicks. Closing it over a core that
is not seated scores 0.

Files: `task.json`, `jumbo_tissue_dispenser_stub_to_reserve.py` (the scene, `success()`, the partial-credit
`score()`, the scripted `demo()` with its degenerate variants, and an offline self-test) and `mechkit.py`
(joints, sliders, bonds and catches added to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py
```

This asserts the scoring rules on hand-built states. It also replays every demo variant through the kinematics at
20 Hz and checks that each joint stays inside its URDF limits and the IK error stays under 0.5 mm. It prints
`scoring: 16 cases pass` and then one line per arm and variant.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This dispenser is almost empty. Restock it with the new roll so the old one gets used up first, and lock it." \
  --scorer task_score --max-steps 2000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/jumbo-dispenser-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/jumbo-dispenser-demo`. The demo is
open loop and deterministic. The right arm turns the gate clear and holds the primary tab while the left arm opens
its fingers inside the stub's core and slides the stub up off the spindle. The left arm then stands the stub on
the back plate between the spindles. The right arm takes it from there onto the reserve spindle while the left
holds the reserve tab. Next the left arm carries the new roll from the bench onto the primary spindle while the
right holds the primary tab. Finally the right arm presses the stay button and the cover closes and latches on
its own in about 6 s.

Degenerate checks use the same command with a variable set in the environment. Each one scores 0:

- `DEMO_VARIANT=close_only` releases the cover over the untouched dispenser. The cover points need both rolls
  seated.
- `DEMO_VARIANT=trash_stub` drops the stub into the bin and loads the new roll on the primary. This trips the bin latch.
- `DEMO_VARIANT=loose_roll` moves the stub correctly, then sets the new roll on the primary without holding its
  tab, so the roll rests on the hub unseated. Closing the cover onto it trips the forced-cover latch.

`JUMBO_DEBUG=1` prints the state every 20 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/jumbo-dispenser-demo/*.json
ffmpeg -y -i ~/yam-eval/jumbo-dispenser-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,000 steps (100 seconds at 20 Hz). The demo plan is about 70 s long.
- There is one fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is
  deterministic. Dream-1 is hosted and not seeded.
- A trial takes roughly 8 to 10 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Press the latch, open the cover to 100 deg and set its stay. Hold the reserve tab and take the stub off, then hold the
primary tab and take the new roll off. Put the stub back on the primary and lay the new roll at its mark on the
bench. Turn the gate back over the reserve hub.
