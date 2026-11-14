# Tip in a tilt-sash window to clean it, then shut and lock it

Instruction given to the policy: `Tip this window in so its outside can be wiped, then shut it again.`

The instruction leaves the mechanism unstated. The lower sash of a double-hung tilt window only tips in while both
tilt latches in its top rail are held drawn in, so the two hands work at once: each pinches one latch tab, draws it
in against its spring and keeps it there while both pull the top of the sash in. Let one go before the bolts
have cleared the front stops (about 10 deg) and that bolt springs back out under its stop and holds the sash. Past
30 deg is the cleaning position. The sash then has to go back into the frame with both bolts snapping into their
tracks, be lowered 3 cm to the sill and the thumb-turn lock on its top rail turned shut. Leaving it tipped in,
shutting it without locking it, or letting it drop back against the stops scores 0.

The window unit lies on its back on the bench, room side up, sill end away from the robot: the arms cannot reach
the top rail of a sash standing upright. "Up" for the sash is toward the robot and "tipping in" lifts its top rail.

Files: `task.json`, `window_tilt_in_sash_clean_and_relock.py` (the scene, `success()`, the partial-credit
`score()`, the scripted `demo()` with its degenerate variants, and an offline self-test) and `mechkit.py`
(joints and sliders added to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/window-tilt-in-sash-clean-and-relock/window_tilt_in_sash_clean_and_relock.py
```

This asserts the scoring rules on hand-built states and checks the latch geometry (bolt behind its stop when out,
clear of it when drawn in 6 mm; the bolts clear the stops at about 10 deg). It also replays every demo variant
through the kinematics at 20 Hz and checks that each joint stays inside its URDF limits and the IK error stays under
0.5 mm. It prints `scoring: 12 cases pass` and then one line per arm and variant.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/window-tilt-in-sash-clean-and-relock/window_tilt_in_sash_clean_and_relock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tip this window in so its outside can be wiped, then shut it again." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/tilt-window-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/tilt-window-demo`. The demo is open
loop and deterministic. Each arm pinches one latch tab and draws it in (about 7 mm); together they tip the sash in
to about 35 deg along its arc, hold it there, and bring it back into the frame. The sash slides up its tracks while
it is tipped and back down to the sill as it comes back. They let the latches out, push the sash down onto the sill
and let go. The left arm then turns the lock from +45 to -45 deg.

Degenerate checks use the same command with a variable set in the environment. Each one scores 0:

- `DEMO_VARIANT=leave_tilted` tips the sash in to about 35 deg and keeps it there.
- `DEMO_VARIANT=no_lock` does everything except turn the lock: shut but unlocked.
- `DEMO_VARIANT=one_latch` draws in only the left latch; both arms pull, the right bolt holds the sash under its
  stop, and the arms then lower the sash and lock it. It was never tipped in past 30 deg.

`WINDOW_DEBUG=1` prints the state every 20 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/tilt-window-demo/*.json
ffmpeg -y -i ~/yam-eval/tilt-window-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz). The demo plan is about 42 s long.
- There is one fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is
  deterministic. Dream-1 is hosted and not seeded.
- A trial takes roughly 6 to 8 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Turn the lock open (bar at +45 deg), raise the sash 3 cm from the sill, and check both latches are out.
