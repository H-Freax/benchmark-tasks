# Take the tray off a high chair with a toddler in it: squeeze both release paddles and draw it straight out

Instruction given to the policy: `Take the tray off the high chair.`

The instruction gives no hints. The steps it leaves unstated: the tray is held by a ratchet (detents every 3 cm) that
only lets go while both side paddles are squeezed past 6 mm at the same time. Let go of either one and it catches
again at the nearest detent. The tray has to come straight out 12 cm until it leaves the rails, then be set down flat
on the bench. If you lift or tip the tray while it is on the rails, its back edge pushes up into the seated toddler's
forearms. More than 20 N there scores 0, latched. Dropping the tray also scores 0. Partial credit: 3 cm out 0.2,
6 cm out or off the rails 0.4, flat on the bench and let go 1.0.

Files: `task.json`, `remove_high_chair_tray.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/remove-high-chair-tray-two-releases/remove_high_chair_tray.py
```

Asserts the scoring rules on hand-built states and replays both arms' demo plans through the kinematics. Expected:

```
scoring: 13 cases pass
left: 21.0 s plan, worst IK 0.090 mm / 0.013 deg
right: 21.0 s plan, worst IK 0.090 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/remove-high-chair-tray-two-releases/remove_high_chair_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the tray off the high chair." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hct-dream
```

`task_score` is 1.0 once `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Use the same command with `--policy yam_scripted` (log dir `~/yam-eval/hct-demo`). The demo is open loop and
deterministic. Both arms come down over the paddle pockets at once. Each pinches its paddle against the fixed fin
beside it, so the closing gripper squeezes the paddle its full 8 mm. Together they draw the tray straight out 12 cm
off the rails, then 4 cm more to clear the rail blocks. They lower it flat onto the bench, let go and back away. Set
`HCT_DEBUG=1` to print the rail travel, paddles, tray pose, forearm force and score every 10 steps.

Degenerate checks (same command, with `DEMO_VARIANT` set in the environment) all score 0:

- `one_paddle`: the left hand squeezes its paddle; the right hand holds only the fixed fin. Both pull hard. The
  ratchet holds and the tray does not reach the first detent: 0.
- `lift`: both paddles squeezed, the tray lifted 5 cm straight up instead of drawn out. The tray rises about 1 cm
  into the forearms (about 9 N), the grip slips off the paddles, the ratchet catches again and the tray never clears
  the first detent: 0. A harder lift past 20 N on the forearms scores 0, latched.

## Videos and screenshot

```bash
for log in ~/yam-eval/hct-*/*.json; do inspect-robots video "$log" --ffmpeg "$(command -v ffmpeg)"; done
ffmpeg -y -i ~/yam-eval/hct-demo/frames/<run>/~f1~scene-0-e0~top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz). The demo succeeds in about 430 steps.
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Slide the tray back onto the rails to the first detent. Seat the manikin with its forearms just over the tray.
