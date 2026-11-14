# Raise a folded ironing board to its working height and lock it there

Instruction given to the policy: `Put the board up at its working height.`

The instruction gives no hint about the mechanism. The steps it leaves unstated: squeeze the red release trigger
behind the nose handle (the height ratchet only moves while it is held), lift both ends together until the rack
reaches notch 3 (the blue tag), let the trigger go so the pawl drops in, then let go of the board. Lifting without
the trigger binds; forcing it strips the pawl and the board collapses; letting the board fall while the pawl is out,
or tipping it, scores 0.

Files: `task.json`, `ironing_board_unfold_to_lock.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/ironing-board-unfold-to-lock/ironing_board_unfold_to_lock.py
```

Asserts the scoring rules on hand-built states and replays every demo plan through the kinematics. Expected:
`scoring: 11 cases pass`, then one IK line per arm and variant (worst IK under 0.1 mm).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ironing-board-unfold-to-lock/ironing_board_unfold_to_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the board up at its working height." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/iron-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/iron-demo`). Open loop and deterministic. The left
arm closes round the nose handle and the trigger together, squeezing the trigger shut; the right arm pinches the
tail handle; both lift 5.8 cm (the arms sag about 1.3 cm under the board) over 5 s; the left hand opens (the pawl drops into notch 3), then the right lets go
and both park. `IRON_DEBUG=1` prints rack, pawl, trigger and score every second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment), all score 0:

- `notch1`: the same, but stops and locks at notch 1.
- `forced`: grips both handles without touching the trigger and pulls up 4 cm. The locked pawl binds: the whole stand lifts and tilts (about 12 deg), and the board is set back down at notch 0.
- `on_end`: lifts only the tail end high, swinging the board up onto its nose.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Squeeze the trigger, lower the board to folded (notch 0), let go.
