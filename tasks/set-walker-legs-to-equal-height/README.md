# Set all four legs of a folding walker to the marked height

Instruction given to the policy: `Raise the walker to the height shown by the band.`

A folded aluminium walker lies flat in a bench fixture, legs pointing away from the robot. Every leg is an inner
tube with a rubber foot sliding in the frame's outer tube, locked by a sprung push pin through one of five holes
2.5 cm apart. All legs start at hole 1; the red band marks where each foot's back edge sits at hole 3. A leg only
slides while its pin is held down, so one hand holds the pin while the other pulls the foot; the pin must snap
back up into a hole when let go, or it rides on the tube wall and the leg stays loose.

Files: `task.json`, `set_walker_legs_to_equal_height.py` (scene, pin rule, `success()`, partial-credit `score()`,
the scripted `demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory),
`README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/set-walker-legs-to-equal-height/set_walker_legs_to_equal_height.py
```

Asserts the scoring rules on 11 hand-built states and replays the demo and every variant through the kinematics
(worst IK about 0.1 mm).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/set-walker-legs-to-equal-height/set_walker_legs_to_equal_height.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the walker to the height shown by the band." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/walker-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted`. Open loop and deterministic. Leg by leg (inner left, inner right,
outer left, outer right), the arm on the leg's side pinches the foot, the other arm presses the pin with closed
fingers, the foot is drawn out 5 cm, the pin hand lifts (the pin snaps into hole 3), then the foot is let go.
`WALKER_DEBUG=1` prints every leg's travel, pin and lock once a second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment):

- `no_press`: pulls every foot without pressing the pin. Legs stay at hole 1: 0.
- `midway`: lets the pins go 3.7 cm out, between holes 2 and 3. Pins ride, legs loose: 0.
- `overshoot`: every leg to hole 4: 0.
- `pull_out`: the first leg pulled past hole 5 out of its tube, the rest done right: 0 (latched).
- `mixed`: two legs to hole 3, two to hole 4: 0.5 (only the matched legs count).

## Videos and screenshot

```bash
for log in ~/yam-eval/walker-*/*.json; do inspect-robots video "$log"; done
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz); the demo takes about 1,290.
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Press each pin and push every leg back to hole 1.
