# Install a bi-pin fluorescent tube in a strip fixture with rotary lampholders

Instruction given to the policy: `Install the new tube in the fixture.`

The instruction says nothing about how the holders work. The scored steps it leaves unstated: pick the 2 ft T8 tube
up with both hands with its pins upright, lower both pin pairs into the holders' slots together (the right holder
is sprung and stands 4 mm inside the tube's length, so pressing the tube down onto its bevelled guard pushes it
4 mm out), then turn the tube a quarter turn so the rotors lock the pins across the slots, and let go. Let go before the
turn and the tube is counted as fallen out (in an overhead fixture the pins only sit in open slots).

Files: `task.json`, `bipin_tube_tombstone_twist.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/bipin-tube-tombstone-twist/bipin_tube_tombstone_twist.py
```

Asserts the scoring rules on hand-built states and replays both arms' demo plans (and the degenerate variants)
through the kinematics. Expected first line: `scoring: 8 cases pass`.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/bipin-tube-tombstone-twist/bipin_tube_tombstone_twist.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the new tube in the fixture." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/bipin-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/bipin-demo`). Open loop and deterministic. Both
arms grip the tube 20 cm either side of its middle (fingers across it), lift it, carry it over the holders and
lower it until its end caps rest on the holders' bevelled guards, and let go. With fingers closed they press the
glass down 3 cm in from each end: the right end cap cams the sprung holder 4 mm out and both pin pairs drop to the
slot floors. They then press the fingertips on top of the glass and sweep them together 32 mm forward, rolling the
tube a quarter turn into the rotors' 90 deg detent, and lift away. `BIPIN_DEBUG=1` prints the tube, seats, slide, rotor
angles and score every second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment), all score 0:

- `no_turn`: seats both pin pairs, lets go and parks without turning the tube.
- `half_turn`: seats it and rolls it only part of the way (a 10 mm sweep); the detent springs the rotors back to 0
  and it is let go.
- `drop`: lifts the tube 15 cm and lets go over the bench.

## Videos and screenshot

```bash
for log in ~/yam-eval/bipin-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/bipin-demo/frames/<run>/~f1~scene-0-e0~top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,600 steps (80 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Turn the tube back a quarter turn, lift it out and lay it on the bench at its mark, pins upright.
