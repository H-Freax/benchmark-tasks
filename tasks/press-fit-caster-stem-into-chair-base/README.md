# Press five casters into an upturned office-chair base, holding the base steady

Instruction given to the policy: `Fit the five wheels to this chair base.`

The instruction gives no hint. What it leaves unstated: each caster stem has a split ring that only passes its
groove under 35 N of axial load with the stem within 5 deg of the socket axis; a sideways load above 8 N at the ring
cams it out, and over 30 N more than 12 deg off axis bends the stem (that caster scores 0 for the rest of the
run). Pushing on one arm of the upturned base lifts the opposite arm, so the other hand has to hold the base down.
The base sliding more than 10 cm, dropping below the bench or tipping over 45 deg scores 0, latched. Partial credit:
0.2 per socket holding a caster that went past its ring and is home, counted with the grippers clear and everything still.

Files: `task.json`, `press_fit_caster_stem_into_chair_base.py` (scene, `success()`, partial-credit `score()`, the
scripted `demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/press-fit-caster-stem-into-chair-base/press_fit_caster_stem_into_chair_base.py
```

Asserts the scoring rules on hand-built states and replays both arms' plans (demo and the three degenerate variants)
through the kinematics. Expected first lines:

```
scoring: 12 cases pass
demo left: 109.9 s plan, worst IK 0.091 mm / 0.014 deg
demo right: 109.9 s plan, worst IK 0.097 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/press-fit-caster-stem-into-chair-base/press_fit_caster_stem_into_chair_base.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the five wheels to this chair base." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/caster-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/caster-demo`). Open loop and deterministic, about
2,120 steps to success. The left arm fits sockets 0, 4 and 3 with casters from its racks, the right arm sockets 1
and 2. For each caster, one arm pinches the wheel block, carries it over the socket, sets the stem on the ring and
drives straight down past it; meanwhile the other arm presses its fingertips on the arm across the hub so the base
does not rock. `CASTER_DEBUG=1` prints the ring loads, angles, depths and score every 10 steps.

Degenerate checks (same command, `DEMO_VARIANT` in the environment):

- `loose`: sets every caster in its socket on the ring and lets go without pushing (6 mm in, resting on the ring): 0.
- `hub_hole`: drops every caster into the hub's column bore instead of the arm sockets: 0.
- `no_brace`: pushes every caster as the demo does but never holds the base. The near sockets 0, 1 and 4 rock the base
  about 15 deg and the stems bend; on the two far sockets the pushing arm's own links rest over the base and steady it,
  so those two go in: 0.4.

## Videos and screenshot

```bash
for log in ~/yam-eval/caster-*/*.json; do inspect-robots video "$log" --ffmpeg "$(command -v ffmpeg)"; done
ffmpeg -y -i ~/yam-eval/caster-demo/frames/<run>/~f1~scene-0-e0~top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (120 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- About 7 minutes of wall clock for a demo run on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Pull the five casters out of the base and stand them back in the racks; set the base back on its hub at its mark.

## Submitted runs

- Demo (`yam_scripted`): `task_score: 1.0` in two separate runs, 2,122 steps each, ended by success.
- Dream-1: `task_score: 0.0` at the 2,400-step limit.
- Degenerate checks: `loose` 0.0, `hub_hole` 0.0, `no_brace` 0.4.
