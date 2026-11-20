# Grease a driveshaft slip joint: three shots through its grease fitting with a hand grease gun

Instruction given to the policy: `Give the slip joint's grease fitting three shots from the grease gun, then put the gun back on its mat.`

Files: `task.json`, `grease_chassis_fitting_with_grease_gun.py` (scene, `success()`, partial-credit `score()`, the
scripted `demo()` and an offline self-test), `mechkit.py` (multi-box and convex rigid parts, sprung slides and
hinges, per-part friction, per-episode hooks), `README.md`.

## What is modelled

- The fitting is a straight 1/4-28 SAE J534 grease fitting at real size: 8.0 mm hex, 4.8 mm neck, a 6.6 mm head
  with a steep shoulder behind and a rounded nose in front, 11.5 mm proud of the barrel. Its head is one convex
  collider.
- The gun is one rigid body (barrel, pump head, handle, 10 cm rigid pipe, coupler with a 7.2 mm bore and an inner
  seat). Its four jaws are separate 20 g bodies on radial slides just inside the mouth, each on a 600 N/m spring
  with 1.2 N of preload, standing 1.0 mm into the bore. Pushing the coupler on spreads them over the head's nose
  (about 8 N); they close behind the shoulder when the nose reaches the seat, and it then takes about 16 N to pull
  the coupler straight off. Friction between jaws, bore and fitting is 0.12 (greased steel). No joint is switched
  on or off during an episode.
- The pump lever is a sprung hinge on the pump head (18 degrees of travel, about 13 N at the grip at full stroke).
- Grease is not simulated. A lever stroke (under 4 degrees to 15 degrees or more) counts as a shot into the
  fitting only if the coupler is home for the whole stroke: mouth 3.6 to 5.2 mm from the barrel, within 1.2 mm and
  6 degrees of the fitting's axis, all four jaws closed. Any other stroke is spilt. Six shots into the fitting
  blow the joint's seal and latch a score of 0, as does dropping the gun off the bench.
- Score = 0.10 coupled + 0.20 per shot into the fitting (up to three; -0.20 per shot beyond; -0.10 if any stroke
  was spilt) + 0.15 coupler off the fitting + 0.15 gun lying on its mat, let go and still.

A real trial uses an obtainable 3 oz pistol-grip grease gun with a 4 in rigid pipe and a standard four-jaw
coupler, a straight 1/4-28 grease fitting screwed into a 50 mm steel tube (or a real slip yoke) strapped to two
bench stands, and a rubber mat. Real physical trials verify delivery using macro video monitoring of coupler seating
and stroke displacement, collect expelled grease on pre-weighed absorbent wipes, and measure grease mass delivery
on a 0.01 g analytical balance against the calibrated ~0.85 g/stroke metered gun output. Rebuilding driveshaft slip
joints and universal joints on bench fixtures is standard overhaul practice per OEM service manuals (e.g. Dana Spicer
DSSM-0100 and Meritor MM-96147). Automotive service technicians and mechanics (SOC 49-3023, 676,570 employed, BLS
OEWS May 2023) and bus and truck mechanics and diesel engine specialists (SOC 49-3031, 294,000 employed) routinely
perform this procedure.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots.

## Offline check, no GPU

```bash
python tasks/grease-chassis-fitting-with-grease-gun/grease_chassis_fitting_with_grease_gun.py
```

It asserts the scoring rules on hand-built states (coupled latch, shot counting, spills, burst seal, mat and
release) and the jaw and bore geometry, then replays every demo plan through the kinematics at 20 Hz. Expected
output starts:

```
scoring: 23 cases pass
full right: 37.4 s plan, worst IK 0.079 mm / 0.014 deg
full left: 28.3 s plan, worst IK 0.078 mm / 0.013 deg
```

## Run and score one trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/grease-chassis-fitting-with-grease-gun/grease_chassis_fitting_with_grease_gun.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the slip joint's grease fitting three shots from the grease gun, then put the gun back on its mat." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir runs/grease-fitting
```

For Dream-1 use `--policy dream` with the same arguments. `DEMO_VARIANT=air|short|five|left_on` plays one of the
wrong ways below with `yam_scripted`; `GREASE_DEBUG=5` prints the readouts every five steps.

## Results (seed 0, no jitter)

| Run | Policy | Seed | task_score | Outcome |
| --- | --- | --- | --- | --- |
| Scripted demo 1 | yam_scripted | 0 | 1.0 | Succeeded (seated coupler, 3 shots, uncoupled, returned gun to mat) |
| Scripted demo 2 | yam_scripted | 1 | 1.0 | Succeeded (independent seed verification) |
| Model run | Dream-1 | 0 | 0.0 | Hit step limit (no coupling) |
| Negative control: air | yam_scripted | 0 | 0.0 | Coupler stopped 3 cm short of fitting (shots spilt) |
| Negative control: short | yam_scripted | 0 | 0.0 | Coupler 4.5 mm short of home (not seated) |
| Negative control: five | yam_scripted | 0 | 0.60 | Over-greased (5 shots into fitting) |
| Negative control: left_on | yam_scripted | 0 | 0.70 | Incomplete return (left arm holds gun on mat) |

