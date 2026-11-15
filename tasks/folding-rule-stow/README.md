# Folding-rule storage

Put the full-size, partly extended wooden folding rule into its storage recess.
The final outcome is scored; any grasp, fold order or path is allowed.

## Run

Use IsaacLab Arena with the `yam_arena` embodiment and Inspect Robots. Source
local runtime and model credentials privately; they are not part of this task.
Set `ROBOCURVE_GPU_LOCK` privately to the common lock file used by every simulator
on the host. This is an operator parameter, separate from the task instruction.
From the repository root:

```sh
flock "${ROBOCURVE_GPU_LOCK:?set shared GPU lock path}" inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/folding-rule-stow/folding_rule_stow.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=720 --instruction "Put the folding rule into its storage recess." --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir /tmp/folding-rule-reference
```

Replace `--policy yam_scripted` with `--policy dream` for the model trial and
choose a new output directory. The limit is 1,200 controls, or 60 simulated
seconds at 20 Hz. The fixed Cartesian reference lasts 45.5 seconds. Local wall
time depends on rendering and model latency. Seed 0 and zero object jitter
reproduce the submitted fixture.

## Objects and materials

The nominal 2 m rule has ten rigid leaves at 200 mm hinge pitch and nine real
revolute joints. Each leaf is 204 x 16 x 3 mm, including 2 mm overlap beyond each
hinge, with a 3.4 mm layer pitch. Engineered density 650 kg/m3 gives 6.3648 g per
leaf, 63.648 g total. The 1.10 x 1.30 m station table top is at z=0.7475 m. Reset
origin is (0.36, 0.205, 0.7493) m, oriented toward -y. The first seven hinges
start folded at 180 degrees and the final two open at 0 degrees; the complete
reset fits on the real table. The fixed polymer pocket is centered at
(0.32, -0.18) m, with 224 x 36 x 50 mm clear space, a 4 mm floor and 4 mm walls.

Coated wood and polymer use static/dynamic friction 0.4/0.3, zero restitution
and average material combination. Robot rubber-pad materials are unchanged.
Each hinge has a disclosed engineering over-center spring: 0.012 Nm/rad,
rest -20 degrees below the 90-degree midpoint and 200 degrees above it,
0.0003 Nms/rad damping and 0.002 Nm dry friction. Endpoint preload is 0.00419 Nm.
The native joint coordinate is centered at the physical 90-degree midpoint to
retain real 0..180-degree stops. Connected leaves still collide.

The design references the ordinary Series 600 Type 617 wooden folding-rule
family. [STABILA product information](https://www.stabila.com/en/products/details/series-600-wooden-folding-rule.html)
supports the 2 m, ten-segment wood tool; [STABILA technology](https://www.stabila.com/en/products/quality-and-technology/folding-rules.html)
describes spring-loaded joints; [retailer dimensions](https://www.reichelt.com/de/en/shop/product/stabila_wooden_folding_rule_series_600_type_617-106851)
report 16 mm width and 3 mm thickness. No OEM CAD, textures or task code are
included. Overlap, mass, hinge torque, layer spacing, friction and the generic
storage recess are engineering approximations, not measured OEM properties.
Wood flexibility, fracture and measurement accuracy are outside the model.
All supplied geometry and code are authored here and released under MIT.

## Score and controls

A real leaf falling more than 30 mm below the table latches zero. Otherwise,
0.25 is orientation-independent compactness progress: the maximum 3D distance
between any pair of all 80 collision corners is at most the 230.09 mm diagonal
of a 222 x 34 x 50 mm box. This tier does not establish pocket fit. Standing the
unfolded rule vertically cannot shorten that distance. 0.5 requires all corners inside the actual
pocket volume, with 0.6 mm floor tolerance, and measured pocket-floor reaction
above 0.05 N. Full credit also requires every leaf below 15 mm/s and 0.1 rad/s,
and every actual robot collision piece at least 5 mm away, continuously for
one second. Missing support or incomplete robot geometry cannot yield full
credit. Containment never uses only body centers. Geometry separation uses
conservative bounds around each actual URDF collision piece.

The reference is fixed from documented reset geometry. It folds the terminal
leaf, releases and regrips the overlapping pair, folds the penultimate hinge,
then carries the complete tool and releases it above the pocket rim. It reads
no live object poses or joint angles. Live state is used only by passive
material forces, scoring and diagnostics. There are no runtime object pose
writes, gripper welds, threshold captures or hidden contact exclusions.

Diagnostic controls use the same code and scorer: `FOLD_BRANCH=stop` starts
already folded and supported in the recess; `DEMO_VARIANT=always_stop` leaves
the unfinished ACTION fixture alone; `DEMO_VARIANT=hold_above` completes the
fold/carry but deliberately keeps the package grasped above the target. The
STOP variant is a control in this task family, not another submitted task.
Safe harmless action in an already completed fixture is not penalized by the
functional outcome scorer. Parameter environment variables support explicit
sensitivity studies; the submitted setup uses their source defaults.

Reset restores all ten leaves and nine joints with the robot parked, empties
the target pocket and clears scoring history. Nothing is consumed.


## Qualification and recordings

The two complete references use exactly the same final Python files as the
model and controls. The task entry SHA-256 is
`2b4b0146374cc2b2e002345f4428410d6168e20832d1880d27e8ee08a1886950`.
The dependency hashes are:

| File | SHA-256 |
| --- | --- |
| folding_rule.py | `0dd0e0da0c0833229d72561f989c562707d4607e4c7d2982a2596862b886d88e` |
| mechkit.py | `7b359b56d43fc1ba37fb653ce6fe53141ee69b199a7695cf268afb8c9bb977ea` |
| two_leaf_reference.py | `162294bdf18dcd56b12204f8e0c65c1699b9db6d733dcb6a7e3e3860edeaae1e` |

Metadata score/media reporting and this README are filled after evaluation;
those reporting edits do not change any Python, dimensions, physics, reference
or score. Each evaluation preserved matching source copies, hashes, terminal
results, action logs and all three cameras' stored observations.

| Trial | Score | Controls | End reason | Measured wall time |
| --- | ---: | ---: | --- | ---: |
| Scripted reference 1 | 1.00 | 872 | success | 361.4 s |
| Scripted reference 2 | 1.00 | 872 | success | 334.3 s |
| Already completed STOP | 1.00 | 39 | success | 25.8 s |
| Incorrect ACTION stop | 0.00 | 60 | max_steps | 31.6 s |
| Real held-above negative | 0.25 | 971 | policy_stop | 332.6 s |
| Dream-1 | 0.00 | 1200 | max_steps | 402.5 s |

The model gets the full 1,200-control budget. The reference reaches the final
state after 872 controls (43.6 simulated seconds). The STOP and incorrect-stop
checks are matched completed/unfinished fixtures in the same family. They do
not establish a separate T3 qualification or another independent task.

Verified runtime: YAM SDK commit
`bd62771e60320433893bd1d2df0fc79b5ddcd6d1` (clean checkout), yam-arena 0.1.0,
inspect-robots 1.0.0, IsaacLab 17.0.2, Isaac Sim 6.1.0.0, Torch 2.11.0+cu128 and
NumPy 2.3.1. The submitted MP4s contain every stored top-camera observation at
20 fps, with no encoder cropping, interpolation or extra resizing. Stored
observations are 1280 x 720; this SDK renders 640 x 360 and resizes observations
to the requested dimensions. The setup screenshot is the actual reset frame.
All supplied MP4s are H.264/yuv420p and decode completely under the media limit.
