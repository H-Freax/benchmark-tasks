# Brush the inner bottom and shoulder of a narrow-neck bottle

Use the bottle brush to remove visible surrogate film from the inner bottom and shoulder, then remove the brush and safely release both objects. Any ordinary grasp, support or placement is permitted. Nothing is attached to the robot.

This is a development candidate. All completed scripted reference trials scored0.0. The current reference stopped normally at a conservative neck-clearance guard before cleaning. Dream-1 completed 4200 actions and scored0.0. A full-score demonstration is not yet available; the current model and failed-reference media are supplied for review.

## Setup and reset

All original dimensions, explicit poses, material assignments and mark areas are in `geometry/geometry.json`. Table height is747.5mm. The loose bottle stands atX300/Y0mm. The horizontal brush points+X, with its head origin atX120/Y-180/Z770.5mm. A3mm polypropylene return tray supports the head; a20x35x17.5mm wooden rest supports the existing steel stem atX250/Y-180mm. The original brush is240mm long: headz0-40mm, steelstem40-200mm, polymerhandle200-240mm. The reference grasps the original handle atz220mm. All630 stock YAM collision hulls and original drives are retained.

The bottle is139mm tall,65mmOD,59mminnerbody,44mmneckID and4mmclosedbase. The20mm shoulder joins its body to the25mm neck. Glass density integration gives209g; the original brush geometry/material densities give39g. These are original reproducible engineering equivalents, not manufacturer dimensions or weights.

For a physical visible-film surrogate, use washable orange poster paint mixed with water1:4 by mass; deposit1microlitre per supplied2-2.5mm stencil mark, dry10minutes at20-25C, and photograph the24bottom/24shoulder marks before and after. Use a bottom view and a mirror or inexpensive endoscope for the shoulder. Mark center, normal and area are supplied in the geometry file. This original protocol and the simulated removal thresholds have not been calibrated against physical removal. No microbial or medical claim is made. Do not use the marked test bottle for feeding.

Reset restores documented initial poses, all48marks and counters. The task never writes object poses/velocities or applies object forces during manipulation.

## Running

Use the pinned YAM/Inspect environment with Python3.12.13 and set these task-local paths. The runtime requires NumPy and SciPy; native observation uses Isaac/PhysX APIs supplied by the runtime.

```sh
export TASK_ROOT=/absolute/path/to/tasks/brush-narrow-neck-bottle
export RUN_DIR=/absolute/path/to/a/new/run-directory
export ROBOCURVE_GEOMETRY_ROOT="$TASK_ROOT/geometry"
export ROBOCURVE_TRACE_DIR="$RUN_DIR/trace"
inspect-robots run --embodiment yam_arena --policy dream -E task="$TASK_ROOT/native/task.py" -E object_jitter=0 -E control_hz=20 --instruction 'Brush the visible film from inside the bottle, then remove the brush and leave both items safely released.' --scorer task_score --max-steps 4200 --seed 0 --no-prompt --store-frames --log-dir "$RUN_DIR/inspect"
```

Configure the Dream-1 policy connection in the runtime before a model run. The model task entry contains no reference controller. For the scripted reference use `--policy yam_scripted` and `-E task="$TASK_ROOT/native/reference.py"`. On runtimes requiring the non-UJITSO cooking backend, use `native/local_task.py` for the model or `native/local_reference.py` for the reference. These wrappers only select the collision-cooking dispatch and record its actual setting; geometry, drives and physics are identical. No simulator is launched merely by importing a task file.

The nominal reference contains1420joint targets over71seconds, with actual-arrival gates allowed up to4200controls/210seconds. It requires five fresh real two-pad observations for both objects before loaded movement; it bounds actual material steps, checks the whole brush through the neck and stops normally on missing contact, excessive material slip, joint limits or arrival timeout. The scoring code does not read these phases or gates. Earlier completed reference trials scored0.0; the current controller has not established a successful demonstration. Physical execution time and throughput are still unknown; the trial wall-time cap used for the completed current development run was1000seconds.

## Scoring and fidelity

`native/cleaning.py` accumulates marked-area removal only when real native bottle/brush contact is at least0.1N, the contact belongs to the40mmhead, the marked point intersects the actual head within0.5mm modeling tolerance, and cumulative relative tangential travel reaches20mm. Duplicate observations, stationary contact and shaft-only contact cannot clean marks. Hiding a removed visual mark never changes physical geometry.

Score0below25%removedarea;0.25at25%;0.5at60%. Score1requires90%overall and80%eachbottom/shoulder, the head outside the bottle, both objects above the table withlinearvelocity<20mm/s andangularvelocity<0.1rad/s, handpoints30mmclear and knownzero normal/tangent contacts against all18robotrigidbodies continuously2seconds. The native task, grader and contact observer are the executable scoring authority. Missing data fails closed and the first observation error is persisted.

The native bristle envelope is force-based500N/m compliance,0.05Ns/m damping and0.45friction. This unmeasured engineering approximation preserves real head/neck/shoulder contact but does not simulate individual bristles or biological cleaning. The reference uses ordinary stock joint targets; no task or robot force settings are changed.

The close public `wash_cup` prior already includes one hand holding a cup while the other brushes its interior: https://huggingface.co/dddddsds31341/pi05-wash_cup_100-10k . This candidate adds contact access behind a narrower neck; a distinct capability claim remains provisional. CDC cleaning context: https://www.cdc.gov/hygiene/about/clean-sanitize-store-infant-feeding-items.html . Neither source is copied into scene assets.

## License

Original task code and engineering geometry are supplied under the included MIT license. The primitive/lifecycle adapter retains its MIT notice. Stock YAM assets remain external runtime dependencies under their upstream licenses; they are not redistributed here. No OEM CAD, copied web page, internal review, credential, private evidence or experiment archive is included.


## Recorded submission evidence

The uploaded Dream-1 attempt completed its full allowed action budget and scored 0.0. The uploaded scripted reference also scored 0.0; it is an unsuccessful attempt, not a full-score demonstration. Both videos are original simulator camera streams at 20 fps. The task is submitted with the implementation and these current results; full-score feasibility remains unresolved.
