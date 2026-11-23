# Lift a can out of a drawer divider row

Take the upright soup can out of the deep kitchen drawer and leave it standing in the marked area beside the drawer. The hand must fit between the dividers to grip the can directly. No cord, handle or tool is provided.

## Scene and reproduction

The two bamboo dividers represent **Lipper International model 8897 deep drawer dividers**. The [manufacturer specification](https://www.lipperinternational.com/products/bamboo-deep-drawer-dividers-set-of-two/) gives a thickness of 0.625 in, height of 5 in and adjustable length of 18.125–22 in: **15.875 × 127 × 460.375–558.8 mm**. The chosen length is 460.375 mm. These product dimensions come from the manufacturer's published specification and have not been physically measured. The drawer dimensions and layout below are scenario choices.

One **standard soup can**, diameter **67.7 mm**, height **101.9 mm** and mass **450 g**, stands at world **(0.160, +0.100) m**. Its exact simulated dimensions are diameter 67.66 mm and height 101.86 mm. The can is represented by a rigid cylinder with the stated dimensions and mass. The drawer floor and countertop are coplanar. The can starts 1 mm above the floor and settles.

The drawer interior is **460.375 × 300 × 150 mm**, with four 18 mm plywood frame boards and a 12 mm floor. Two fixed bamboo boards span its full internal length and form a **77.052 mm** clear row. The row axis is **7.971604°** from world +x. The can is at its longitudinal midpoint, **230.188 mm** from each end frame's inner face, **70 mm** from the near side's inner face and **230 mm** from the opposite side's inner face. This places the row toward one side of the drawer and leaves the robot base outside the frame.

For exact board placement, the local origin is the row midpoint at world (0.160, +0.100) m on the station tabletop. Local u follows the row, v is perpendicular to it, and z is up. All values in this table are millimetres; sizes follow (u, v, z).

| Object | Size | Local centre |
| --- | --- | --- |
| Bamboo divider, near | 460.375 × 15.875 × 127 | (0, −46.463522, 63.5) |
| Bamboo divider, far | 460.375 × 15.875 × 127 | (0, +46.463522, 63.5) |
| Plywood end frame, negative u | 18 × 336 × 150 | (−239.1875, −80, 75) |
| Plywood end frame, positive u | 18 × 336 × 150 | (+239.1875, −80, 75) |
| Plywood side frame, near | 460.375 × 18 × 150 | (0, −239, 75) |
| Plywood side frame, far | 460.375 × 18 × 150 | (0, +79, 75) |
| Plywood floor | 496.375 × 336 × 12 | (0, −80, −6) |

All seven boards are fixed. The divider tensioners and end pads are omitted; the task does not operate drawer slides. The pale landing outline is a **110 mm square** centred at world **(0.240, +0.280) m**. Its four fixed strips follow world x and y: two 2 × 110 × 0.5 mm strips centred at (0.186, 0.280) and (0.294, 0.280) m, and two 106 × 2 × 0.5 mm strips centred at (0.240, 0.226) and (0.240, 0.334) m. Their centres are 0.25 mm above the countertop. The left arm works on the positive-y side; the right arm starts in its raised ready pose. There are no other task objects. Dimensions and layout are also recorded in `assets/can_info.json` and `assets/drawer_info.json`.

## Scoring and physical trials

Record the trial at 20 Hz with calibrated top and side views. Use a ruler or callipers to establish the floor, divider tops, landing centre and gripper opening. Measure can tilt from vertical and hand clearance using a conservative envelope around the fingers and palm. Track the can centre over the last ten frames. Award the highest satisfied tier at the end:

| Score | Conditions |
| --- | --- |
| 0 | No witnessed upright extraction, the can remains in the drawer, final tilt is at least 5°, the setup is invalid, or an off-countertop event was observed |
| 0.25 | A continuous upright lift raises the can bottom at least 3 mm above the 127 mm divider tops, then the can is wholly outside the drawer and upright |
| 0.50 | Also standing on the countertop, bottom from −1 to +2 mm relative to its surface; both grippers open by at least 85 mm and both complete hand envelopes at least 30 mm away |
| 0.75 | Also within 15 mm on each world axis of the landing centre, with every pair of can-centre positions in the last ten frames within 2 mm |
| 1.00 | All full-score conditions persist for one second |

The extraction height is **130 mm above the floor** because the task is to remove the can from the row between the dividers. The fixed outer frame physically constrains the transfer. Divider contact does not affect the score. The can must be wholly outside the drawer before any nonzero terminal tier applies. Tilt and inaccurate placement can be repaired.

An observed can extent outside countertop bounds **x=[−0.19, 0.91] m, y=[−0.65, 0.65] m**, or a bottom more than **10 mm** below the surface, permanently yields zero for that trial. Before ten frames are available, stillness requires speed below **10 mm/s** and spin below **0.2 rad/s**. Invalid or missing object readings yield zero; unavailable hand readings prevent release credit. A direct jump to a finished pose without a continuous upward crossing earns zero.

Start with the can centre within 5 mm of the stated pose and tilt below 5°. Fixed-board centres must match within 0.5 mm, with rotation-matrix entries within 0.001 of the stated orientation. Restore the can upright at the row midpoint and reset the arms for each trial. A fresh episode clears trial history; an in-place reset requires a can-position jump greater than 100 mm back into the complete nominal layout. Slow restoration does not clear an off-countertop event.

## Run and score

Use Isaac Sim **6.1**, Isaac Lab-Arena **0.3.1** and `yam_arena`. The limit is **800 control steps at 20 Hz (40 seconds)**. Run these commands from the repository root.

Scripted demonstration:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lift-a-can-out-of-a-drawer-divider-row/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --seed 0 --no-prompt --store-frames --max-steps 800 --scorer task_score \
  --instruction 'Take the soup can out of the drawer and leave it standing in the marked area beside it.' \
  --log-dir /tmp/drawer-can-demo
```

Dream-1 trial:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/lift-a-can-out-of-a-drawer-divider-row/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --seed 0 --no-prompt --store-frames --max-steps 800 --scorer task_score \
  --instruction 'Take the soup can out of the drawer and leave it standing in the marked area beside it.' \
  --log-dir /tmp/drawer-can-model
```

Read the result JSON `status`, `results.metrics.task_score` and the trial's termination reason. The command's exit code alone does not establish the result.

Run local tests in the configured `yam_env` environment:

```bash
python -m pytest -q tasks/lift-a-can-out-of-a-drawer-divider-row/test_task.py
```

## Recorded GPU results

Both runs used commit **`cd25002c2772b1a107682cb971971d05f3c744cf`**, seed 0 and the environment above on an NVIDIA L40S. Reported times are wall-clock time for these runs.

| Run | JSON status | Score | Control steps | End reason | Wall-clock time |
| --- | --- | --- | --- | --- | --- |
| Scripted demonstration | success | 1.0 | 279 | success | 53.42 s |
| Dream-1 | success | 0.0 | 800 / 800 | max_steps | 119.70 s |

The screenshot shows the fixed initial layout. Both videos use the top camera and H.264 encoding. Task geometry is generated from analytic shapes; no external can mesh or texture is included.
