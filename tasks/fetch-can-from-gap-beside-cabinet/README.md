# Retrieve a soup can from the gap beside a cabinet

Retrieve the upright soup can directly with the robot's own hand and leave it standing inside the marked landing area. Keep it upright and avoid scraping the two fixed panels. The scene is a cabinet side beside a wall, with the table as its floor; neither panel represents an appliance. One arm performs the retrieval on the bimanual YAM station.

**CPU feasibility passed; GPU physics and demonstration score remain pending.** The full scan contains 4,032 groups and 1,728 passing groups. The selected group has the smallest scanned nominal per-side margin, 2 mm, and a certified minimum robot-to-panel SAT margin of 4.493 mm. This establishes a feasible commanded geometric path, not a demonstrated successful PhysX grasp.

## Scene and measured grasp

The real Arena `tomato_soup_can_ycb_robolab` mesh has native bounds 67.854 × 67.750 × 102.037 mm. `Asset(extent=0.066)` scales it uniformly to 66.000 × 65.899 × 99.249 mm. Its actual 30 mm grasp band is narrower than 85 mm and taller than 15 mm. The can is grasped at a TCP height of 65 mm, with the fingers pointing down and closing along the gap's length.

Dimensions, standing bounds, volume, estimated uniform-mesh centre of mass and density are measured by the external build script and stored in `assets/can_info.json`. `task.py` reads that file. The estimated filled density is 1,409.9 kg/m³; preserving it after scaling gives a mass of 414.1 g. The native library mass is 450 g. The geometric centre-of-mass estimate is not a measured distribution of soup; the built-in library's automatic PhysX mass properties remain in use. No custom USD or replacement can primitive is distributed.

| Selected parameter | Value |
| --- | --- |
| Can centre, x / y | 0.140 / −0.080 m, on the right arm's side |
| Landing centre, x / y | 0.140 / −0.220 m |
| Panel length / thickness | 240 / 18 mm |
| Panel height | 114.249 mm = measured scaled can height +15 mm |
| Gap width | 71.999 mm = measured transverse hand envelope +4 mm |
| Nominal hand margin per side | 2 mm |
| Grasp yaw | +90°, closing along the gap length |
| Grasp TCP above table | 65 mm |
| Travel TCP above table | 183.249 mm; can bottom clears panel tops by 4 mm |
| Release TCP above table | 69 mm; nominal can-bottom release gap 4 mm |
| Full demo duration / sampled frames | 10.3 s / 207 at 20 Hz |
| Worst full-demo IK position / angle error | 0.082102 mm / 0.013817° |
| Minimum joint-limit margin | 5.510° |
| Minimum robot/panel SAT margin | 4.493454 mm, active right fingertip at 3.0 s |
| Reserve test, every waypoint 10 mm higher | 0.080332 mm / 0.012612° worst IK error |

Measured at a nominal 66 mm pad aperture, the distal pad thicknesses are 6.333 and 6.339 mm; the distal pads' outer span is 78.734 mm. The complete moving finger pair spans 113.547 mm along the opening direction and 67.999 mm transversely. The latter is the gap-normal dimension after yaw; the former lies along the 240 mm panel length. The gripper housing spans 102 × 65 × 74 mm. The scan never substitutes the can's diameter for the hand envelope.

At the selected grasp pose, the housing bottom is 135.531 mm above the table, the wrist-link bottom 199.029 mm, and the next forearm-link bottom 206.303 mm. None intersects the 114.249 mm panel-height slab. Their measured world spans and exact slab-clipped bounds are recorded in `selected_path_checks.json`; a null clipped section means the link stays above the slab. The measured can grasp band is 64.318 × 63.773 mm across and 30 mm high.

The selected wall height follows the revised scan range, can height +{5,15,25,40} mm. It is lower than the earlier rejected 195 mm wall, whose full path had 12.877 mm / 48.241° IK errors and −21.746 mm minimum geometric clearance. The fingers enter the restricted space, but the wider housing, wrist and forearm mostly stay above its top. This task does not claim that the full forearm passes between the panels. The measured path still constrains actual finger clearance to a few millimetres. Both panel ends remain open; a different approach from an end may be possible and is a known construct-validity risk.

## Complete parameter scan

`parameter_scan.csv` is the full per-group table. It includes x, y, arm, yaw, wall height, gap width, nominal per-side margin, worst IK position and orientation residuals, minimum SAT margin, joint margin, frame count and the limiting link, wall, time and segment. `scan_summary.json` records counts, environment, method, grid and the selected group. `selected_path_checks.json` contains the detailed selected trajectory check. `preflight_measurements.json` is retained historical evidence for the rejected 195 mm-wall/near-end-exit path; it does not describe the delivered demo.

Grid: x = 0.14 to 0.26 m in 0.02 m increments, |y| = 0.08 to 0.24 m in 0.02 m increments, both arms on their own side, yaw ±90°, four wall offsets and four gap offsets: 7 × 9 × 2 × 2 × 4 × 4 = 4,032 groups. Four gap widths share the same robot motion, so 1,008 unique trajectories are evaluated; every row still represents all 207 `ArmPlan.action7(t)` calls. There is no early exit after an IK failure.

The full path moves to a hover above the can, descends vertically, closes, lifts vertically until the can bottom clears the wall, carries sideways to the landing area, lowers, opens, and retreats. There is no extraction around the near panel end. All object handling uses downward orientation, translation, yaw and opening/closing.

All 315 shipped convex robot collision hulls are checked against both panels for both arms at every 20 Hz frame, including the inactive arm. AABB separation is used only to safely prune distant hulls. Narrow checks use the complete convex separating-axis set: both shapes' face normals and hull-edge × box-edge axes. Projections are shared across widths, not omitted. A positive SAT margin is a certified lower bound on Euclidean separation; negative means convex-model intersection. Values at 20 mm are censored lower bounds (at least 20 mm), since this already exceeds the passing threshold. This is a geometric CPU test, not a PhysX contact-force measurement.

An initial whole-link AABB screen was deliberately replaced by the complete scan: it rejected safe short-wall paths because it included wide finger parts above the wall. An independent SciPy convex-hull edge reconstruction in `test_task.py` checks the selected demo against **all fixed structures**, including the marking strips, rather than trusting the scan CSV or its counters alone.

The required pass criteria are strict position error <1 mm, angle error <1°, minimum SAT margin ≥3 mm and nominal per-side margin ≤10 mm. The selected nominal 2 mm is the minimum possible in this grid: smaller values were not requested. There are ties at this margin; this chosen right-arm path also keeps the limiting close clearance at the fingers rather than the fixed arm base. No global claim over continuous x/y values or untested paths is made.

## Run and score

Use the existing approved `yam_env` interpreter, referenced below as `YAM_PYTHON`; do not create another Python environment. Run commands from this checkout's root. The CPU checks need real `yam_arena`, NumPy, SciPy and pytest, but no Isaac Sim import or GPU.

```bash
PYTHONDONTWRITEBYTECODE=1 "$YAM_PYTHON" -m pytest -q -p no:cacheprovider \
  tasks/fetch-can-from-gap-beside-cabinet/test_task.py
```

On a GPU host with the existing Arena/YAM/Inspect Robots setup, first run **200 steps** with fixed reset and the scripted policy, never Dream-1:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fetch-can-from-gap-beside-cabinet/task.py \
  -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 200 \
  --instruction 'Retrieve the soup can and leave it standing in the marked area. Keep it upright and avoid scraping the panels.' \
  --store-frames --log-dir /tmp/can-gap-smoke
```

Check the smoke before running the complete demo:

- Spawn: can dimensions 66.000 × 65.899 × 99.249 mm, bottom gap 1.000 mm, no overlap; panels 114.249 mm tall with a 71.999 mm gap. The first `task_progress` must show `armed=true`, no violation and score 0.
- Path: worst CPU IK 0.082102 mm / 0.013817°; the rejection limits are 1 mm / 1°. No joint exceeds its URDF limit. CPU minimum model margin is 4.493454 mm; investigate any contact or observed scrape. A CPU certificate does not guarantee the simulated joint tracking stays within that clearance.
- Can: tilt stays below 5°; its bottom clears the wall top by about 4 mm during the carry; the nominal release gap is 4 mm. Reject a fall, bounce, throw, tip or can penetration even if a partial score appears.
- Final: can bottom within 3 mm above the table, centre within 15 mm of the mark along x and y, positions within 2 mm over the 10-frame window, and the framework's success conditions held for 1 s. Check the video for an **open** gripper with at least **30 mm** clearance from the can; inspect the actual termination frame, since success can terminate a rollout before all waypoints are consumed.
- Debug cadence: `task_progress` appears only on steps 1, 20, 40, …, 200, or until earlier termination. Score tops out at 0.75; the framework alone promotes held success to 1.0.

Then run the complete 400-step trial (20 s budget; the 10.3 s plan plus the default 3 s policy hold fits):

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fetch-can-from-gap-beside-cabinet/task.py \
  -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 400 \
  --instruction 'Retrieve the soup can and leave it standing in the marked area. Keep it upright and avoid scraping the panels.' \
  --store-frames --log-dir /tmp/can-gap-demo
```

Both commands are handoff instructions and were **not** run locally. Local `yam_env` does not contain `inspect_robots` or its CLI. No GPU physics, video or achieved full score is claimed. Nominal layout is fixed (`object_jitter=0`), with no random task seed. Runtime is 20 simulated seconds maximum; wall-clock GPU runtime is unmeasured.

## Scoring and thresholds

`SCORE_OVER = "end"`. Return the highest satisfied tier, without summing. `score()` and `distance()` are read-only; `success()` updates the step history once per control frame. The framework calls success before score and distance, and requires its true return for 1 s to award 1.0.

| Tier | Progress |
| --- | --- |
| 0 | Not initialized in the nominal layout, still in the gap, or any latched violation |
| 0.25 | The whole upright can is outside the finite gap prism, including a vertical lift wholly above its top |
| 0.5 | Also supported on the table; not necessarily at the mark or still |
| 0.75 | Also in the marked goal region and still |
| 1.0 | The 0.75 conditions continuously hold for the framework's 1 s success hold |

Tipping anywhere, including inside the gap, latches zero: a tipped can on the gap floor defeats the intended direct upright retrieval. Leaving the table, a malformed state, an invalid initial layout, changed fixed layout, or an unexplained one-step can jump over 100 mm also latches zero. Violations are checked in success and current validity is rechecked at the final score. Slowly restoring the can cannot erase a latch. An episode reset is recognized only when a >100 mm one-step jump also returns every task object to its nominal layout; an ordinary slow return is not a reset. A fresh imported module starts an independent episode state.

Every threshold here is **开发估计 (development estimate)** unless it is a directly measured dimension. No physical trial calibration has been performed.

| Threshold | Value | Basis |
| --- | --- | --- |
| Grasp width / height design limits | <85 / ≥15 mm | Given YAM grasp envelope; real 30 mm mesh band measured |
| Gap / wall offset | transverse hand width +4 mm / can height +15 mm | Minimum scanned nominal side-margin tier with a verified direct vertical path |
| Model clearance requirement | ≥3 mm | Requested guard against hand/panel scraping; selected margin 4.493 mm |
| IK limits | <1 mm / <1° | Requested scan acceptance; actual residuals much smaller |
| Upright | tilt <5° | Can base stability; yaw is ignored, including 180° yaw |
| Table support | bottom −2 to +3 mm; vertical history spread ≤2 mm; before the window fills, vertical speed ≤30 mm/s | Small contact/mesh tolerance and vertical position stability; avoids frozen velocity readings after sleep |
| Landing centre | within 15 mm on x and y | Keeps the roughly 66 mm can fully inside the 96 mm inner marking span |
| Stillness | all position pairs within 2 mm over latest 10 frames | 0.5 s sample window at 20 Hz; avoids stale sleeping-body velocity readings |
| Incomplete stillness window | speed <10 mm/s, spin <0.2 rad/s | Explicit fallback passed to `resting(max_speed=..., max_spin=...)` |
| Success hold | 1.0 s | Framework's consecutive-success hold, separate from score |
| Nominal start tolerance | 5 mm from spawn centre | Allows the 1 mm gravity settle without accepting a different layout |
| Fixed layout tolerance | centre displacement ≤1 mm, orientation ≤0.5° | Fixed structures must remain in their reproducible layout |
| Reset/discontinuity | one-step jump >100 mm | Much larger than adjacent 20 Hz demo targets; reset additionally needs nominal layout |
| Off-table | any AABB beyond table edges, or bottom >10 mm below table | Detects a launched/fallen object; world bounds are the station table's |
| Release gap / final hand clearance | 4 mm / ≥30 mm | Close placement and the required observable open-hand finish |

`ObjectState.low/high` are treated as world AABBs for conservative extent checks, not as unrotated geometry. Uprightness is computed from the can's local vertical axis using `quat_xyzw`. Collision verification uses real robot hulls, not object AABBs. The frame history contains only copies of the can position. Read-only final-frame checks also compare the supplied current position against the stored window and previous position, so a fresh displacement or unexplained jump cannot borrow stale stillness.

## Validation and reverse tests

Final CPU environment: existing `yam_env`, macOS 26.6 arm64, CPython 3.12.16, NumPy 2.5.4, pytest 9.2.0, real yam_arena commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. Arena library source: release/0.3.1 at c7edf3a9d5d8276565208f72422f8f8a0806d6cf. The completed scan and final test report use this environment. Preliminary runs in another interpreter were stopped when the approved interpreter was specified and are not used as final scan evidence.

Tests: 50 passed, 0 failed, 0 skipped. They cover every constructor signature through the real framework, all score tiers and hold semantics, every latch and later restoration, direct terminal jumps, tipping/launching, slow restoration, initial-layout failures, both sides of thresholds, yaw 180°, score/distance purity, reset detection, spawn overlap, debug cadence, complete-frame IK/limits, clearance of both arms against **all** task structures, release height, actual open-gripper final waypoint and fingertip clearance, run limit and asset metadata. The holding-without-release case explicitly demonstrates the documented simulation limit rather than falsely claiming a rejection.

The following wrong implementations were actually substituted into the task source during tests, with no framework monkeypatch or private robot access:

| Deliberate error | Observed expected test failure |
| --- | --- |
| Forget the tilt latch after restoration | 2 failures |
| Sum partial-credit tiers | 1 failure |
| Treat 180° yaw as tilt | 1 failure |
| Mutate position history inside score | 3 failures after strengthening the test's nonidentical history |
| Print debug every step | 1 failure |
| Clear latches on a slow nominal return | 1 failure |
| Omit opening in the release waypoint | 1 failure |
| Infer stillness only from frozen zero velocity | 1 failure |
| Move demo hover 400 mm higher | 1 failure from independently recomputed per-frame IK |
| Shrink the gap to 20 mm | 1 failure from full robot/structure geometry checks |

`validation_report.json` records final counts, reverse tests, elapsed time and limitations. The external scripts are `make_can_gap_info.py`, `scan_can_gap.py` (conservative screen), `scan_can_gap_exact.py` (final complete scan) and `refine_can_gap.py`, in the designated reference-tools directory, outside this repository. Their source hashes identify the measurement workflow. The mesh, density and centre-of-mass arithmetic were rerun with the approved interpreter. No custom asset builder enters the submission.

## Known limits and review risks

- **Release observability:** simulation scoring cannot distinguish a can still clamped motionless against the table from a released can with identical ObjectState fields. The real-trial requirement is: **the gripper is open and at least 30 mm from the can at the end, judged from the trial video**. This supplements simulation scoring. The demo test asserts opening and a final fingertip clearance over 30 mm, but real release/contact and the actual final video frame remain GPU checks.
- GPU PhysX execution, friction, grip force, slip, tracking lag, stable standing rest pose, contact compliance, camera visibility, real finish time and achieved score are unverified. CPU target clearance is not a guarantee of tracking clearance. No Dream-1 or other model run has been made; `scores` remains `pending` and `media` remains `{}`.
- The stock framework uses static/dynamic grasp friction 1.2/1.0 and a 5 mm can contact offset with zero rest offset. Other colliders keep their framework offsets. Contacts may be generated before geometric surfaces touch; the scan does not model these envelopes or resulting forces. The 4.493 mm geometric clearance therefore requires a GPU contact check, especially in this narrow gap. During geometric checks, finger closure is blocked at a conservative nominal 66 mm aperture; a successful grasp is assumed for carrying the can, rather than simulated. No contact settings were changed through private interfaces.
- The short walls constrain the fingers but do not force the wrist/forearm fully into the gap. Both ends are open, so an end approach could bypass the demonstrated vertical route. Reviewers could judge this insufficiently distinct from ordinary constrained pick-and-place, or reject the construction for allowing an easier strategy. The scan establishes the requested path's feasibility, not necessity or uniqueness.
- Can mass uses the official USD's nominal mass scaled at constant estimated filled density; actual contents, mesh watertightness, label thickness, inertia and physical centre of mass were not independently measured. The native USD COM is an automatic sentinel. The uniform-mesh estimate is stored in JSON, not claimed as a true soup distribution. Library collision decomposition and automatic mass properties were not GPU-inspected.
- The official external USD was obtained from the public 6.2 staging mirror at the registry path; a previously tried 6.1 URL returned 404. Its SHA-256 is in `assets/can_info.json`. A GPU with a different asset root must verify the same source bytes/dimensions. The referenced texture remains external and was not copied or metadata-modified. The USD reader emitted an arm64 cache-line assumption warning while still reading all vertices.
- The sweep is a 20 mm spatial grid with one full direct-retrieval path family, not an exhaustive continuous motion search. SAT checking is discrete at 20 Hz; continuous swept-volume clearance, self-collision and arm-to-arm clearance were not certified. The selected path's 10 mm higher copy passed IK as a reach reserve test.
- Final source imports no Isaac Sim, reads no robot state and modifies no framework code. Episode state relies on the documented success-before-score callback order at 20 Hz. A reset without a >100 mm observed jump cannot be distinguished from slow physical restoration with this API and deliberately cannot clear a latch.
- The live task board required sign-in, so pending tasks newer than the saved snapshot were not checked. No duplicate-clearance claim against the live board is made. The local environment lacks the Inspect Robots CLI, so GPU smoke/full-demo commands were not executed. Submission feasibility, media and review acceptance remain pending.
- Scan implementation issues corrected during development: a NumPy boolean initially prevented JSON summary serialization; conversion to builtin bool fixed it. An earlier score-purity mutation escaped an all-identical history fixture; the strengthened test catches it. A naive all-vertex-pair SAT edge set was too slow; equivalent convex-hull edges made the independent full check practical. A support-window fixture initially expected landing support immediately after descent; it was corrected to wait for the required window. A later audit added read-only current-position and discontinuity checks at the final frame. The staged whitespace check also rejected CSV CRLF line endings; they were normalized to LF with every parsed cell checked unchanged. These issues did not alter the final acceptance thresholds.
- Stopping the superseded interpreter's process pool needed explicit termination of that scan's process tree. An optional process-inspection package was absent, so standard process listing was used instead; no environment or package was installed. The source-only local environment has no Inspect Robots entry point. Historical preflight data remain labeled as such.

## Comparison and third-party assets

Based on pr-crawl snapshot-20261119T072321Z; all three comparators were accepted and merged there:

| PR | Difference |
| --- | --- |
| [#182: Fish a charger out from behind a box by its cord](https://github.com/robocurve/benchmark-tasks/pull/182) | Uses an exposed flexible cord to recover an obscured charger while limiting box movement. This task directly grasps the can with the robot hand inside the gap, without a cord. |
| [#1957: Replace the cabin air filter behind a car's glove box](https://github.com/robocurve/benchmark-tasks/pull/1957) | Releases a glove-box access mechanism and exchanges a filter. This task has fixed panels and directly retrieves an upright can with a few millimetres of hand clearance, with no access mechanism or replacement step. |
| [#2376: Pick up litter with a reacher-grabber and drop it in the bin](https://github.com/robocurve/benchmark-tasks/pull/2376) | Uses an actuated long tool and prohibits direct gripper/litter contact. Here direct contact is required and no tool is supplied. |

The #1957 review initially rejected side access that bypassed its intended mechanism. The open-end shortcut risk here is therefore material, even though the demonstrated path passes the requested scan.

Third-party assets: YCB tomato soup can, **CC BY 4.0**; B. Calli et al., “The YCB Object and Model Set: Towards Common Benchmarks for Manipulation Research,” ICAR 2015; [YCB project](https://www.ycbbenchmarks.com/). It remains an external Arena asset; only measured JSON is distributed here. The robot geometry used for verification is the framework's I2RT YAM model under its bundled **MIT** license; no robot mesh is copied. Panels and marks are original analytic fixed boxes. `assets/` contains only JSON, with no images or texture bytes; its identity-path/metadata scan is covered by tests. No CC0 model is used.
