# Fetch a soup can from the gap beside a cabinet — feasibility stop

**Status: STOP. Gate 3 failed before task implementation. This folder is an engineering report, not a runnable benchmark or submission.** No task.py, task.json, or test_task.py was created. The mandatory task tests were not replaced by diagnostic tests. No GPU simulation, Dream-1 run, media upload, or pull request was made.

The intended instruction is: “Please get the soup can out from between the cabinet side and the wall, and leave it standing in the marked area in front of you. Keep the can upright and avoid scraping either panel.” The scene uses two wooden/plaster panel surrogates, not an appliance. Its category would be Societal relatability.

The intended distinction is direct retrieval with the robot's own fingers and hand inside the restricted space, without a cord or tool. The measurements show that the hand can descend into one candidate gap; they do **not** establish a feasible complete retrieval, and the two open panel ends do not establish that a vertical approach is necessary.

## Three gates

| Gate | Real evidence | Result |
| --- | --- | --- |
| Graspable part narrower than 85 mm and at least 15 mm high | Official tomato_soup_can_ycb_robolab USD: 8,404 vertices; native bounds 67.854 × 67.750 × 102.037 mm. Asset(extent=0.066) would uniformly scale this to 66.000 × 65.899 × 99.249 mm. A 30 mm band on the actual can mesh spans at most 64.318 × 63.773 mm horizontally. | Pass |
| Allowed demo actions | Proposed Waypoint sequence uses downward orientation, translation, yaw, and finger opening/closing only. No tool, tipping, scooping, or rotation about a horizontal axis. | Pass for the proposed motion vocabulary; no successful demo claim |
| Every path frame reachable and robot clear of panels | Real yam_arena.demo.ArmPlan.action7(t), 20 Hz, 307 frames, 15.3 s. Every robot collision hull was checked against both panels, for both arms including the inactive right arm and both pairs of fingers. | **Fail** |

The failed path is one concrete candidate, not a proof that every possible layout or path is impossible. No layout relaxation or task implementation followed the failure, in accordance with the stop gate.

## Measured hand and candidate geometry

Values come from yam_arm.urdf, its referenced collision STL hulls, the actual can mesh, and station.BASES. All measurements and source hashes are in preflight_measurements.json. Dimensions are collision dimensions unless stated otherwise.

| Measurement | Result |
| --- | --- |
| Distal pad thickness along closing axis, left / right | 6.333 / 6.339 mm; this is the distal band used by the framework's own constants test, not a constant thickness of the entire finger |
| Actual pad aperture at joint7 = joint8 = −33 mm | 66.063 mm |
| Pad pair outer width at that aperture | 78.734 mm |
| Entire moving finger pair outer width along closing axis | 113.547 mm, including the wider proximal parts |
| Entire moving finger pair transverse width / vertical height | 67.999 / 93.843 mm |
| Entire moving finger pair outer width fully open | 141.447 mm; pad aperture 93.963 mm |
| Gripper housing closing-axis / transverse / vertical span | 102.000 / 65.000 / 73.999 mm |
| Candidate can centre | x = 0.200 m, y = +0.200 m, on the left arm's own side |
| Two fixed panels | Each 240 mm long along x, 18 mm thick, 195 mm high; x spans 0.080–0.320 m |
| Gap and orientation | 74 mm between the inner faces; yaw = π/2 puts finger closing along the gap's x length |
| Nominal transverse hand clearance | (74 − 67.999)/2 = 3.000 mm per side; raw can clearance about 4 mm per side |
| Wall height basis | 195 mm > 93.843 mm finger height + 99.249 mm scaled can height = 193.092 mm |
| Candidate TCP grasp / hover | 65 / 210 mm above the station tabletop (table z = 0.7475 m) |

At the grasp pose, clipping convex hulls against the full table-to-panel-top slab gives the following bounds. Clipping includes edge/plane intersections, not just vertices already in the slab. Full world spans are pose dependent and are not cylinder diameters.

| Part | Full x × y × z span (mm) | Lowest point above table (mm) | Portion inside the 0–195 mm panel-height slab |
| --- | --- | --- | --- |
| Gripper housing | 102.001 × 65.000 × 73.999 | 135.524 | 102.001 × 65.000 × 59.476 mm |
| Wrist link5 | 73.187 × 64.113 × 69.102 | 199.022 | None; 4.022 mm above panel top at this pose |
| Wrist link4 | 133.594 × 119.318 × 119.797 | 206.299 | None; 11.299 mm above panel top at this pose |
| Forearm link3 | 304.124 × 209.563 × 169.419 | 280.137 | None; 85.137 mm above panel top at this pose |

This explains why inspecting the can alone is insufficient: at the proposed near-end exit, TCP x = 35 mm leaves the can in front of the panel ends at x = 80 mm, while the wider hand still overlaps their longitudinal range. The failed IK also changes the actual hand orientation and increases the collision risk.

## Full-path failure

| Segment | Worst IK position (mm) | Worst IK angle (degrees) | Minimum robot/panel SAT margin (mm) |
| --- | ---: | ---: | ---: |
| Initial hover | 0.075 | 0.011 | +6.104 |
| Vertical descent | 0.082 | 0.009 | +2.988 |
| Close | 0.023 | 0.003 | +2.998 |
| Small vertical lift | 0.075 | 0.008 | +2.989 |
| Move towards near open end | 12.822 | 32.769 | −11.909 |
| Turn outside panels | 12.877 | 48.217 | **−21.746** |
| Move over target | 9.048 | 48.241 | +4.258 |
| Lower | 3.030 | 20.551 | +12.758 |
| Release command | 3.030 | 20.551 | +8.046 |
| Retreat | 3.030 | 20.551 | +7.861 |

Overall worst errors are **12.877 mm / 48.241°**. At least one joint reaches its limit (minimum margin 0°). The minimum collision margin occurs at t = 9.05 s, left tip_right against the panel on the lower-y side. The 21.746 mm value is penetration in the convex geometry model, not a measured PhysX contact depth. Later release and retreat entries describe failed commanded waypoints, not demonstrated physical actions.

The collision test uses complete convex-hull/box separating axes: both shapes' face normals and every hull-edge × box-edge axis. Positive values are certified separating-axis margins, which can be lower bounds on Euclidean distance; negative values mean the modeled shapes intersect. It does not rely on ObjectState.low/high as actual geometry or on vertex inclusion alone.

Before the final candidate, short four-second hover/descent probes were evaluated at (0.16, 0.20) and (0.20, 0.20), with yaw 0 and π/2. The (0.16, 0.20), π/2 probe reached a limit and reported 0.668 mm / 3.012°. The other three probes reported about 0.08 mm / 0.014° or less. These endpoint probes did not establish a full path or collision clearance and were superseded by the complete candidate's failure.

## Independent scorer blocker

The inspected framework calls object_states(env, task.names), then success(states), distance(states), and score(states). With only a can and two panels declared, these hooks receive exactly those three objects. ObjectState provides only pos, quat_xyzw, lin_vel, ang_vel, low, and high. It exposes no finger opening, robot link pose, contact force, grasp state, or release event.

A can standing in the target while still gently clamped and the same can after release can have identical values for all six allowed fields throughout the entire final second. Therefore a scorer restricted to these inputs cannot reliably reject “holding without release” while accepting the released case. Stillness, tabletop height, a 1 s hold, or a timed assumption about the demo does not supply this missing observation. The diagnostic test constructs the two identical real ObjectState records to document the information limit; it is not a physical simulation test. Adding scene objects as pretend robot markers or reading private simulator/robot state would change the authorized contract and was not done.

This is a separate blocker even if a future path passes the geometric gate. No partial-credit or success functions were authored. The proposed 0.25/0.5/0.75/1.0 tiers, tilt latch, reset detection, terminal recheck, stillness window, and TASK_DEBUG cadence remain unimplemented and unverified.

## Development thresholds

All candidate values and numerical tolerances below are **开发估计 (development estimates)**, not validated physical acceptance thresholds.

| Quantity | Candidate threshold | Physical or implementation basis |
| --- | --- | --- |
| Grasp width / band height | <85 mm / ≥15 mm | User's gripper design envelope; actual mesh measurement above |
| Gap | 74 mm | 67.999 mm transverse hand envelope plus 3 mm nominal clearance per side |
| Panel height | 195 mm | Exceeds measured scaled can + finger height by 1.908 mm |
| Minimum modeled robot/panel margin | ≥2 mm | Leaves about 1 mm of the nominal 3 mm for model/path variation; no physical validation |
| IK position / orientation residual | <0.5 mm / <0.1° | Tolerances used by the real framework's example demo test; sensible compared with 3 mm gap clearance |
| Sampling | 20 Hz, every frame calls action7 | Required control rate; the ArmPlan error counters update only on action7 |
| Release command height | 69 mm TCP above table | 65 mm grasp height + 4 mm nominal bottom-to-table release gap; failed IK means actual release is not verified |
| Stillness and success hold | Intended 10 frames within 2 mm; intended success hold 1 s | Requested scoring design only; no scorer exists |

The 10 mm reserve below each position's reach ceiling was not certified for all path points. Short endpoint reach data cannot substitute for this certification. The candidate is already rejected by the much larger IK and collision failures.

## Validation and reproducibility

Environment: local Apple Silicon macOS 26.6, arm64, CPython 3.12.14, NumPy 2.3.5, real yam_arena source commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. USD was read with usd-core 26.11; tests used pytest 9.2.0. Isaac Sim, GPU physics, and Dream-1 were not used. Source hashes, exact dimensions, segments, and audit counts are in preflight_measurements.json.

The independent diagnostic audit produced **15 passed, 0 failed, 0 skipped**. Reverse checks actually substituted three deliberately incorrect diagnostic implementations and reran the same tests:

| Incorrect implementation | Expected failing tests observed |
| --- | --- |
| Check only whether hull vertices enter a box; miss crossing edges | 1 failed, 14 passed; crossed slender solids were incorrectly reported as separated |
| Ignore the IK residuals when judging a path | 2 failed, 13 passed; excessive position and angle errors were independently caught |
| Use only the can's width when judging the gap | 2 failed, 13 passed; the hand envelope and wrong-yaw rejection were both lost |

These reverse tests validate the measurement/diagnostic logic, not nonexistent task scoring. **Task tests: 0 executed, no pytest skips.** All required task-level checks remain unrun: constructor binding, each scoring tier, irreversible violation and later restoration, direct jump to terminal state, knockdown/launch, slow restoration, holding without release, wrong starting layout, score-boundary checks, yaw-180 tilt check, score/distance purity, reset, spawn overlap, TASK_DEBUG cadence, and a complete feasible demo's IK, limits, release, hand clearance, and run limit. The diagnostic checks both arms against the two panels; it does not certify self-collision, arm-to-arm clearance, robot/table clearance, camera clearance, or contact dynamics.

The measurement script and its diagnostic tests are stored outside the repository in the designated reference-tools directory, named fetch_can_preflight.py and test_fetch_can_preflight.py. Set REFS_TOOLS to that directory, YAM_SOURCE to the actual yam_arena checkout, and CAN_USD to the downloaded official USD. Use a Python 3.12 environment containing NumPy, usd-core and pytest:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$YAM_SOURCE" \
  python "$REFS_TOOLS/fetch_can_preflight.py" --asset "$CAN_USD" \
  --output tasks/fetch-can-from-gap-beside-cabinet/preflight-rerun.json

CAN_PREFLIGHT_DATA=tasks/fetch-can-from-gap-beside-cabinet/preflight_measurements.json \
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$REFS_TOOLS:$YAM_SOURCE" \
  python -m pytest -q -p no:cacheprovider "$REFS_TOOLS/test_fetch_can_preflight.py"
```

The script is a diagnostic experiment, not a task file. Its rerun output contains the core measurements; the committed JSON also records source provenance, opening measurements, and audit outcomes collected separately. Keep any rerun output within this task folder inside the worktree.

## GPU handoff: blocked

There is no runnable task file to send to a GPU. The following are **future command templates only**. Their guard intentionally exits on this report-only branch. A feasible reviewed path, an authorized way to observe release, task.py, and all required CPU task tests are prerequisites. Do not proceed from smoke to full demo after any failed check.

```bash
TASK_FILE=tasks/fetch-can-from-gap-beside-cabinet/task.py
test -f "$TASK_FILE" || { printf '%s\n' 'Blocked: feasibility report only; task.py does not exist.'; exit 1; }

TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$TASK_FILE" -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 200 \
  --instruction 'Please retrieve the soup can and leave it standing in the marked area without scraping the panels.' \
  --store-frames --log-dir /tmp/can-gap-smoke

# Only after all smoke checks pass; 600 is a provisional diagnostic budget, not a validated task run_limit.
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$TASK_FILE" -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \
  --instruction 'Please retrieve the soup can and leave it standing in the marked area without scraping the panels.' \
  --store-frames --log-dir /tmp/can-gap-demo
```

Future smoke instrumentation must show: ≤0.5 mm / ≤0.1° IK error at every 20 Hz action, no joint outside its URDF limit, at least 2 mm modeled hand/wrist/forearm clearance from both panels, 1 mm initial can-bottom gap, about 99.249 mm can height at the declared 66 mm extent, 4 mm actual release gap, and task_progress only at steps 1, 20, 40, …, 200. Stillness must use the position history and success must hold for 1 s. Release needs an actual observable signal, not a zero velocity inference. The failed candidate's t=9.05 s collision occurs within a 200-step smoke and must reject that candidate. These quantities require instrumentation that has not been implemented.

## Closest accepted tasks and review risk

Comparison source: pr-crawl snapshot-20261119T072321Z, including pull_requests.json, accepted_abilities.json, and pr_analysis.json. All three were marked accepted and merged in that snapshot.

| Existing PR | Difference sought here |
| --- | --- |
| [#182, Fish a charger out from behind a box by its cord](https://github.com/robocurve/benchmark-tasks/pull/182) | Uses an exposed cord to retrieve an obscured charger and constrains box movement. This proposal instead puts the robot's own hand into the gap to directly grasp an upright rigid can; no cord is available. |
| [#1957, Replace the cabin air filter behind a car's glove box](https://github.com/robocurve/benchmark-tasks/pull/1957) | Requires accessing a filter through a released glove-box assembly and seating its replacement. This proposal concerns tight hand-envelope clearance during direct vertical retrieval between two panels, with no access mechanism or replacement part. |
| [#2376, Pick up litter with a reacher-grabber and drop it in the bin](https://github.com/robocurve/benchmark-tasks/pull/2376) | Requires an actuated long tool and prohibits direct robot-gripper contact with litter. This proposal requires direct contact and provides no tool. |

Likely rejection reasons if submitted now: infeasible demonstrated path; no full-score physics demonstration or media; release is unobservable with the authorized scoring inputs; open ends may admit a horizontal shortcut that defeats the claimed vertical-access requirement; and the remaining distinction from ordinary constrained pick-and-place may be judged insufficiently important. A close precedent is #1957's initial rejection: its side access bypassed the intended access mechanism and its scorer did not require that mechanism. That PR was accepted only after revisions; its review history directly warns against claiming a geometry-enforced action without checking bypasses.

The live task board returned its sign-in page, so new pending/accepted tasks after the snapshot were not checked. No complete live duplicate-clearance claim is made.

## Assumptions, failures, and assets

- The official registry USD was obtained from the configured public 6.2 staging asset mirror. An earlier attempted 6.1 URL returned 404. The GPU's actual asset-root version is unknown; matching the committed SHA-256 is required before transferring these dimensions to another environment.
- The source-only real yam_arena was imported through PYTHONPATH. The default system Python 3.9 had no NumPy, pytest, USD reader, or installed yam_arena. A bundled Python 3.12 runtime and temporary usd-core/pytest packages resolved CPU diagnostics, not Isaac Sim availability.
- Initial sandbox Git/network writes were restricted; authorized fetch and worktree creation succeeded with the permitted escalation mechanism. No original working-tree edits were made. Broad initial discovery hit absent migrated directories and protected macOS cache paths; these were not evidence about host installations. A process-status probe was sandbox-denied and was not needed for the final measurements.
- The USD reader emitted ARCH_CACHE_LINE_SIZE != Arch_ObtainCacheLineSize() on this arm64 host. It still read the complete mesh; the warning is preserved here as an unvalidated runtime caveat.
- Finger closure was modeled as blocked at the conservative 66 mm nominal aperture; no can/finger forces, friction, slip, compliance, robot tracking lag, restitution, can tipping, or actual release was simulated. The scoring observability counterexample does not claim a simulated successful held pose.
- Only the native standing mesh bounds and candidate scaling were measured. A stable resting pose under gravity, mesh inertial tensor/true material density, collision decomposition of the can, physical centre of mass, and contact stability were not verified. The official USD centreOfMass is the automatic (-inf, -inf, -inf) sentinel rather than a measured finite centre. No custom can model was created and no substitute primitive was used.
- The external USD references textures/obj_000004.png; that texture was not fetched or included. No third-party asset bytes, images, or assets/ directory are committed. Therefore the requested asset-file path/EXIF/XMP/caBX metadata scan has an empty committed scope; it was not represented as a scan of texture pixels or of the undownloaded texture.
- No media exists, all task scores remain pending in concept, and no reproducibility/feasibility claim suitable for submission is made. A marked target, exact materials/friction, irreversible-tilt policy, score thresholds, reset behavior, and run_limit are not finalized.

Asset provenance for the measured external model: YCB tomato soup can, CC BY 4.0, cited as B. Calli et al., “The YCB Object and Model Set: Towards Common Benchmarks for Manipulation Research,” ICAR 2015; [YCB project](https://www.ycbbenchmarks.com/). YAM robot meshes are the framework's I2RT assets under the bundled MIT license; those bytes also remain external. Panel shapes were only analytic boxes in the diagnostic. No third-party model is distributed by this report.

Branch: fetch-can-from-gap-beside-cabinet. Base: origin/main at 573e3868db43cb69b9cb0b446ddf8c7de7045518. The final commit hash and exact diff stat are supplied in the delivery report; a report cannot embed its own final commit hash without changing that hash.
