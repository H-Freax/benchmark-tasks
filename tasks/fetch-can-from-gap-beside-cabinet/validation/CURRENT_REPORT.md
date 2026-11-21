# Current deep-drawer implementation report

The current scene uses a native-scale soup can and full drawer geometry. All older root-level CSV/JSON checks, `validation/gap_scan.csv`, `yaw_variants.csv`, `gripper_dims.json`, `scan_summary.json`, `audit.json`, `REPORT.md`, `environment.json` and their scanner/audit sources are historical short-wall/scaled-can work. They are retained rather than deleted and do not certify the current task. Current records have the `drawer_` prefix.

## Supplemental scan

Existing macOS arm64 `yam_env`: CPython 3.12.16, NumPy 2.5.4, pytest 9.2.0, real yam_arena `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. Python-FCL 0.7.0.11 was installed with `--no-deps` into a temporary target, leaving yam_env and its NumPy unchanged. The scan did not import Isaac Sim, run GPU physics or simulate contact. Arena source is release/0.3.1 `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; its pyproject version string alone had earlier been mistaken for evidence against the release and that claim was corrected.

Fixed: x=0.160, y=+0.100 m, theta=60°, h=35 mm; native can 102.037005 mm high; dividers 127 mm high and 15.875 mm thick; 460.375 mm long. Thus delta=24.962995 mm and TCP descent depth=92 mm. All four c values use the same successful-pose section of 73.052045 mm for both yaws.

| Corrected layout | Combinations | Pass | Failure |
| --- | ---: | ---: | --- |
| Drawer centred on row; longitudinal offsets 0,+80,−80 mm | 24 | 0 | Near frame/floor overlap robot base; upper arm intersects near frame |
| Drawer transverse offset −80 mm, longitudinal offset 0 | 8 | 4 | All four preferred yaws pass; all four alternate yaws fail |

Each row checks 98 extraction frames at 20 Hz by actually calling ArmPlan.action7, plus five independent waypoint+10 mm solves. All robot URDF collision components on both arms are checked against two complete dividers, floor and all four outer frame boards using FCL convex distance. AABB distances are lower-bound pruning only.

| c (mm) | w (mm) | yaw (°) | Minimum distance (mm) | IK position (mm) / angle (°) | Joint margin, including reserve (°) | Result |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | 77.052045 | −82.028396 | 4.499704 | 0.083102 / 0.010637 | 5.691942 | Pass |
| 6 | 79.052045 | −82.028396 | 5.499704 | 0.083102 / 0.010637 | 5.691942 | Pass |
| 8 | 81.052045 | −82.028396 | 6.499704 | 0.083102 / 0.010637 | 5.691942 | Pass |
| 10 | 83.052045 | −82.028396 | 7.499704 | 0.083102 / 0.010637 | 5.691942 | Pass |
| 4,6,8,10 | 77.052045–83.052045 | +97.971604 | 0 | 4.685017 / 28.654842 | 0, saturated | Fail |

Preferred-yaw +10 mm checks: worst 0.069267 mm / 0.011912°. The selected c=4 gives position-error margin 0.416898 mm and meets the finger-floor condition: tip is about 22.158 mm above the floor. Grasp width is below 85 mm and the measured can body contains a measured 29.939 mm grasp band (65.966 × 64.866 mm transverse mesh bounds; 1,466 vertices in the 20–50 mm body-height slice).

Failure reason counts for the corrected 32 rows overlap: **IK 16; joint-limit saturation 16; actual joint-bound violation 0; structure clearance 28**. Each failing row appears in every applicable category, so these are not a partition. Link/structure counts below count rows where any component of that link violates 3 mm:

| Link / structure (scan labels) | Rows |
| --- | ---: |
| left base / floor | 24 |
| left base / near side frame | 24 |
| left link2 / near side frame | 24 |
| left base / near end frame | 8 |
| left gripper / each divider | 16 per divider |
| left tip_left / positive divider | 16 |
| left tip_right / negative divider | 16 |

The first preliminary 24-row attempt derived width separately from each yaw's solved pose. The failed yaw supplied a distorted section estimate. That reporting/design error was corrected by using a shared 73.052045 mm section for both yaws; all 24 centred rows were rerun. `drawer_initial_width_attempt.json` retains the preliminary attempt and its 24 failures, separate from the corrected grid. Board scan labels with `_-1` correspond to task identifiers ending `_neg`; `floor` corresponds to `drawer_floor`. Names do not alter geometry.

## Full demonstration and corrections

The first READY-to-row approach left only **1.098533 mm** between a fingertip and the near outer frame. This failed the 3 mm criterion. Raising the outside approach and row hover to the full carry height fixed the issue. A later spawn-overlap test detected a landing outline/frame overlap at the initially chosen goal (0.240,+0.260) m. Moving only the landing goal to **(0.240,+0.280) m** fixed it, and the entire path was rerun.

Current complete demonstration: **289 frames / 14.4 s**; worst IK **0.079463 mm / 0.011867°**; normal-path joint margin **7.453880°**; minimum joint margin at +10 mm reserve waypoints **5.478628°**. Minimum exact robot-to-drawer distance is **4.495518 mm**, left tip_left at 6.15 s during vertical extraction. The automated geometry test also includes the four marking strips and the inactive arm. It does not certify robot self-collision, inter-arm distance, station camera fixtures or continuous swept volumes.

The carried can bottom is 7 mm above the 150 mm outer frame. Every lowering follows a 0.4 s stationary alignment waypoint. Release TCP height is `TABLE_Z + 0.0015 + (pick_tcp_z - TABLE_Z)`, giving a nominal 1.5 mm drop. All framework object/Waypoint numeric configuration values are builtin floats.

The task reads gripper states lazily via public SimulationContext and articulation-view methods already seen in source/verified read-only recorder code. No framework patch or private attribute is used. Release scoring requires both joint openings and the entire gripper/finger collision-envelope clearance; unavailable readings fail closed. Synthetic CPU observations test the scorer, not this task's GPU integration.

Only off-countertop history is physically irreversible. Tilt, incorrect landing, a temporarily bad observation or altered fixed layout can recover when conditions are restored; invalid initial setup cannot arm a new episode without a recognized reset. A continuous upward crossing over the taller outer frame witnesses extraction, preventing a direct terminal jump from borrowing progress.

## Test outcomes and reverse checks

Final CPU baseline: **50 passed, 0 failed, 0 skipped** in 2.483 s. All **16/16** executed deliberate mutants produced the expected assertion failure. See `drawer_validation.json` for the definitive counts, source hash and every mutant result. Its top-level `status` is derived from JUnit assertions and the expected rejection of all deliberate errors, not a CLI return code. GPU run status and score remain unmeasured.

Actual mutations exercise: summed tiers; yaw treated as tilt; score mutating history; slow restoration clearing a latch; forgotten off-table latch; permanent tilt failure; missing-state release credit; ignored hand clearance; velocity-only stillness; absent extraction witness; no release opening; excessive release drop; low approach hitting the frame; absent alignment pause; missing per-step critical debug; NumPy scalars passed to framework constructors. Every mutation is task-owned in-memory source, not a framework monkeypatch.

Development failures and issues, including test tooling:

- The required general rules file was initially absent; implementation waited until the user supplied it, then it was read in full.
- The first full-scene layouts failed as described above; widening the slot did not cure the external-frame/base overlap.
- The alternate yaw failed both pose tolerances and saturated a joint, with actual finger/palm/divider intersections.
- An initial full-demo approach failed clearance, then the landing-marker overlap failed spawn checks; both were corrected and rechecked.
- Real public task loading rejected minus signs in board names and reserved `floor`; identifiers were corrected, without changing geometry.
- One test reused a mutated NaN-containing fixture rather than a clean copy; the fixture was corrected. The first suite had 3 failures/38 passes; the next had 1 failure/41 passes; the third baseline had 42 passes before additional boundary coverage.
- Rerunning the metadata preparer after replacing the old schema exposed a missing `source_mass_kg` key; the preparer now accepts the native mass schema and preserves 450 g.
- Initial mutation-result parsing expected a JUnit failure `type` attribute that pytest omitted, then an assertion message without the literal word AssertionError was misclassified. The parser was corrected to inspect the full saved assertion text. These were reporting failures, not escapes by an incorrect implementation. Raw JUnit files confirmed expected assertion failures.
- OpenUSD printed its arm64 cache-line assumption warning while reading meshes successfully. FCL was missing initially and was added only to a temporary no-deps target.
- Some source-discovery paths (`yam_arena/cli.py`, `run.py`, `task_runtime.py`, a local inspect-robots checkout) were absent. Interfaces used were subsequently located in existing public source and the verified recorder. The local environment still has neither Isaac Lab runtime nor Inspect Robots CLI.
- Historical experiments had separate issues (JSON NumPy-bool serialization, an overly uniform score-purity fixture, costly SAT edges, CSV line endings and the earlier interpreter choice). Those historical records remain labelled; none is substituted for current drawer checks.

## Outstanding verification and risk

- No Isaac Sim 6.1 GPU run, contact grasp, friction, force, slip, tracking clearance, camera framing, release/drop, final native mass properties, current public gripper integration, externally awarded success hold or achieved score is validated. `task.json` correctly remains pending.
- The floor shares its support plane and overlaps a fixed portion of the station table. This lets the public Asset API place the native can at the correct drawer bottom; duplicate support contacts need GPU inspection. A raised movable drawer is not claimed.
- The official external USD was read from a 6.2 staging mirror; tried 6.1 mirror URLs returned 404. Its SHA-256 and native size must match on the target 6.1 runtime. External texture metadata was neither copied nor audited. Local assets contain JSON only, checked for identity paths and metadata tokens.
- Robot target clearance is about 4.5 mm. Contact offsets may generate contacts before mesh touch; the can's source contact offset is 5 mm, with other framework defaults unchanged. Native collision decomposition and physical margins require dynamic inspection.
- Fixed planar envelopes omit divider springs and end pads and use chosen, explicitly disclosed drawer dimensions. The recessed drawer/table support construction is an approximation. Physical product dimensions have not been independently measured.
- The L-versus-TCP depth classification in the original brief is inconsistent; actual geometry, not that heuristic, determines collision. At 92 mm depth the palm enters while wrist stays above the divider. The task must not claim full forearm insertion.
- Scoring certifies extraction and the final upright/released landing; it does not latch recoverable divider contact or prohibit another physically valid strategy. The scripted path meets the geometric no-touch criterion, but runtime wall-contact history is not a separate zero-score condition.
- A history reset without an observed >100 mm can jump cannot be distinguished from slow restoration and cannot clear an off-countertop latch. The framework callback order and 20 Hz control must match the documented configuration.
- Live task-board sign-in prevented a fresh uniqueness audit; the saved snapshot is the comparison source. No acceptance prediction is claimed.

## Comparison with nearest accepted tasks

Based on snapshot `snapshot-20261119T072321Z/accepted_abilities.json`:

- [#182, Fish a charger out from behind a box by its cord](https://github.com/robocurve/benchmark-tasks/pull/182): relies on an exposed flexible cord and pulling direction. Here the hand itself enters, aligns its yaw to a narrow row, directly grips the can and vertically clears fixed boards.
- [#1957, Replace the cabin air filter behind a car's glove box](https://github.com/robocurve/benchmark-tasks/pull/1957): involves releasing access stops and exchanging a seated filter. Here the boards stay fixed; no mechanism, replacement or insert is manipulated.
- [#2376, Pick up litter with a reacher-grabber and drop it in the bin](https://github.com/robocurve/benchmark-tasks/pull/2376): operates an extended tool and forbids direct hand/litter contact. Here no tool exists and the robot's own fingers must fit.

Possible rejection risks: insufficient distinction from constrained pick-and-place; product-envelope and recessed-floor simplifications; lack of demonstrated GPU success/media; alternative strategies or contact history beyond what the scorer enforces; newer overlapping tasks absent from the snapshot. These comparisons and concerns do not appear in task.json or README.

## GPU handoff

README contains the exact commands, first a **200-step** scripted smoke and then the full **400-step** scripted demo. Neither has been executed. Inspect result JSON `status`, not CLI exit code. Smoke must establish native dimensions/hash, no initial overlap or support instability, armed initial state, readable grippers, 20 Hz timing, no contact or saturation, and ≥3 mm carried-can/outer-frame clearance. Full trial additionally checks ≤2 mm drop, tilt <5°, support −1..+2 mm, landing ±15 mm, both joint gaps ≥85 mm, both hand envelopes ≥30 mm, ten-frame displacement ≤2 mm and a full second of successful hold. No PR, GPU, Dream-1 or upload was performed here.

CSV files normalized to LF before staging, with parsed cells checked identical: gap_scan.csv, yaw_variants.csv.
