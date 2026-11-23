# Lift a can out of a drawer divider row

Take the upright soup can out of a deep kitchen drawer and leave it standing in the marked area beside the drawer. The hand turns to align with the row and directly grips the can inside the narrow slot. No cord, handle or tool is provided.

The supplemental CPU scan and the complete commanded demonstration pass geometric checks. A successful physical grasp and achieved simulation score remain pending GPU validation. `scores` is `pending`; no media is included.

## Scene

The two fixed bamboo boards represent **Lipper International 8897 deep drawer dividers**. The [manufacturer specification](https://www.lipperinternational.com/products/bamboo-deep-drawer-dividers-set-of-two/) gives 0.625 in thickness, 5 in height and adjustable length 18.125–22 in: **15.875 × 127 × 460.375–558.8 mm**. The selected length is 460.375 mm. Fixed planar envelopes omit the spring tensioner and end pads; this task does not operate the dividers or drawer slides.

The drawer dimensions are scenario choices, not product specifications: interior **460.375 × 300 × 150 mm**, four 18 mm plywood frame boards and a 12 mm floor. The floor's top is the station table reference plane. Its fixed solid overlaps the top 12 mm of the built-in table collider, modelling a recessed open drawer next to a countertop without raising the native library asset. No dynamic object starts embedded. Static overlapping solids and their coincident support surfaces need a GPU contact check.

The can is at world (0.160, +0.100) m, at the longitudinal midpoint of the 77.052 mm clear row. The row axis is 7.971604° from +x, or 60° from the left-base-to-can line. The left hand's yaw is −82.028396°. Both dividers span the entire internal drawer length and meet the end frames. The can centre is 230.188 mm from each end frame, 70 mm from the near side's inner face and 230 mm from the opposite side's inner face. This places the row toward one side of a normal-width drawer, keeping the robot base outside the drawer. The 70 mm distance contains the half slot, divider thickness and 15.599 mm unused lane beside that divider. The outer frame blocks entry through the ends. The marked countertop landing centre is (0.240, +0.280) m, on the left arm's side; the right arm remains at READY.

The native Arena `tomato_soup_can_ycb_robolab` measures **67.853995 × 67.749999 × 102.037005 mm**, with a maximum horizontal Feret width of 68.402622 mm. `extent=0.06785399466753006` gives uniform scale **1.0**. Its nominal library mass is 450 g. Measured native dimensions, standing bounds, mesh volume and geometric centre of mass are in `assets/can_info.json`. The mesh COM assumes uniform filling; actual soup distribution is unknown, and automatic native PhysX mass properties remain in use. `assets/drawer_info.json` distinguishes manufacturer and chosen dimensions.

## Feasibility evidence

The table below summarizes the completed CPU checks. Measurement reports, scan results and their scripts are archived outside the repository. The collision test reads the real URDF meshes directly and reconstructs each convex collision component in `test_task.py`; it does not depend on archived files.

| Current selected check | CPU result |
| --- | --- |
| c / clear width / divider height | 4 / 77.052 / 127 mm |
| Can below divider tops / TCP grasp height | 24.963 / 35 mm |
| Full path / 20 Hz frames | 14.4 s / 289 |
| Worst IK position / orientation | 0.079463 mm / 0.011867° |
| Minimum joint-limit margin, normal path / +10 mm reserve | 7.454° / 5.479° |
| Minimum robot-to-task-structure convex distance | 4.495518 mm |
| Carry can bottom above 150 mm frame | 7 mm |
| Nominal release drop | 1.5 mm |

The complete path calls `ArmPlan.action7` at every 20 Hz sample, independently recomputes TCP errors, checks limits and verifies every waypoint 10 mm higher. URDF collision meshes are convexified per existing collision component and checked with FCL/GJK against all task boards and marking strips for both arms. World AABBs prune only proven distant pairs; they do not replace convex distances. Finger closure uses a conservative can-blocked aperture. Grasp acquisition, finger contact and carrying are assumed geometrically, not dynamically simulated. The alternate 180° yaw fails IK and reaches a joint limit; it is not interchangeable for this arm.

The old L-based width rule mixes a palm-to-tip length with a TCP-based depth. Actual palm entry begins at about 70.501 mm above TCP; the selected 92 mm descent depth includes the palm, while the wrist remains above the dividers. Actual full collision geometry is checked regardless of that classification. The selected gap uses the 73.052 mm wrist/palm envelope from the successful grasp plus 4 mm. The fingers' transverse span is 67.999 mm, leaving about 4.5 mm per side at the limiting pose.

## Scoring

`SCORE_OVER="end"`. Highest tier only; `score()` and `distance()` read state without updating history. `success()` updates history once per 20 Hz control step. The framework alone awards 1.0 after a one-second successful hold.

| Score | Conditions |
| --- | --- |
| 0 | No witnessed upright extraction, still inside the drawer, tilted, invalid state/layout, or observed off-countertop event |
| 0.25 | Continuous upward crossing with the can bottom ≥3 mm over the taller drawer frame, then wholly outside the drawer and upright |
| 0.50 | Also standing on the surrounding countertop, both grippers open and both complete hand envelopes clear |
| 0.75 | Also inside the landing tolerance and still |
| 1.00 | Framework holds all full conditions for 1 s |

Only an observed off-countertop event is physically irreversible and latched. Tipping or an inaccurate landing can be repaired. An incorrect initial layout cannot arm the episode. A direct teleport to a finished pose has no witnessed extraction. Latches reset only on a >100 mm observed can-position jump into the complete nominal scene; slow restoration cannot clear them.

The task lazily reads public `isaaclab.sim.SimulationContext.instance().physics_sim_view` articulation views. Finger positions and link transforms determine opening and conservative collision-envelope separation. If either hand reading is missing, release credit fails closed at a maximum 0.25. Local scorer tests substitute synthetic observations in the task-owned adapter; they do not validate this task's GPU readback or patch the framework.

| Threshold (开发估计) | Value | Physical basis |
| --- | --- | --- |
| Upright tilt | <5° | Standing can with limited lean; yaw is unrestricted |
| Nominal start radius | ≤5 mm | Fixed reproducible row placement |
| Fixed board tolerance | 0.5 mm translation; rotation-matrix difference ≤0.001 | Detect changed setup geometry |
| Extraction clearance | ≥3 mm above the 150 mm outer frame | Whole can clears the frame before transfer |
| Countertop support | bottom −1 to +2 mm | Small contact/mesh tolerance |
| Landing centre | ≤15 mm per axis | Fits the can inside the 110 mm marked outline |
| Stillness | all pairs in last 10 positions ≤2 mm | Rejects motion hidden by frozen velocity readings |
| Unfilled window fallback | speed <10 mm/s; spin <0.2 rad/s | Explicit early-frame settling check |
| Hand release | both joint gaps ≥85 mm; hand AABB separation ≥30 mm | Positive open-and-withdraw evidence; conservative mesh envelopes |
| Off countertop | full world AABB within table bounds; bottom ≥−10 mm | Detects escaped/fallen bodies; latched |
| In-place reset | >100 mm jump and nominal full layout | Distinguishes reset from physical repair |
| IK / geometric reserve | <0.5 mm; <0.1°; ≥3 mm; +10 mm reachable | Feasibility tolerances |
| Release drop | 1.5 mm, at most 2 mm | `table_z + release_gap + (pick_tcp_z - table_z)` |

Every lowering follows a stationary alignment waypoint. All numeric configuration fields sent to framework constructors are builtin Python floats. `TASK_DEBUG` alone enables `task_progress`: step 1, every 20 steps, and every step while the can is lifted, displaced, tilted, off the countertop or the hand is approaching the row. Debug includes release-readback reason, extracted flag, pose and score.

## Local tests

Use the existing `yam_env` Python with NumPy, SciPy, pytest and python-fcl, plus the real `yam_arena` checkout installed. From the repository root:

```bash
python -m pytest -q tasks/lift-a-can-out-of-a-drawer-divider-row/test_task.py
```

The complete CPU suite passed 50 tests with no skips; all 16 deliberate source mutations triggered the expected assertion failures. Validation logs and result JSON are kept outside the repository. Tests cover constructor types, tiers, release evidence, repairable tilt, permanent off-countertop history, reset, bad starts, direct terminal jumps, boundaries, full stillness window, read-only scoring, debug cadence, spawn overlap, full demo IK/clearance, pauses and release height. Mutation tests replace task-owned source in memory, never framework code. Asset metadata checks inspect only the JSON distributed here; external texture bytes are not redistributed or audited.

## GPU handoff (not executed locally)

First use the existing Isaac Sim 6.1 / Isaac Lab-Arena 0.3.1 + yam_arena environment for a 200-step scripted smoke:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lift-a-can-out-of-a-drawer-divider-row/task.py \
  -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 200 \
  --instruction 'Take the soup can out of the drawer and leave it standing in the marked area beside it.' \
  --store-frames --log-dir /tmp/drawer-can-smoke
```

Read the result JSON **status**, not the CLI exit code. The smoke stops during placement and need not achieve full score. Check the can's native size and 1 mm spawn gap, 127 mm dividers, 77.052 mm clear slot, 150 mm frame, first `armed=true`, no initial overlaps, recorded divider/frame contact behavior, no joint-limit saturation, tilt <5°, hand readback `ok`, and actual can-bottom clearance ≥3 mm while crossing the outer frame. Confirm the loaded asset hash matches `can_info.json`. Debug must print every critical step.

Then run the complete scripted trial:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lift-a-can-out-of-a-drawer-divider-row/task.py \
  -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 800 \
  --instruction 'Take the soup can out of the drawer and leave it standing in the marked area beside it.' \
  --store-frames --log-dir /tmp/drawer-can-demo
```

The 14.4 s plan plus a default 3 s policy hold fits the 800-step (40 s) run limit. Inspect result JSON status and score, actual release drop ≤2 mm, final bottom tolerance, both open joint gaps ≥85 mm, both hand envelopes ≥30 mm away, centre ±15 mm, ten-frame motion ≤2 mm and a one-second full-success hold. Inspect the actual termination frame, since success may end the rollout before the final waypoint. Wall-clock runtime is unmeasured. No Dream-1, upload or PR command is part of this handoff.

## Limits and assets

No local GPU physics, contact, friction, tracking, camera visibility, library collision decomposition, native inertia/COM, readback integration, success-hold integration or achieved score is certified. Geometric checks sample at 20 Hz; continuous swept volume, robot self-collision, arm-to-arm distance and station camera-stand clearance are not certified. Source API defaults use contact offsets that may create forces before mesh surfaces touch; clearance must be inspected dynamically. No contact parameters are changed. Divider/frame contact history is not scored; the geometric clearance checks describe the scripted path. Alternative physical extraction strategies are not prohibited by scoring.

The registry source is Arena release/0.3.1 at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`. The public source USD came from a 6.2 staging mirror because the tried 6.1 mirror URL returned 404; verify the target GPU bytes and native dimensions. OpenUSD on arm64 emits a cache-line assumption warning but reads all vertices. No live task-board uniqueness check or physical product measurement is claimed.

The soup can is **YCB, CC BY 4.0**: B. Calli et al., “The YCB Object and Model Set: Towards Common Benchmarks for Manipulation Research,” ICAR 2015; [YCB project](https://www.ycbbenchmarks.com/). It remains an external Arena asset; this folder distributes measured JSON only. YAM collision geometry used in verification is the framework's bundled **MIT** robot model; no robot mesh is copied. Drawer boards and marking strips are original analytic fixed geometry. No CC0 model is used.
