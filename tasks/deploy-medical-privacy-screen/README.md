# Deploy a medical privacy screen

Position a free-standing, three-panel medical screen to hide the marked treatment-area target from three specified doorway viewpoints, leave all six feet supported, and release the screen. Visual privacy while changing or receiving personal care is the intended real-world use. This task measures optical shielding and sampled stable placement; there is no patient, clinical procedure or validated clinical-safety claim.

This review revision adds three visible doorway viewing marks and raises partial coverage to90% (511/567). The unchanged initial U layout blocks486/567 and earns0. Current-revision native reset, camera visibility, complete references, setup capture, matched demonstration and model attempt remain pending. Earlier reference1.0/824 controls and model0.5/1,200 controls are previous-revision historical diagnostics, not current scores or media. See `task.json`, `PHYSICAL-PROTOCOL.md`, `BOM.csv`, `SETUP.json`, `geometry/OBSERVATION-MARKS.json` and `assembly/`.

## Construction and fixed starting layout

Use metres for world positions, millimetres for construction dimensions. World x points away from the arm bases, y left, z up. The simulation floor top is z=-0.0125 m. All six panel foot bottoms settle onto this actual floor; authored panel roots begin at z=0 before gravity settling.

| Object | Construction and material | Count / declared mass |
|---|---|---|
| Screen panel | Nominal 560 wide ×1652 high; two 30×20×2-wall aluminium tubes, 1622 long; two 500×20×30 aluminium horizontal tubes, 2-wall; one opaque rigid PVC-foam inlay, 500×3×1542; two rigid rubber-equivalent protectors, 32×24×30 | Three panels, 3.3156072 kg each; 9.9468216 kg linked moving total |
| Vertical screen hinges | Two passive revolute connections, native adjacent-panel contact enabled; zero stiffness, 0.08 N m s/rad damping | Two ideal joints; hardware mass omitted |
| Robot pedestals | Fixed steel-equivalent blocks, 140×140×750; bottom centres (0.15,+0.55,0) and (0.15,-0.55,0) | Two, 115.395 kg each |
| Robots | Stock YAM parallel-gripper arms, base centres (0.15,+0.55,0.75) and (0.15,-0.55,0.75); original gains, effort and gripper limits | Two, upstream robot assets |
| Treatment target | Fixed opaque PVC-equivalent board, 8×1040×940, centre (1.50,0,1.05), with 189 dark 1×6×6 dots | Board and dots together 3.913802 kg |
| Doorway viewing marks | Three amber square outlines, 76×76×4 mm outside, 60×60 mm clear aperture; centres equal the three scored sight points; fixed, non-colliding context | Three; nominal transport mass 0.001 kg each, excluded from moving mass |
| Room context | Native station ground, lighting and camera stand/housing; station table removed; three policy cameras | Upstream station context; no extra loose objects |

Panel frame coordinates, all finite solids, materials, densities, friction assumptions, masses, inertia and initial transforms are in `geometry/geometry.json`; `BOM.csv` lists each component and world bounds. The maximum panel width including feet is562mm. Initial U-shaped screen bounds are592×614mm, height1652mm; this is a partly deployed screen, rather than a compact folded storage state. Its geometric initial coverage is486/567 sight segments (85.7143%); this unchanged layout earns0 even after a complete valid final window because486 is below511.

| Panel | Authored origin (x,y,z) | Yaw | Initial foot centres (x,y), before settle |
|---|---|---|---|
| centre | (0.950,-0.280,0) | 90° | (0.950,-0.265), (0.950,+0.265) |
| left wing | (0.965,-0.295,0) | 180° | (0.950,-0.295), (0.420,-0.295) |
| right wing | (0.935,+0.295,0) | 180° | (0.920,+0.295), (0.390,+0.295) |

The centre/left hinge uses local anchors (0,-0.015,0) on each body; centre/right uses (0.560,+0.015,0) on the centre and (0,+0.015,0) on the wing. Initial world axis xy positions are (0.965,-0.280) and (0.935,+0.280). Both are vertical; the abstract z=0 reference denotes a point on the axis, not a pin attached to the floor. Relative to the authored joint frames, the left hinge limits are -170° to+80° and the right hinge limits are -80° to+170°; finite-body contact still constrains legal motion. No particular final hinge angle is required.

Target dot centres are x=1.495, y=-0.50..+0.50 in0.05 increments and z=0.60..1.50 in0.1125 increments. Sight coordinates are (0,-0.30,1.10), (0,0,1.10), (0,+0.30,1.10). Three amber square outlines named `doorway_view_1` through `doorway_view_3` are centred exactly on these scored observer positions, in planes x=0 facing the target. They have no colliders and cannot support the screen. In a physical layout reproduce the same outlines on independent camera-positioning jigs, with camera optical centres at the aperture centres. Use `geometry/OBSERVATION-MARKS.json` for exact dimensions and floor registration. The earlier setup image lacks these new marks and is historical; fresh native policy-camera visibility checks and a setup capture remain pending.

The drawings give cut lengths, frame/inlay retention, a proposed pair of short coaxial hinges at300/1300mm per axis, drilling and foot fasteners. This physical proposal is unbuilt. Bolts, sleeves, inserts, adhesive and weld details are not silently included in the simulated ideal joints or masses. See `assembly/README.md` for the omitted hardware envelope and volume-based mass estimate.

## Reset and actions

Start a fresh episode. Restore every fixed object and the panel origins/yaws above, set both arms to stock READY joint angles [0,1,1,-0.6,0,0] rad and fully open grippers, and allow the normal0.5s gravity settle plus camera warm-up. Object jitter is disabled. Restore the original U layout after every trial; nothing is consumed. Do not reset task-body poses, add hidden supports, lock joints or attach the screen to a hand during a trial. The reference emits ordinary arm-joint/gripper targets and checks actual jaw/pad contact before moving its grasp.

For a physical trial, register the actual floor plane and all templates, remove temporary human support and alignment jigs, confirm free hinges and six-foot support, and verify the target and bases. Hardware retention, tipping risk and robot clearances require physical validation before attempting a real trial; this package does not claim such a trial has occurred.

## Score and limit

A trial allows at most1,200 controls:60 simulated seconds at20Hz, with960Hz physics. The final acceptance window must fit inside that limit. The previous-revision historical seed-1 scripted reference took1,026.556 seconds of wrapper wall time, including simulator startup/reset, and completed at824 controls/41.2 simulated seconds. Wall runtime varies with hardware, renderer and policy/API latency; the measured Dream-1 wrapper wall runtime was1,517.820 seconds. External wall-time caps do not add robot actions.

Use exactly the final21 fresh canonical20Hz observations, spanning1.000s. At every sample, require all three panels upright within5°, all six foot bottoms within±5mm of the actual floor, both hinge-anchor gaps≤5mm and finite-solid Euclidean screen-to-robot clearance≥50mm. Clearance covers all630 actual collision hulls of both robots, including link, hand and finger geometry. Across that same window every panel origin must stay within5mm of its first-sample origin and its shortest SO(3) change within1°. Native velocities must be finite; their magnitudes are not score gates.

Coverage counts567 finite segments from the three sight points to the189 target dots. Only intersection with a finite opaque screen solid before the dot blocks a ray; hinge gaps remain open. Score1.0 requires at least562/567 blocked at every sample; score0.5 requires at least511/567 throughout, without satisfying the full tier. Valid evidence meeting neither tier scores0. Missing/stale/nonfinite evidence or unresolved geometry is invalid/indeterminate, rather than proof of a physical violation. A conservative clearance lower bound below50mm requires refinement or unknown. Neither hand assignment, two-hand use, prescribed angle nor reference phase is a scoring gate. Physical judging uses the direct-photo and ordinary-gauge procedure below; sampled conditions do not certify continuous-time safety or motion between samples.

### Human physical judging

[PRIMARY-HUMAN-SCORING.md](PRIMARY-HUMAN-SCORING.md) is the primary proposed real-trial method. At each of the same21 instants, read the189 registered dot centres in each of three fixed-viewpoint pictures: the567 centre sight tests are judged directly as screen-blocked, visible or uncertain. This avoids a requirement to reconstruct a room in3D or trace software rays. Near-edge, blurred, occluded or misregistered centres remain uncertain; shadows or obstruction by a robot do not count as screen coverage. The full/partial count thresholds remain562/511.

Use floor/side rulers and a5mm feeler for all six foot-bottom heights, a level/angle gauge for5° upright, and registered frame landmarks/rulers for each5mm hinge-anchor separation. Bound x/y/z origin changes conservatively and require the Euclidean upper bound≤5mm. For rotation, the sum of absolute yaw/pitch/roll changes and their error bounds≤1° is sufficient for the same SO(3) condition; each axis≤1° separately is insufficient. An insufficient upper bound is inconclusive, not proof of motion beyond the limit.

Check every real robot exterior, including links, hands and fingers, against the screen using ordinary50mm clearance gauges and complete views of possible closest regions, or a verified whole-object separation bound. For example, ruler/plumb-line planes that bound all robot exteriors on one side and all screen exteriors on the other, with a verified gap≥50mm, establish the same Euclidean clearance. The630 hulls are the simulator's complete representation; physical judges do not have to reconstruct630 meshes.

Place levels or insert gauges only after the window while uninterrupted video establishes the same screen/robot pose; never let a gauge support the screen or disturb a robot. A changed or unverified measuring pose cannot certify the earlier window. Missing views, uncertain boundaries or inadequate reading precision are indeterminate, rather than unknown being scored as zero. This is a proposed ordinary-tool procedure, with no claim of demonstrated physical precision or a completed physical trial.

## Run independently

Install [YAM Arena](https://github.com/kairosiann/yam-isaac-lab-arena) and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena), including the in-process Dream policy, using their setup instructions. Use a Linux NVIDIA RTX host and activate its Arena environment. `RUNTIME.json` records the package versions used to prepare this candidate; successful installation alone is not task qualification. Obtain Dream credentials through the client's normal protected mechanism; never put them in the task directory.

From the benchmark repository root, after placing this folder at `tasks/deploy-medical-privacy-screen`:

```sh
TASK_DIR="$(pwd)/tasks/deploy-medical-privacy-screen"
RUN_DIR="$(pwd)/runs/privacy-screen-reference-seed1"
export ROBOCURVE_GEOMETRY_ROOT="$TASK_DIR/geometry"
export ROBOCURVE_TRACE_DIR="$RUN_DIR/trace"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
: "${GPU_LOCK:?Set GPU_LOCK to the host existing global GPU lock path}"
: "${GPU_SLOT_LOCK:?Set GPU_SLOT_LOCK to the host allocated existing GPU slot lock path}"
test -f "$GPU_LOCK" && test -f "$GPU_SLOT_LOCK" || { echo "Use the host existing lock files." >&2; exit 1; }
! test "$GPU_LOCK" -ef "$GPU_SLOT_LOCK" || { echo "Global and slot locks must differ." >&2; exit 1; }
flock --shared "$GPU_LOCK" flock --exclusive "$GPU_SLOT_LOCK" \
  inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$TASK_DIR/native/reference_entry.py" \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction 'Set the privacy screen so the marked treatment area is screened from all three amber doorway viewing marks. Leave it standing securely, then let go.' \
  --scorer task_score --max-steps 1200 --seed 1 --no-prompt --store-frames \
  --log-dir "$RUN_DIR/inspect"
```

For the formal model attempt, select `model_entry.py` and Dream-1; this entry does not import the scripted reference:

```sh
RUN_DIR="$(pwd)/runs/privacy-screen-dream1-seed1"
export ROBOCURVE_TRACE_DIR="$RUN_DIR/trace"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
: "${GPU_LOCK:?Use the host existing global GPU lock path}"
: "${GPU_SLOT_LOCK:?Use the host allocated existing GPU slot lock path}"
test -f "$GPU_LOCK" && test -f "$GPU_SLOT_LOCK" || { echo "Use the host existing lock files." >&2; exit 1; }
! test "$GPU_LOCK" -ef "$GPU_SLOT_LOCK" || { echo "Global and slot locks must differ." >&2; exit 1; }
flock --shared "$GPU_LOCK" flock --exclusive "$GPU_SLOT_LOCK" \
  inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P timeout_s=90 \
  -E "task=$TASK_DIR/native/model_entry.py" \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction 'Set the privacy screen so the marked treatment area is screened from all three amber doorway viewing marks. Leave it standing securely, then let go.' \
  --scorer task_score --max-steps 1200 --seed 1 --no-prompt --store-frames \
  --log-dir "$RUN_DIR/inspect"
```

Obtain `GPU_LOCK` and the allocated `GPU_SLOT_LOCK` from the host scheduler before either command. Both paths must name the host existing lock files; do not create substitute locks or choose another slot to bypass reservations. Both commands take the global lock shared and the allocated slot exclusive. Use new run directories. Read the finalized non-live Inspect JSON: its `task_score`, actual control count, seed, revision and termination must agree with the native trace and final one-second window. A shell exit0 or a live log is insufficient. Export every stored camera frame, including initial0 and terminalN, with `inspect-robots video FINAL_LOG.json --fps 20 --out MEDIA_DIR`. Record the actual measured wall runtime and match both videos to this package's final source hashes before filling `scores` or uploading media. The documented reference entry has a matched seed-1 score1.0 demonstration; the Dream-1 result is0.5 after its full1,200-action budget. Neither the reference success nor this setup image establishes a calibrated physical trial or task-board acceptance.

## Sources, license and task distinction

`PROVENANCE.md` lists the engineering/context sources and retained controller licenses. Geometry and drawings are original engineering equivalents; they do not redistribute OEM CAD. Upstream robot/station assets retain their own licenses.

`RELATED-TASKS.md` compares the saved task-board/GitHub results. The intended contribution is the deployment of a linked, full-height, independently supported visual barrier with multi-view coverage and released stability. Related PIN shielding, slotted-partition assembly and eyelet-curtain hanging exercise different objects, actions and outcomes. This comparison is evidence for review, not an ownership or acceptance decision.

The Dream client returned25 successful action chunks across49 SDK attempts, including24 expired-session retries. No environment reset or scripted policy was substituted. Both runs use the same packaged scene/scorer and stock robot settings; this single fixed-layout result is not a general model-capability claim.
