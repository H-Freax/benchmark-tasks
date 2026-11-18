# Drain the lower phase from a separatory funnel

After a liquid–liquid extraction has settled, recover the dense aqueous layer through the stopcock without contaminating it with the upper oil layer. Remove the top stopper so replacement air can enter, use the visible interface to slow and stop drainage, close the stopcock and leave the apparatus and receiver supported and released. The receiver is already under the outlet, as a normal fail-safe laboratory setup.

Use **only room-temperature tap water and edible vegetable oil** for a real trial: 100mL water dyed blue and80mL oil. No volatile solvent, acid, alkali, heating or pressurised source. This task measures post-settling selective phase drainage. It does not include shaking, mixing or chemical titration.

## Apparatus and fixed layout

Dimensions are original, full-size engineering equivalents, not OEM CAD or measured manufacturer specifications. Coordinates are in the stock calibrated YAM station frame, metres. `scene_spec.py` reconstructs every solid as boxes/cylinders; `geometry/scene.json` records the generated parts and datums. The table is the normal station table at z0.7475m.

| Item | Construction and dimensions | Initial state |
| --- | --- | --- |
|250mL nominal separatory funnel|Borosilicate-equivalent wall2mm; conical sump90mm high, inner top radius32mm; upper cylinder50mm high; reduced neck about22mm ID; open downward outlet2.5mm effective flow bore|Upright in a weighted ring stand; cone bottom at (0.36,0,0.925), outlet at (0.36,0,0.852).100mL water below80mL oil, settled.|
|Stand and clamp|Visible200×160×12mm weighted base,350mm post and rubber-lined neck support; combined funnel/stand mass1.72kg|Stand and clamped funnel are fixed to the bench as an explicitly mounted apparatus. No robot or removable object is fixed.|
|Stopper|25×25×16mm cap,21.96mm cylindrical seating stem28mm long; total28g|Cap centre (0.36,0,1.119); snug idealised stem in the upper neck. Freely removable by contact.|
|PTFE-equivalent stopcock key|50×16×14mm normal key,12mm stem;14g|Native revolute axis+Y at (0.36,-0.047,0.925); closed at0degrees, open near90degrees. The declared joint-friction attribute0.005 was reported ignored by native PhysX for this standalone hinge; effective hinge friction remains uncalibrated/unknown. No angular drive or opening motor.|
|Receiving beaker|Approx250mL,70mm OD,104mm overall height, glass base4mm and wall3mm; normal external loop handle;145g empty|Base datum (0.36,0,0.7475), already under the outlet. No robot movement of the beaker is required.|
|Clean stopper dish|80×70×6mm|Centre (0.30,0.08,0.7515); place the removed stopper here.|

The two standard YAM parallel grippers have their unmodified stock gains, effort and velocity limits. Three stock D405 camera views are used. The scripted candidate uses the left arm to remove and put down the stopper, and the right arm to turn and release the stopcock. These are actual contact actions, not attachment or pose-setting operations.

## Score and real measurement

Evaluate the end state. Record the graduated receiver, residual water in the funnel, the interface, oil carryover, spill tray and final released state. Use a0.5mL-graduated small cylinder/pipette for final residual/carryover measurements; drain the two final receiver phases separately for measurement if needed. These are author-chosen tolerances.

If more than1mL spills, more than0.5mL oil enters the receiver, or the valve remains flowing at the end, score0. Unknown/invalid observations score0 in the software, with their reason retained. For a closed valve and no excessive spill/contamination, award by water recovered into the receiver:

- Below50mL:0.
-50to under75mL:0.25.
-75to under97mL:0.5.
- At least97mL:0.75.
-1.0 requires at least97mL water recovered, at least79.5mL oil retained in the funnel, stopper wholly on the clean dish, closed/no-flow valve, receiving beaker supported upright, both grippers actually open and all18 robot rigid bodies clear of the stopper, key and receiver. The safe released terminal state must persist for1s.

In a real trial, inspect clearance/release in the video and check no drop falls for30s after closing. Simulation closes the valve within3degrees and requires estimated flow at most0.005mL/s. It measures full native robot contact-count coverage for all three removable/manipulated targets; missing/misbound/saturated contact views cannot establish release. The continuous stability counter resets if actual physics observations skip or reverse, and only fresh adjacent control observations accumulate it.

`grade.py` is independent of the reference policy and accepts measured outcome records. No reference phase, requested angle, motion count, elapsed drain time, or remembered success event earns liquid-recovery credit. The normal task is `task.py`; `scripted.py` only supplies ordinary14D joint targets for a privileged reference candidate.

## Fluid and contact fidelity

Rigid bodies, stopcock hinge, grasping, stopper removal, collision and support are native physics. The stand/clamped funnel is visibly mounted, not a hidden object fixture. There is no run-time object pose writer, robot attachment, automatic key turn or snap-to-seat. The only native shape transforms updated by the task are collider-free visual liquid indicators.

Liquids use an explicit quasi-static1D approximation, not native particles. Density998kg/m³ for water and915kg/m³ for oil; water drains first. Actual key rotation determines a sin² bore-overlap area and actual stopper separation admits atmospheric air; a sealed upper chamber follows isothermal PV pressure and naturally airlocks after a small amount drains. Hydrostatic head and a2.5mm effective orifice use Torricelli flow, Cd0.62 for water and0.22 as an uncalibrated edible-oil viscosity surrogate. The closed effective chamber volume is250mL. These chosen parameters are disclosed approximations, not measured glassware curves.

Actual receiver mouth position/tilt decides catch or spill; large beaker tilt spills its retained volume. Fluids do not teleport vessels. No sloshing, droplets, wetting, emulsion, compressibility transients or liquid-dependent rigid-body mass/inertia are simulated. The fixed funnel carries its load through the physical stand; the receiving vessel's changing liquid mass is omitted. Surface/stream drawings present the model volume and must not be described as an independently validated fluid solver. Safe real-world calibration and final native feasibility remain necessary.

## Run and reproduce

Use the existing YAM Isaac Lab-Arena environment with Python3.12.13, IsaacSim6.1, the pinned YAM/Isaac Lab versions from the deployment, and `inspect-robots`1.0.0. The task uses NumPy/SciPy and the local `partkit.py` hook; it imports no native USD/Isaac modules during task loading.

```sh
source /path/to/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/scripted.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction 'Recover the lower water phase into the receiver without oil carryover. Remove the top stopper, operate and close the stopcock at the interface, and leave the stopper in its dish and the apparatus released.' \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames \
  --log-dir "$PWD/run-reference"
```

Set `ROBOCURVE_TRACE_DIR` to a dedicated output directory before each run.3600 controls at20Hz is180s simulated time. The current pure CPU ordinary-reference plan uses about2400controls; actual native wall time, reference success and model score are **not yet measured**. A Dream-1 attempt uses `task.py` and the existing `dream` policy, separate from the scripted reference.

```sh
python -m unittest -v test_fluids.py
python cpu_path.py
python collision_check.py
```

`CPU-PATH.json` is a hypothetical IK plan, and `CPU-630-HULLS.json` checks all630 stock arm collision hulls against this plan. They do not prove native grasp, flow, closing or task success. After native validation, preserve the source hash, complete observations, native media and independent endpoint score before submitting.

Reset the rigid stopper to the neck, key to closed, receiver empty under the outlet, and refill exactly100mL dyed water followed by80mL oil. Let the phases settle fully. Rinse/dry glassware and replace contaminated liquids as necessary. A real-world operator supports fragile glassware and uses a spill tray; no hazardous solvent is needed.

## Prior art and licensing

The actual operation follows Nichols, *Organic Chemistry Lab Techniques*,4.6, “Separate the Layers”: remove the stopper before drainage because air must replace the liquid, then stop at the interface. The local source record preserves the exact reference URL. RoboCurve#1156 concerns mixing/shaking and inverted venting;#2480 concerns a chemical titration endpoint and swirling;#2463 retains a solid stirring bar magnetically during decanting. Those do not directly measure lower-phase recovery through a gas-admission/stopcock system. This is a bounded scope comparison, not a guarantee that no newer submission overlaps; refresh the task board before submission.

All geometry and task/model/reference/grader code are original engineering descriptions under MIT. `partkit.py` is reused from the team's MIT task utility with its accompanying license preserved. No mesh, media, code or artwork was copied from the nearby task PRs or the textbook. Native reference, model media and platform submission IDs are pending; this folder is a development candidate.

Observer revision2 reads actual native robot tensors directly, with backend body/joint name order, instead of cached ProxyArray slices. The first observation exception records its full traceback and stage. No geometry, controller, physical parameter or grade change accompanies this revision.

Stopper reference revision4 keeps its SIDE grasp orientation during transfer and uses a normally placed dish at(0.30,0.12) rather than rotating the grasped stopper in midair. The completed reference requests normal transport stop regardless of its actual score; only the independent grade defines success.

Revision5 latches the actual whole-stopper geometry in the current hand frame after grasping, computes the desired TCP so the predicted whole-stopper XY bounds center—not the hand center—lands over the clean dish, and preserves the hand orientation. The ordinary dish is at(0.30,0.08). Its80x70mm size, score and material are unchanged. A new small grasp_geometry.py helper is required alongside reference.py.

The reference withdraws the open right gripper straight along the key approach axis to a clear point while preserving its orientation, before rotating joints back to READY. Actual complete robot-key contact counts guard against recontact during withdrawal. This corrects the observed V4 recontact impulse during direct joint retreat; no hinge friction, gain or force changes are used.

Category: Other. This is a common wet-lab research procedure following liquid-liquid extraction; no claim is made about a qualifying500,000-worker employment domain.

Revision6 is a single bounded controller-only unloading test: hold the closed key for60 controls (up to120 if fresh pose differences are not quiet), then open the gripper over100 controls instead of25. The scripted held-pose check uses actual angle within3degrees, estimated flow<=0.005mL/s and10 consecutive fresh pose differences<=0.02rad/s. It logs native angular velocity separately and explicitly flags disagreement; a quiet pose difference does not establish that the native velocity channel is quiet. Scoring, all physical parameters and the final no-flow/released-stability rules remain unchanged.


## Submission status

This is the current implementation submitted for review before all feasibility repairs are complete. The completed V5 native reference scored 0.0: 98.36mL water recovered, 80mL oil retained, no carryover or spill, and stopper released in the dish, but residual valve motion reopened the stopcock. The uploaded demonstration records that failed attempt and is not a full-score demonstration. V6 adds a measured settling phase and slower release; its complete native result and the separate Dream-1 result are pending. No model success, full-score feasibility or acceptance is claimed. The physical scene, fluid model and scorer in the V5 recording are byte-identical to this version; only reference controls and metadata changed.
