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
|PTFE-equivalent stopcock key|50×16×14mm normal key,12mm stem;14g|Native revolute axis+Y at (0.36,-0.047,0.925); closed at0degrees, open near90degrees. Passive finite angular drag approximates the retained plug: zero stiffness/target velocity,0.020Nm s/rad damping,0.010Nm cap. Uncalibrated author settings; no selected-angle return. See PASSIVE-RETENTION.md for sources, units, real assembly/calibration and limits.|
|Receiving beaker|Approx250mL,70mm OD,104mm overall height, glass base4mm and wall3mm; normal external loop handle;145g empty|Base datum (0.36,0,0.7475), already under the outlet. No robot movement of the beaker is required.|
|Clean stopper dish|80×70×6mm|Centre (0.30,0.08,0.7515); place the removed stopper here.|

The two standard YAM parallel grippers have their unmodified stock gains, effort and velocity limits. Three stock D405 camera views are used. The scripted reference uses the left arm to remove and put down the stopper, and the right arm to turn and release the stopcock. These are actual contact actions, not attachment or pose-setting operations.

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

Actual receiver mouth position/tilt decides catch or spill; large beaker tilt spills its retained volume. Fluids do not teleport vessels. No sloshing, droplets, wetting, emulsion, compressibility transients or liquid-dependent rigid-body mass/inertia are simulated. The fixed funnel carries its load through the physical stand; the receiving vessel's changing liquid mass is omitted. Blue and amber closed meshes show the current model volumes. They have no colliders and are authored before simulation starts. Glass visual opacity is 0.02 so overlapping polygonal faces do not hide the layers; this is a rendering approximation, not a measured glass optical property. The right wrist camera sees the interface during valve approach. The initial fixed camera has a small overview, and the READY wrist cameras do not initially face the apparatus. Surface/stream drawings present the model volume and must not be described as an independently validated fluid solver. Real-apparatus calibration remains necessary; the native reference results below establish feasibility only in this disclosed simulator.

## Run and reproduce

Use Python 3.12.13, IsaacSim 6.1.0, IsaacLab 17.0.2, IsaacLab-Arena 0.3.0, inspect-robots 1.0.0, inspect-robots-yam 0.38.0, yam-arena 0.1.0, NumPy 2.3.1, SciPy and Torch 2.11.0+cu128. Run from this task directory in the configured YAM environment. Task loading imports no native USD/Isaac modules before the simulator starts.

```sh
source /path/to/IsaacLab-Arena/yam.env
export ROBOCURVE_TRACE_DIR="$PWD/run-reference/trace"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/scripted.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction 'Recover the lower water phase into the receiver without oil carryover. Remove the stopper and leave the closed apparatus released.' \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames \
  --log-dir "$PWD/run-reference"
```

For Dream-1 use the same command with `--policy dream -P server_url=null`, `-E task="$PWD/task.py"`, a separate trace/log directory and the existing Dream authentication. The model entry does not import the scripted controller. The run limit is 3600 actions, 180 seconds at 20 Hz. Reference sequence completion requests an ordinary transport stop; only the independent grader establishes its score.

Recorded reference runs each produced 2942 fresh observations over about 148 simulated seconds and 10–13 minutes of wall time on an L40S; Dream produced 3601 observations including startup for 3600 actions and approximately 10 minutes of wall time. Actual wall time varies with rendering and model latency. The references repeat seed 0 without object jitter; they do not establish cross-seed or cross-host robustness. A separate local GPU seed-1 attempt on the preceding visually opaque source failed at the initial stopper pickup and is not a portability pass.

```sh
python -m unittest -v test_fluids.py
```

The included six tests check fluid accounting, airlock and outcome logic. Offline IK/collision checks helped author the ordinary reference but do not establish native grasp or success.

Reset the rigid stopper to the neck, key to closed, receiver empty under the outlet, and refill exactly100mL dyed water followed by80mL oil. Let the phases settle fully. Rinse/dry glassware and replace contaminated liquids as necessary. A real-world operator supports fragile glassware and uses a spill tray; no hazardous solvent is needed.

## Prior art and licensing

The actual operation follows Nichols, *Organic Chemistry Lab Techniques*,4.6, “Separate the Layers”: remove the stopper before drainage because air must replace the liquid, then stop at the interface. Reference: https://chem.libretexts.org/Bookshelves/Organic_Chemistry/Organic_Chemistry_Lab_Techniques_(Nichols)/04%3A_Extraction/4.06%3A_Step-by-Step_Procedures_For_Extractions . RoboCurve#1156 concerns mixing/shaking and inverted venting;#2480 concerns a chemical titration endpoint and swirling;#2463 retains a solid stirring bar magnetically during decanting. Those do not directly measure lower-phase recovery through a gas-admission/stopcock system. This is a bounded scope comparison, not a guarantee that no newer submission overlaps; refresh the task board before submission.

All geometry and task/model/reference/grader code are original engineering descriptions under MIT. `partkit.py` is reused from the team's MIT task utility with its accompanying license preserved. No mesh, media, code or artwork was copied from the nearby task PRs or the textbook. The submitted videos come from the visible-phase source described here. Geometry and code are supplied; no third-party native asset files are required.

## Recorded validation

Two complete native references from the final visible-phase source each scored 1.0 after 2942 observations. The separate Dream-1 attempt completed3600 actions and scored0.0. Independent endpoint reconstruction recovered97.852mL water, retained80mL oil, measured no carryover or spill, verified the whole stopper on its dish, all18robot contacts zero, open grippers, a supported upright receiver and a closed/no-flow valve. The endpoint persisted for91fresh observations, exceeding1s. These are simulator results under the disclosed material/flow approximations, not calibrated real-apparatus results.

Before the rendering-only update, a negative with identical physics/controller/grader opened the actual key to48.26degrees while keeping the actual stopper seated: water recovery reached only0.781mL before airlock, with zero spill. A passive initial-condition probe released the key at45degrees with no robot contacts; it stayed at45degrees throughout121observations. Neither probe is a formal task or model result.
