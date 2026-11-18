# Set an internal expansion anchor

Use the punch and hammer to set the anchor in its prepared concrete hole. Withdraw the punch, return both tools to their ordinary supports and leave the anchor stable with both arms clear.

This is an implemented candidate with recorded unsuccessful reference attempts. No full-score demonstration, physical load rating or hardware calibration is claimed. The current model and reference runs must be reported with their actual scores and matching media.

## Setup and fabrication

`geometry/geometry.json` contains every task solid's convex vertices, physical pose, material assumptions, mass, inertia and all four material-root joints. `SETUP-INVENTORY.json` gives the dimensional bounds, mass and starting pose of every body. All dimensions are in metres and kilograms. No external task asset download is required.

The concrete coupon is 120×120×80 mm, with a 12.7 mm prepared bore 39.6875 mm deep. The exterior anchor dimensional class is 3/8-inch/DIAB 37. It contains four finite steel sleeve petals and a free internal cone. The setting punch has a 6 mm lower shaft and 12 mm shoulder/upper grip. The unchanged steel/ash hammer weighs 0.41284 kg. Two ordinary 152.5 mm-high welded-steel box pedestals put the stock YAM bases at z=0.900 m. The workpiece origin is x=0.300 m, y=0.000 m. A wooden cradle supports the hammer; two wooden V rests support the withdrawn punch horizontally. Exact fabrication solids, support heights and assembly poses are in the two JSON files.

The sleeve material roots are permanent internal flexure approximations, not world fixtures. Plastic rest angle updates respond only to measured strain. There is no event-triggered expansion, object teleport, robot-tool attachment or task-applied external wrench.

Reset to the declared poses with a fresh sleeve/anchor after each trial. Plastic setting is not reversed during a run. Keep the same full-size punch, hammer and supports. The block may be held by its normal work stand; the sleeve, cone, punch and hammer remain physically free as declared.

## Run and reproduce

Install Isaac Sim 6.1, Isaac Lab/Arena and the YAM Inspect Robots embodiment using the runtime repository linked in `SOURCE-LICENSE.md`. Activate that runtime's environment and install NumPy and SciPy there. Use seed 0 and zero object jitter. The task itself sets 960 Hz physics,20 Hz control and 48 physics substeps per action in `native/task_runtime.py`; the stock robot gains, limits and gripper capability are unchanged.

From this task directory, run the ordinary reference:

```sh
ROBOCURVE_TRACE_DIR="$PWD/run-reference/trace" inspect-robots run \
  --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/native/full_entry.py" -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction 'Set the anchor in the prepared concrete hole, withdraw the setting punch, and leave both tools supported.' \
  --scorer task_score --max-steps 1440 --seed 0 --no-prompt \
  --store-frames --log-dir "$PWD/run-reference/inspect"
```

For Dream-1, use `--policy dream` and `-E task="$PWD/native/model_entry.py"`, with all other scene, seed, control, instruction and scorer settings identical. Load credentials through the normal runtime configuration. The model entry imports the physical scene and scorer only; it does not import the reference controller or its plan.

The limit is 1440 actions/72 simulated seconds. The nominal reference contains 1389 controls plus 20 initial settle. Actual wall time is reported by the run log. Previous partial attempts used the same physical geometry and ended at 0; the current complete reference has not yet demonstrated full completion. Run logs and media must establish its actual result.

The reference grasps the punch and the existing hammer handle at local x 225 mm. It uses ordinary robot joint commands, actual pose/contact feedback and up to six real strokes. Each 100 mm descent takes 8 controls. Before a stroke, it adjusts the left wrist from the measured shoulder/ring lateral and axis errors; correction is bounded to 2 mm/2 degrees total. Internal arrival is.30 mm with five quiet actual samples, not a change to the formal.4 mm scoring gate. Failed contact, material-grasp or geometry checks retain the previous joint target and stop normally. Tools are never repositioned by writing their poses.

## Scoring and evidence

The outcome scorer never reads reference phases or commands. It uses actual object poses, native contacts, material observations and all robot links:

- 0: unset, invalid or insufficiently verified outcome.
- 0.5: an actual visible shoulder-seating witness exists, the anchor remains within.8 mm of its initial origin and 2 degrees upright, all four petals contact the bore wall and the actual cone has moved down at least 1.4 mm. At the witness the shoulder gap is−0.20 to+0.15 mm, lateral offset is below.4 mm and axes differ by less than 2 degrees.
- 0.75: the set anchor remains retained, the complete punch is at least 10 mm clear axially or entirely outside the bore projection, and 11 fresh samples span.5 s of stability.
- 1.0: both tools are released in their declared supports, all robot links are at least 20 mm clear of the tools/anchor, and 21 fresh samples span 1 s of final stability. Native linear speeds must be below 5 mm/s and angular speeds below.1 rad/s; quiet poses do not override contradictory native velocities.

The real-world endpoint is inspected with a shoulder/feeler check, ruler and calibrated photos of the withdrawn punch, supported tools and robot clearance. The cone/material/contact conditions are simulation consistency guards for the declared mechanism, not a claim of a structural proof-load test. Physical calibration and alignment of the simulator's auxiliary checks with a manufactured prototype remain limitations.

`observations.jsonl` records actual bodies, complete robot poses, contacts and independently recomputed score. `impact-contacts.jsonl` samples hammer/punch and punch/cone contact at each fresh physics step; a 20 Hz snapshot alone cannot prove an impact was absent. `material.jsonl`, `material-grasp.jsonl` and `punch-alignment.jsonl` preserve the constitutive and controller measurements. Rescore a recorded trace without physics:

```sh
python native/score_trace.py run-reference/trace/observations.jsonl --geometry geometry/geometry.json
```

## Fidelity and limits

This is a full-size original manufacturable equivalent. The upper threaded region is a smooth clearance-bore proxy. Concrete cracking, spalling, thread engagement, structural pull-out capacity and OEM setting-force certification are outside scope. Steel density 7850 kg/m³, modulus 200 GPa and yield 250 MPa imply root stiffness 250 Nm/rad and yield moment 1.25 Nm under the declared approximation. These are A 36-class engineering inputs, not a calibrated product curve. Steel/steel friction.25/.20 and steel/concrete.50/.40 are disclosed estimates. Masses and inertias come from the stated nonoverlapping solid volumes and densities. Robot materials and actuator limits are not adjusted to obtain a score.


## Recorded submission evidence

The uploaded Dream-1 attempt completed its full allowed action budget and scored 0.0. The uploaded scripted reference also scored 0.0; it is an unsuccessful attempt, not a full-score demonstration. Both videos are original simulator camera streams at 20 fps. The task is submitted with the implementation and these current results; full-score feasibility remains unresolved.
