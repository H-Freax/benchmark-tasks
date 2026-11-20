# Clamp an end stop bracket onto a DIN rail against a terminal block and tighten screw

On the station table: an industrial control panel subassembly features a standard slotted 35 x 7.5 mm steel top-hat DIN rail (length 250 mm, oriented along the Y axis) rigidly bolted to the workstation bench at x 26 cm, y 0 cm. A stack of four modular feed-through terminal blocks (Phoenix Contact UT 2.5, width 5.2 mm each, overall stack width 20.8 mm) is mounted centered at y 0 cm. A screw-clamp end stop bracket (Phoenix Contact CLIPFIX 35-5, width 9.5 mm, fitted with a captive slotted M4 clamping screw and sliding steel clamping foot) rests loosely on the rail at y -8 cm. A 3.5 mm flat-blade slotted terminal screwdriver rests on an adjacent tool rest at x 22 cm, y 18 cm. The robot must slide the end bracket along the rail into flush contact with the terminal block stack, hold the bracket and block stack tightly compressed to prevent tilting, engage the screwdriver into the M4 clamping screw, tighten clockwise 1 full turn (torque >= 0.4 N*m) to draw the clamping foot firmly against the DIN rail underside, return the screwdriver to its rest, and clear both arms away.

Terminal strip assembly and maintenance is a fundamental, high-frequency operation across industrial control panel fabrication, machine automation, and building power distribution (Electricians SOC 47-2111, 762k workers; Electrical Assemblers SOC 51-2022, 195k; Industrial Machinery Mechanics SOC 49-9041, 412k; >1.37M US workers). Modular terminal blocks clip onto the DIN rail without longitudinal retention; without firmly clamped end brackets at both ends, terminal blocks slide along the rail, separate under wire bundle stiffness, and sever internal plug-in bridge contacts.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, bimanual demo waypoints, and `score()` / `success()` interface |
| `dinkit.py` | Rigid-body geometries for DIN rail, terminal blocks, CLIPFIX 35-5 end bracket, screwdriver, and panel |
| `scoring.py` | Geometric and temporal scorer with stack compression check, screw torque, 20 N proof push, and clearance |
| `test_scoring.py` | Offline unit tests for scorer logic, partial credit stages, proof push resistance, and IK reachability |
| `task.json` | Task metadata, character-constrained fields, and benchmark configuration |

## Run a trial

From this directory, with the `yam_arena` environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the end stop bracket along the DIN rail flush against the terminal block, hold it snug, and tighten the clamping screw with the screwdriver, then return the screwdriver and clear both arms." \
  --scorer task_score --max-steps 1800 --store-frames
```

## Scripted demo

Run the scripted waypoint sequence:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clamp-din-rail-end-stop-with-screwdriver/task.py -E object_jitter=0 -E control_hz=20 \
  --max-steps 1800 --scorer task_score
```

1. **Approach & Slide End Bracket:** Left arm descends, grasps the CLIPFIX end bracket body, and slides it along the DIN rail from y -8 cm to y -1.5 cm until its vertical face bears flush against the terminal block stack.
2. **Counter-Hold Stack:** Left arm maintains axial compressive force (>= 10 N) against the end bracket to keep the terminal blocks tightly registered against the rail stops.
3. **Pick & Align Screwdriver:** Right arm lifts the 3.5 mm slotted screwdriver from the tool rest, transfers it above the end bracket, and aligns the flat blade with the M4 clamping screw slot.
4. **Tighten Clamping Screw:** Right arm engages the screw slot and rotates 360 degrees clockwise (screw advance 0.7 mm, reaching tightening torque >= 0.4 N*m) to lock the clamping foot onto the DIN rail.
5. **Return Screwdriver & Clear:** Right arm extracts the screwdriver and places it back into the tool rest; left arm releases the end bracket; both arms retract to >= 20 mm clearance. A 20 N lateral proof push is automatically evaluated to confirm zero slip (< 0.5 mm), scoring 1.0 after holding steady for 1.0 s.

### Cheat variants (`DIN_CHEAT=<name>`)

- `nothing`: Arms remain in ready position; scores 0.0.
- `slide_only`: End bracket slid to terminal block but screw not tightened; scores partial 0.25.
- `loose_screw`: Screwdriver engages screw but does not torque past resistance (slips under 20 N push); scores partial 0.50.
- `no_release`: End bracket torqued and locked, but screwdriver or hands remain held; scores partial 0.75.

## Scoring

Scoring evaluates end bracket registration, clamping screw advancement/torque, resistance to a 20 N lateral proof push, screwdriver stowage, and arm clearance:

- **1.0 (Success):** End bracket seated flush against terminal block stack (gap <= 0.5 mm), M4 clamping screw torqued (advance >= 0.6 mm, torque >= 0.4 N*m), slips <= 0.5 mm under a 20 N lateral proof push, screwdriver returned to rest, both arms clear >= 20 mm, and stationary for 1.0 s.
- **0.75:** End bracket clamped and passes proof push, but robot hands or screwdriver not released.
- **0.50:** End bracket flush against terminal block with screwdriver engaged, but screw not fully torqued.
- **0.25:** End bracket slid into position against terminal block.
- **0.0:** Terminal block stack displaced > 5 mm, end bracket knocked off rail, or slip > 2 mm under proof push.
