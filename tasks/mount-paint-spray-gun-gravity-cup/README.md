# Mount a paint spray gun gravity cup onto the fluid inlet and engage the locking cam ring

On the station table: a professional HVLP automotive paint spray gun (SATAjet / 3M PPS 2.0 pattern, mass 0.65 kg) is mounted upright in a heavy steel bench stand at x 24 cm, y -6 cm. The spray gun features an ergonomic handle, forward atomizing air cap, and a top fluid inlet boss (outer diameter 22 mm, inner bore 16 mm, depth 15 mm) equipped with a knurled anodized locking cam collar ring concentric with the vertical inlet axis. A standard 650 mL rigid translucent nylon gravity paint cup (diameter 90 mm, height 143 mm, mass 0.12 kg) with a bottom cylindrical spout nozzle and dual radial bayonet locking lugs sits upright on the table at x 24 cm, y 16 cm. The robot must steady the spray gun handle with the left arm, grasp the paint cup with the right arm, transfer it coaxially above the fluid inlet, insert the cup spout 15 mm axially into the inlet chamber against the internal Buna-N sealing washer, rotate the knurled locking collar ring 45 degrees clockwise past its positive detent click, apply an active 20 N upward retention proof pull to verify mechanical lock, release the cup, and retract both arms clear (>= 25 mm).

Spray equipment assembly, cleaning, and cup changeover is a daily high-frequency task across automotive body refinishing, industrial equipment coating, and manufacturing surface finishing (BLS SOC 51-9122 Coating, Painting, and Spraying Machine Setters and Operators, SOC 49-3021 Automotive Body and Glass Repairers, and SOC 47-2141 Painters; >780,000 US jobs). Under OSHA and EPA spray application standards, gravity paint cups must be positively cam-locked to the gun body before filling or atomization to prevent fluid leaks, catastrophic cup unseating under spray vibration, and volatile solvent exposure.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, bimanual demo waypoints, and `score()` / `success()` interface |
| `mount_paint_spray_gun_gravity_cup.py` | Main implementation: HVLP gun, 650 mL cup, knurled locking collar, and retention proof pull |
| `mechkit.py` | Pure-Python USD physics and mechanism wrapper for hinges, catches, and force sensing |
| `test_scoring.py` | Offline unit tests for scorer logic, partial credit stages, proof pull resistance, and kinematics |
| `task.json` | Task metadata, character-constrained fields, and benchmark configuration |

## Run a trial

From this directory, with the `yam_arena` environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the spray gun upright, seat the gravity cup firmly into the top fluid inlet, rotate the locking cam ring clockwise until it clicks into locked position, verify retention with an upward tug, and clear both arms." \
  --scorer task_score --max-steps 1800 --store-frames
```

## Scripted demo

Run the scripted waypoint sequence:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/mount-paint-spray-gun-gravity-cup/task.py -E object_jitter=0 -E control_hz=20 \
  --max-steps 1800 --scorer task_score
```

1. **Counter-Hold Spray Gun:** Left arm approaches and firmly grips the spray gun handle at x 24 cm, y -9 cm, maintaining rigid counter-holding against insertion and torquing forces.
2. **Pick & Align Gravity Cup:** Right arm grasps the 650 mL gravity cup body, lifts it clear of the table, and transfers it horizontally above the spray gun fluid inlet boss.
3. **Coaxial Insertion & Seating:** Right arm lowers the cup 35 mm axially into the fluid inlet socket until the spout flange seats flush against the Buna-N seal (radial offset <= 3.5 mm, tilt <= 6 deg).
4. **Engage Locking Cam Collar:** Right arm pinches the knurled collar ring tab and rotates the ring 45 degrees clockwise past the spring detent into positive mechanical lock.
5. **Retention Proof Tug & Clear Arms:** Right arm grasps the cup rim and applies an active 20 N upward proof pull for 1.0 s; a locked cup holds firmly with vertical slip <= 0.8 mm. Right and left arms release and retreat clear (>= 25 mm), holding stationary for 1.0 s to score 1.0.

### Cheat variants (`SPRAY_CHEAT=<name>`)

- `nothing`: Arms remain in parked position; scores 0.0.
- `align_only`: Cup lifted and aligned above inlet but not inserted; scores partial 0.25.
- `seat_only`: Cup inserted into fluid inlet but locking collar ring not turned; scores partial 0.50.
- `no_release`: Cup seated and collar locked, but robot arms remain grasping objects; scores partial 0.75.

## Scoring

Scoring evaluates coaxial alignment, axial spout insertion depth, collar ring rotation past detent, resistance to an active 20 N upward retention proof pull, and complete arm clearance:

- **1.0 (Success):** Gravity cup seated coaxially in fluid inlet (radial offset <= 3.5 mm, tilt <= 6 deg), locking collar rotated clockwise >= 40 deg into positive detent, 20 N upward retention proof pull held 1.0 s with vertical slip <= 0.8 mm, both arms released and retracted clear >= 25 mm, and stationary for 1.0 s.
- **0.75:** Gravity cup seated and collar locked past 40 deg detent, but robot arms unreleased or proof not held.
- **0.50:** Gravity cup spout seated coaxially into fluid inlet socket (depth >= 12 mm, offset <= 3.5 mm).
- **0.25:** Gravity cup lifted from table and aligned within 25 mm XY of inlet axis.
- **0.0:** Gravity cup dropped below table level, overturned > 45 deg, or pulled loose under proof test.
