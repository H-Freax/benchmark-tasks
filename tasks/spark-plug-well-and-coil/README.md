# spark-plug-well-and-coil

Install a spark plug into a recessed cylinder-head well using a specialized spark-plug socket tool, rotate and thread it home into the combustion chamber seat, withdraw the tool, and seat the ignition coil-on-plug (COP) unit over the plug terminal flush against the mounting boss.

## Task Details

- **Title:** Install a spark plug in a recessed cylinder-head well with a socket tool and seat its ignition coil
- **Category:** Impact on jobs
- **Keywords:** `spark plug`, `ignition coil`, `cylinder head`, `socket tool`, `automotive service`, `bimanual`, `recessed well`
- **Instruction:** Take the spark plug socket tool, capture the spark plug from its staging tray, lower it into the cylinder head well and thread it home. Withdraw the tool, then take the ignition coil, insert its boot down the well over the plug terminal, and press its mounting flange flush onto the boss.
- **Run Limit:** 800 steps (40 s at 20 Hz)

## Occupational Relevance

Automotive service technicians and mechanics (SOC 49-3023; 760,000+ US jobs, BLS OEWS May 2023) perform spark plug and ignition coil replacements during routine tune-ups, scheduled maintenance, and diagnostic misfire remediation. In overhead-cam cylinder heads, spark plugs sit recessed at the base of narrow cylinder head wells. Mechanics use specialized spark plug sockets fitted with internal rubber retention sleeves to capture the ceramic insulator, lower the plug into the well without dropping it (which fractures brittle ceramic insulators or bends the preset ground electrode gap, causing cylinder misfires), thread it into the cylinder head threads with manual tool rotation, pull the socket tool free off the seated plug, guide the flexible rubber coil boot down over the terminal stud, and press the coil mounting flange flush against the tapped boss.

## Setup & Real-World Hardware Reproduction

All items are standard, low-cost, off-the-shelf automotive components with total bill of materials under $60:
1. **Cylinder Head Fixture:** Authentic production overhead-camshaft (OHC) cylinder-head training section (Toyota 1NZ-FE / Honda R18A 1.8L DOHC cylinder head section, OEM part 11101-21030 / ConsuLab benchtop trainer, $55-$75 used/rebuilt on eBay/RockAuto). The 180 x 140 x 40 mm aluminum casting is rigidly clamped at (0.26, 0.0, table deck z=0.7475 m). It features an authentic deep counterbored spark plug well (30 mm diameter bore, 30 mm depth from top deck z=0.7875 m down to internal M14x1.25 threaded seat at z=0.7575 m, with thread entry counterbore at z=0.7635 m) and an adjacent integrated M6 tapped ignition coil mounting boss at (0.295, 0.0, z=0.7875 m).
2. **Spark Plug:** Standard 14 mm x 1.25 threaded automotive spark plug (NGK BKR6E, $3): 12 mm threaded steel stem (14 mm diameter), 6 mm hex collar (16 mm width across flats), 18 mm ribbed white ceramic insulator (10.5 mm diameter), and 6 mm brass terminal stud (4 mm diameter); overall length 42 mm, mass 35 g. Starts standing upright in a 14.5 mm socket staging tray at (0.22, -0.12 m).
3. **Spark Plug Socket Tool:** 5/8" (16 mm) spark plug retention socket tool ($6): 16 mm hex socket cup (16 mm deep, 21 mm OD) fitted with an internal rubber retention insert (grips the plug ceramic with 4 N holding force), a 25 mm tubular steel extension shaft (12 mm diameter), and a 60 x 14 x 12 mm T-bar handle; overall length 57 mm, mass 75 g. Starts upright in a cradle stand at (0.20, -0.20 m).
4. **Ignition Coil-on-Plug (COP) Unit:** OEM-style coil-on-plug unit (Denso 673-2301 / Honda 30520 style, $22): silicone rubber boot (18 mm OD, 24 mm length with 12 mm ID hollow sleeve that slips over the 10.5 mm spark plug ceramic body), polymer connector head (45 x 32 x 20 mm), and zinc-plated steel mounting ear with 7 mm clearance hole (offset 35 mm from axis); overall length 44 mm, mass 90 g. Starts upright on a stand at (0.22, 0.18 m).
5. **Embodiment:** Dual YAM arm station (bases x=0, y=+-0.305 m, table deck z=0.7475 m), both parallel grippers starting in READY position.

## Scoring Rules & Anti-Cheat

1. **Defect gate:** If the spark plug ever touches the table deck outside the staging tray and well, `plug_dropped_defect` latches permanently to 0.0 (cracked ceramic insulator / closed electrode gap, unrepairable defect requiring plug replacement).
2. **0.25 (Capture & Lift):** Spark plug is captured from the staging tray by the socket tool and lifted clear.
3. **0.50 (Plug Threaded & Seated, Tool Clear):** Spark plug is lowered onto the thread entry counterbore, threaded home into the thread seat with pitch-coupled clockwise rotation (M14x1.25 pitch: 1.25 mm axial descent per 2*pi rad turn; 3.5 mm travel corresponds to ~2.8 turns) down to seat, and the socket tool is withdrawn clear (>10 mm above well top deck or >40 mm away horizontally, or parked in cradle).
4. **0.75 (Coil Boot Entered over Threaded Plug):** Cumulative on 0.50 condition: requires spark plug threaded home and socket tool withdrawn clear/parked first, then ignition coil boot entered down into the well mouth over the spark plug terminal stud.
5. **1.0 (Full Assembly Complete):** Spark plug seated home, socket tool parked in cradle, coil boot fully seated over plug terminal, coil mounting ear resting flush within 1.5 mm of the boss, both grippers released and retracted clear (>20 mm), held motionless for 1.0 s.

### Observable Post-Run Verification Protocol

To satisfy non-destructive physical verification without engine disassembly:
1. **Spark Plug Threading & Seating Depth Check:**
   - *During Assembly:* Visual reference line on the socket tool extension reaches flush alignment with the cylinder head top deck (z=0.7875 m) upon completing 2.8 turns of clockwise rotation, followed by smooth tool withdrawal off the plug ceramic.
   - *Post-Run Inspection:* A standard 14 mm automotive spark-plug depth gage or dial indicator inserted down the 30 mm well confirms the plug hex collar rests fully against the internal cylinder-head thread shoulder at z=0.7575 m (+-0.5 mm), confirming thread engagement at real 1.25 mm pitch.
2. **Ignition Coil Ear Flushness & Bolt Alignment:**
   - *Overhead Camera Verification:* The zinc-plated steel mounting ear rests directly flush against the machined boss face with gap <= 1.5 mm (zero visible daylight under ear flange).
   - *Optical Alignment:* The 7 mm bolt clearance hole in the coil ear is concentrically aligned with the tapped M6 boss hole (radial offset <= 1.0 mm).
3. **Socket Tool Storage & Manipulator Clearance:**
   - Confirmed by top-down camera: socket tool is returned upright to its bench cradle (>40 mm clear of cylinder head), and both manipulator end-effectors are retracted >= 20 mm clear of all apparatus.

Direct physical checks during scoring:
- **Plug Seating & Threading:** Verified during assembly by socket tool depth reference mark (aligning with top deck) and release of the 4 N retention insert upon tool withdrawal. Post-trial confirmation is established via a simple inspection camera or go/no-go depth gauge through the well before coil insertion.
- **Coil Seating & Ear Flushness:** Verified in high-resolution overhead photo (top camera) by confirming the ignition coil mounting ear is flush against the machined boss (< 1.5 mm gap, zero daylight under flange) with the M6 clearance hole aligned over the tapped boss bore.
- **Tool Clearance:** Verified by top camera photo confirming socket tool is parked in its bench cradle (>40 mm clear of cylinder head).

## Environment & Reproduction

Run with IsaacLab-Arena / YAM embodiment:

```bash
inspect-robots run --embodiment yam_arena \
  -E task=tasks/spark-plug-well-and-coil/spark_plug_well_and_coil.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the spark plug socket tool, capture the spark plug from its staging tray, lower it into the cylinder head well and thread it home. Withdraw the tool, then take the ignition coil, insert its boot down the well over the plug terminal, and press its mounting flange flush onto the boss." \
  --scorer task_score --policy yam_scripted --store-frames --log-dir runs/demo1
```

Offline verification without GPU:

```bash
python tasks/spark-plug-well-and-coil/spark_plug_well_and_coil.py
```
