# spark-plug-well-and-coil

Install a spark plug into a recessed cylinder-head well using a specialized spark-plug socket tool, thread it home, withdraw the tool, and seat the ignition coil-on-plug (COP) unit over the plug terminal flush against the mounting boss.

## Task Details

- **Title:** Install a spark plug in a recessed cylinder-head well with a socket tool and seat its ignition coil
- **Category:** Impact on jobs
- **Keywords:** `spark plug`, `ignition coil`, `cylinder head`, `socket tool`, `automotive service`, `bimanual`, `recessed well`
- **Instruction:** Take the spark plug socket tool, capture the spark plug from its staging tray, lower it into the cylinder head well and thread it home. Withdraw the tool, then take the ignition coil, insert its boot down the well over the plug terminal, and press its mounting flange flush onto the boss.
- **Run Limit:** 2,400 steps (120 s at 20 Hz)

## Occupational Relevance

Automotive service technicians and mechanics (SOC 49-3023; 760,000+ US jobs, BLS OEWS May 2023) perform spark plug and ignition coil replacements during routine tune-ups and diagnostic misfire remediation. In modern DOHC engines, spark plugs sit recessed at the base of deep narrow cylinder head wells (typically 100-150 mm deep). Specialized spark plug sockets with internal rubber retention sleeves prevent dropping the spark plug into the well (which fractures brittle ceramic insulators or bends the preset ground electrode gap). The technician threads the plug into the cylinder head threads, draws the socket tool off the seated plug, guides the flexible rubber coil boot down over the terminal stud, and presses the coil mounting flange flush against the tapped boss.

## Scoring Rules

1. **Defect gate:** A drop from >=10 mm or collision above 0.5 m/s latches a permanent 0.0 score (cracked ceramic insulator / deformed electrode gap).
2. **0.25 (Capture & Lift):** Spark plug captured from staging tray by socket tool and lifted clear.
3. **0.50 (Plug Seated Home & Tool Clear):** Spark plug seated on the thread shelf at the well bottom (radial error <=3 mm, axial error <=3 mm) and socket tool withdrawn clear (>25 mm above well mouth or >40 mm away).
4. **0.75 (Coil Boot Entered):** Ignition coil boot entered into the well mouth over the plug.
5. **1.0 (Full Assembly Complete):** Spark plug seated home, socket tool withdrawn/parked, coil boot seated over plug terminal, mounting ear resting flush within 1.5 mm of the boss, both grippers released and clear (>20 mm), held motionless for 1.0 s.

## Environment & Reproduction

Run with IsaacLab-Arena / YAM embodiment:

```bash
inspect-robots run --embodiment yam_arena \
  -E task=tasks/spark-plug-well-and-coil/spark_plug_well_and_coil.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the spark plug socket tool, capture the spark plug from its staging tray, lower it into the cylinder head well and thread it home. Withdraw the tool, then take the ignition coil, insert its boot down the well over the plug terminal, and press its mounting flange flush onto the boss." \
  --scorer task_score --policy yam_scripted
```

Offline verification without GPU:

```bash
python tasks/spark-plug-well-and-coil/spark_plug_well_and_coil.py
```
