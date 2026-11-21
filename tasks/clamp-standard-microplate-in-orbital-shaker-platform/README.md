# Seat Microplate in Orbital Shaker Nest and Lock Retaining Clamp

## Overview
In wet-lab biological, pharmaceutical, and clinical diagnostic laboratories, high-speed benchtop orbital shakers agitate microplates (such as standard ANSI/SLAS 96-well microtiter plates) to mix reagents, incubate bacterial or mammalian cell cultures, or perform ELISA assays. 

Before powering on the shaker, technicians must seat the microplate flush within the platform's recessed nest against the corner registration stops, advance the spring-loaded retaining bracket to firmly clamp the plate rim, and then toggle the front power switch. Initiating orbital agitation with an unclamped plate leads to rapid centrifugal ejection, shattering the plate and scattering biohazardous or chemical reagents across the laboratory.

```
       [Staging Pad]                             [Orbital Shaker]
 +-----------------------+              +--------------------------------+
 |  +-----------------+  |   Transfer   |  +--------------------------+  |
 |  | 96-Well Plate   |  | ===========> |  | Corner Registration Nest |  |
 |  | (128 x 86 mm)   |  |  (Left Arm)  |  +--------------------------+  |
 |  +-----------------+  |              |        [Retaining Clamp]       |
 +-----------------------+              |   <--- (Right Arm Pulls)       |
                                        |                                |
                                        |  [Power]                       |
                                        |  (Toggle)                      |
                                        +--------------------------------+
```

## Physical Apparatus & Objects
1. **ANSI/SLAS Standard 96-Well Microplate**:
   - Footprint: 127.8 mm x 85.5 mm x 14.3 mm, 45 g dynamic body.
   - Initial position: Staged on a silicone benchtop pad at `(X=0.280, Y=0.180, Z=0.762)`.
2. **Benchtop Orbital Shaker Platform**:
   - Chassis: 220 mm x 200 mm x 50 mm off-white laboratory equipment enclosure.
   - Recessed Nest: 128 mm x 86 mm x 8 mm recess with raised rear and side registration stops centered at `(X=0.280, Y=-0.060)`.
3. **Spring-Loaded Retaining Clamp Bracket**:
   - Anodized red safety bracket mounted on a prismatic slider (`axis=(0, 1, 0)`).
   - Features spring-return behavior (`stiffness=2000.0 N/m`, `damping=30.0`, `friction=2.0 N`) requiring deliberate gripper retention or positive detent travel.
4. **Front Panel Power Rocker Switch**:
   - Revolute switch (`axis=(0, 1, 0)`) with spring return and detent engagement at 25 degrees.

## Task Choreography & Scoring
- **0.15**: Microplate lifted clear of the staging pad (`> 25 mm` Z rise or `> 40 mm` lateral displacement).
- **0.35**: Microplate transferred and seated flush in the recessed orbital shaker nest (`<= 30 mm` XY and `<= 15 mm` Z).
- **0.50**: Spring retaining clamp bracket advanced and locked across the plate rim (stroke `>= 12 mm`).
- **0.75**: Front panel power switch toggled while plate is securely clamped (`>= 15 deg` tilt).
- **1.00**: Shaker started, both grippers retracted `> 50 mm` clear, and apparatus held in a continuous 20-step (1.0 s) stillness dwell.

### Empirical Benchmark Scores
| Condition | Task Score | Description |
|---|---|---|
| `demo1` (Seed 0) | **1.00** | Full bimanual transfer, clamp lock, switch toggle, retraction & dwell |
| `demo2` (Seed 1) | **1.00** | Seed variation generalization |
| `ctrl_no_seat` | **0.15** | Control ablation: plate suspended, not seated |
| `ctrl_no_clamp` | **0.35** | Control ablation: plate seated, clamp not advanced |
| `ctrl_no_release` | **0.75** | Control ablation: clamped & switch toggled, grippers dwell on switch |
| `dream1` | **0.00** | Zero-shot untrained policy baseline |

## Run a Trial
```bash
# Scripted Baseline Demonstration (1.00)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clamp-standard-microplate-in-orbital-shaker-platform/clamp_standard_microplate_in_orbital_shaker_platform.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score --max-steps 300 --store-frames --log-dir ~/yam-eval/microplate-demo
```
