### Description
Adds `tasks/clamp-standard-microplate-in-orbital-shaker-platform/`: Transfer an ANSI/SLAS 96-well microplate into a benchtop orbital shaker nest, advance the spring retaining clamp bracket across the plate rim, and toggle the front power switch before parking clear.

In wet-lab biological, pharmaceutical, and clinical diagnostic laboratories, benchtop orbital shakers agitate microtiter plates to mix reagents, incubate bacterial or mammalian cell cultures, and run ELISA protocols. Operating an orbital shaker with an unsecured microplate causes immediate centrifugal plate ejection, shattering the vessel and dispersing biohazardous fluids across the laboratory.

### Workforce / Domain Grounding (Category: "Other" - Wet-Lab Research Automation)
- **Relevant Occupations**: Medical & Clinical Laboratory Technologists & Technicians (BLS SOC 29-2010, >340,000 workers) + Biological Technicians (BLS SOC 19-4021, >85,000 workers) + Chemical Technicians (BLS SOC 19-4031, >65,000 workers) + Life Scientists & Microbiologists (BLS SOC 19-1020, >55,000 workers) -> **>545,000 laboratory research & diagnostic personnel**.
- **Equipment Practicality**: Standard off-the-shelf benchtop orbital shaker platform (<$200) and universal ANSI/SLAS standard 96-well microplates ($1-$3/unit).

### Physical Mechanics & Anti-Cheat Measures
- **Dynamic Physics Only**: 100% genuine dynamic contact bodies via PhysX USD joints (0 `write_root_pose_to_sim` or pose overwriting).
- **Prismatic Slider Retention**: Spring-loaded retaining clamp bracket (`stiffness=100.0`, `damping=5.0`, `friction=0.5`) requiring deliberate physical mechanical displacement.
- **Rocker Power Switch**: Revolute joint on the front equipment panel requiring positive tactile actuation.
- **Causal Defect Latch**: Starting the orbital shaker before the plate is seated and clamped locks the episode score permanently to `0.00`.
- **Dwell Requirement**: Continuous 20-step (1.0 s) stillness hold with both robot grippers retracted > 50 mm clear.

### Empirical Validation & Benchmark Scores
- `demo1` (Seed 0): **1.00**
- `demo2` (Seed 1): **1.00**
- `ctrl_no_seat`: **0.15** (plate lifted, not seated)
- `ctrl_no_clamp`: **0.35** (plate seated, clamp not engaged)
- `ctrl_no_release`: **0.75** (plate clamped & switch ON, grippers not retracted)
- `dream1`: **0.00** (untrained baseline)
