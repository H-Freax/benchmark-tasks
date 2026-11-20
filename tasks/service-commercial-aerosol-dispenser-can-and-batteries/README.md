# Service a Commercial Metered Aerosol Air Freshener Dispenser

## Overview

Janitors and building cleaners (**2,432,600 jobs**, BLS Occupational Outlook Handbook, SOC 37-2011) maintain, clean, and restock restroom fixtures across commercial office buildings, hospitals, airports, and hotels. A core routine maintenance operation is servicing wall-mounted automated metered aerosol air freshener dispensers (e.g. Technical Concepts / Rubbermaid Commercial Products Microburst 3000 / AutoClean). 

Over monthly service cycles, maintenance workers must:
1. Depress the top spring-loaded release catch to open the front protective enclosure.
2. Swing the front clamshell housing down 90° on its hinge to expose the aerosol can cradle and electronic actuator hammer.
3. Vertically extract the spent 5.3 oz aerosol can from its internal cradle collar.
4. Set the spent can into a service discard tray.
5. Retrieve a fresh metered refill can from the service cart staging tray.
6. Align and vertically seat the fresh can flush into the cradle collar beneath the overhead actuator hammer.
7. Swing the clamshell cover shut until the top mechanical latch snaps closed.
8. Perform a 10 N outward proof tug on the cover to verify positive latching without accidental opening before clearing both hands.

This task models complete 1:1 commercial restroom hardware with zero miniaturization, real hollow cavity enclosures, pure rigid-body contact physics, and zero artificial grasp aids or proximity welds.

---

## Hardware & Component Dimensions

All dimensions correspond exactly to commercial 1:1 metered restroom dispensers:

| Component | Dimensions / Specifications | Material / Physics Properties |
| :--- | :--- | :--- |
| **Dispenser Enclosure** | 210 mm H × 90 mm W × 85 mm D; 4 mm wall thickness | Rigid chassis with rear wall mount plate, sidewalls, floor, and top actuator block. |
| **Front Clamshell Cover** | 210 mm H × 94 mm W × 50 mm return flange; 3 mm shell; mass 180 g | Revolute joint hinged at bottom front edge (0° to 90° downward travel, damping 0.008 N·m·s/rad). |
| **Top Release Latch** | 25 mm L × 35 mm W × 12 mm H; mass 30 g | Prismatic slider along vertical z-axis (6 mm travel, spring stiffness 1,200 N/m, damping 5.0 N·s/m). |
| **Aerosol Refill Cans** | 65 mm OD × 140 mm H (120 mm cylinder body, 20 mm valve/stem dome); mass 210 g | Standard 5.3 oz / 100 mL commercial metered aerosol refills (spent bronze, fresh green). |
| **Internal Can Cradle** | 68 mm ID semi-circular collar (1.5 mm radial clearance), base shelf at Z = 15 mm | Real hollow cylindrical collar pocket formed by dual rigid boundary walls and floor shelf. |
| **Service Wall Panel** | 500 mm H × 500 mm W × 15 mm D vertical partition at X = 0.435 m | Rigid vertical backing panel simulating commercial tiled restroom wall. |
| **Discard & Staging Trays** | 120 × 120 mm discard tray (Z = 20 mm); 100 × 100 mm staging rest on 70 mm riser (Z = 80 mm) | Tabletop staging fixtures with raised lip rims to catch and locate cans securely. |

---

## Coordinate System & Workspace Layout

Positions are given in meters from the midpoint between the robot arm bases on the table surface ($Z_{\text{bench}} = 0.7475\text{ m}$):
- $X$: Forward from robot arm baseline into the workstation.
- $Y$: Lateral axis ($+Y$ to robot's left, $-Y$ to robot's right).
- $Z$: Vertical height above table surface.

| Element | $X$ (m) | $Y$ (m) | $Z$ (m) |
| :--- | :--- | :--- | :--- |
| **Dispenser Origin (front bottom)** | 0.350 | 0.000 | 0.080 |
| **Cover Hinge Axis** | 0.350 | 0.000 | 0.085 |
| **Top Release Latch** | 0.3925 | 0.000 | 0.290 |
| **Seated Can Cradle Center** | 0.3925 | 0.000 | 0.155 |
| **Discard Tray Center** | 0.300 | 0.180 | 0.020 |
| **Fresh Can Staging Rest** | 0.320 | -0.120 | 0.080 |

---

## Scoring Logic & Progression

The task enforces strict 5-stage monotonic cumulative credit:

- **0.20 — Release Latch Depressed & Cover Opened**:
  The top release latch button is pushed downward $\ge 3\text{ mm}$ against its spring ($> 3.6\text{ N}$), disengaging the cover catch, and the front clamshell cover swings open $\ge 60^\circ$.
- **0.40 — Spent Can Removed to Discard Tray**:
  The spent aerosol can is lifted out of the cradle collar and placed into the discard tray on the left ($XY$ offset $< 50\text{ mm}$, $Z < 100\text{ mm}$ above table).
- **0.65 — Fresh Can Seated in Dispenser Cradle**:
  The fresh aerosol can is retrieved from the staging rest, oriented upright (vertical tilt dot product $> 0.98$), and seated flush into the internal cradle collar ($XY$ coaxial error $< 12\text{ mm}$, vertical seating error $< 10\text{ mm}$).
- **0.85 — Front Cover Swung Shut & Latched**:
  The clamshell cover is swung closed (opening angle $< 3.0^\circ$) until the top mechanical catch engages.
- **1.00 — Verification Proof Tug Passed & Hands Clear**:
  An outward 10 N proof tug is applied to the front cover to verify positive latching without unseating, both robot hands move $\ge 50\text{ mm}$ clear of all dispenser and can bodies, and equilibrium is held for $\ge 1.0\text{ s}$.

---

## Qualification Baseline

- **Demo Policy (`yam_scripted`)**: Score = 1.00 (all 5 stages completed smoothly).
- **Negative Control (No-op / `ctrl_noop`)**: Score = 0.00.
- **Negative Control (Cover open only)**: Score = 0.20.
- **Baseline Policy (`dream`)**: Score = 0.00.
