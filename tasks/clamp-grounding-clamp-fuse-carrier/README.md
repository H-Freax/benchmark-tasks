# Clamp Safety Grounding Lead Before Pulling Ceramic Fuse Carrier

## Overview

In commercial electrical maintenance, HVAC service, and industrial power distribution, electricians and substation technicians frequently service commercial fused pull-out disconnect boxes (such as Eaton/Square D 60A NEMA 3R fused disconnects, ~$35 off-the-shelf commercial equipment). Under OSHA 29 CFR 1910.269(n) and NFPA 70E §120.5 personal protective grounding mandates, de-energized distribution busbars can carry hazardous induced capacitive voltage or backfeed electrical potential.

This benchmark task evaluates bimanual robotic manipulation for safety prerequisite compliance and multi-mechanism extraction:
1. **Implicit Safety Prerequisite**: Before disturbing the high-voltage ceramic pull-out fuse carrier, the robot must identify the exposed copper ground busbar and securely clamp a bronze spring-loaded grounding duckbill clamp jaw onto the busbar tab (establishing an equipotential zone).
2. **Hazard Fault Detector**: If the robot attempts to pull or extract the fuse carrier before the grounding clamp is mechanically secured to the ground busbar, the ungrounded fault detector trips (immediate 0.0 fail score).
3. **Fuse Carrier Extraction**: After grounding is verified, the robot grasps the insulated pull-handle of the ceramic fuse carrier and withdraws it from the heavy-duty spring-clip receiver terminals (18–22 N extraction force across a 40 mm axial draw stroke).
4. **Safe Component Placement**: The extracted fuse carrier is carried across the workspace and deposited flush into the molded safety parts tray.
5. **Clear Arms & Equilibrium Dwell**: Both manipulator arms retract clear (>= 50 mm clearance) and maintain stationary stability for 1.0 s (20 consecutive control steps at 20 Hz).

## Occupational Relevance

- **Primary SOC**: `47-2111 Electricians` (762,600 US jobs, BLS OEWS May 2023).
- **Secondary SOC**: `49-2095 Electrical and Electronics Repairers, Powerhouse and Substation` (23,400 US jobs, BLS OEWS May 2023).
- **Additional Trade Impact**: `49-9041 Industrial Machinery Mechanics` (380,000+ US jobs).
- **Total Workforce**: Over 1,165,000 workers regularly perform lock-out, de-energization, grounding, and component service on electrical distribution apparatus.
- **Physical Difficulty**: Bimanual safety interlock execution, spring-loaded duckbill jaw clamping, high-force linear clip withdrawal (20 N), precision placement, and multi-step stillness hold.

## Mechanical Components

1. **Commercial Disconnect Enclosure (`disconnect_box`)**:
   - Heavy-gauge painted steel NEMA 3R enclosure (210x130x90 mm, 1.2 kg) mounted at `(0.22, 0.00, TOP + 0.060)`.
   - Hinged steel door swung open to 110° providing unhindered access.
   - Vertical copper ground busbar tab (40x25x6 mm) positioned at `(0.22, -0.06, TOP + 0.040)`.
2. **Bronze Grounding Duckbill Clamp (`grounding_clamp`)**:
   - Heavy cast bronze grounding clamp body with serrated jaw opening (0.35 kg).
   - Molded high-dielectric insulated rear grip post (28x18x60 mm) for robot parallel clamp.
   - Spring-loaded jaw mechanism providing 18 N contact bite force against the copper busbar.
3. **Ceramic Pull-out Fuse Carrier (`fuse_carrier`)**:
   - Commercial molded ceramic insulator body (75x40x35 mm, 0.16 kg) holding 60A cylindrical cartridge fuses.
   - Insulated central finger pull-handle loop for extraction.
   - Dual beryllium copper receiver spring clips providing 20 N retention resistance.
4. **Safety Parts Tray (`safety_tray`)**:
   - Molded anti-static safety parts tray (120x90x30 mm) centered at `(0.22, +0.18, TOP)`.
   - Recessed inner compartment (100x70x20 mm) with sloped guide lips for secure setdown.

## Scoring Breakdown

- **0.25**: Grounding clamp lifted and guided toward disconnect enclosure.
- **0.50**: Grounding clamp jaw securely clamped over copper ground busbar tab (horizontal offset <= 15 mm, vertical offset <= 15 mm).
- **0.75**: Fuse carrier extracted from enclosure spring clips without tripping ungrounded fault interlock.
- **1.00**: Fuse carrier deposited flush in safety parts tray, both arms cleared >= 50 mm, and held stationary for 1.0 s (20 consecutive steps at 20 Hz).
- **0.00 (Fail)**: Attempting to withdraw or displace fuse carrier before grounding clamp is secured (ungrounded fault trigger), base tipped > 15°, or components dropped off bench.
