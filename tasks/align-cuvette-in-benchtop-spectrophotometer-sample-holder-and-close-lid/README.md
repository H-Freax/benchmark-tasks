# Align Cuvette in Benchtop Spectrophotometer Sample Holder and Close Lid

## Task Overview
A bimanual robot performs standard wet-lab research preparation on a benchtop UV-Vis spectrophotometer:
1. Grasps a standard 12.5 x 12.5 x 45.0 mm optical cuvette strictly by its upper frosted handling ribs (never touching the clear optical faces).
2. Lifts the cuvette clear of the countertop staging rack and transfers it across to the spectrophotometer's recessed sample compartment.
3. Inserts the cuvette squarely into the sample socket with its polished clear optical windows oriented along the instrument's optical light-path axis (+/- Y axis).
4. Closes the light-tight hinged sample-compartment lid flush (<= 8 degrees) to seal out ambient stray light.
5. Retracts both manipulator arms clear (> 40 mm) and dwells in stationary equilibrium for 20 simulation steps (1.0 s at 20 Hz).

## Occupational Relevance
- **Domain**: Other (Wet-Lab Research Automation)
- **Primary Occupations**:
  - Chemical Technicians (BLS SOC 19-4031, >62,000 workers)
  - Biological Technicians (BLS SOC 19-4021, >83,000 workers)
  - Clinical Laboratory Technologists and Technicians (BLS SOC 29-2010, >342,000 workers)
- Total relevant laboratory workforce: **>487,000 workers**.

## Physical Apparatus & Dimensions
- **Benchtop Spectrophotometer (`spectrophotometer`)**: Commercial compact UV-Vis spectrophotometer housing (240 x 200 x 110 mm) with sloped front user-interface bezel, LCD screen, and recessed sample compartment pocket (0.220, -0.040, 0.825).
- **Hinged Compartment Lid (`sample_lid`)**: 65 x 65 x 10 mm molded lid with front lip handle, hinged on rear edge along the world Y axis (revolute joint, 0 to 75 deg, starts open at 65 deg).
- **Standard Optical Cuvette (`optical_cuvette`)**: 12.5 x 12.5 x 45.0 mm standard 10 mm pathlength cuvette (mass 14 g). Polished optical windows on opposing +/- Y faces; ribbed frosted handling surfaces on opposing +/- X faces.
- **Staging Rack (`cuvette_rack`)**: Polypropylene cuvette staging caddy at (0.200, 0.180, 0.765).

## Scoring Rubric (100% Live Instantaneous Physical State)
Scoring evaluates the instantaneous physical state at the current simulation step (zero permanent positive latching):
- **0.00**: Initial state or cuvette unseated.
- **0.25**: Cuvette lifted clear of staging rack.
- **0.50**: Cuvette seated squarely and upright (< 25 deg tilt) inside spectrophotometer sample socket.
- **0.75**: Cuvette seated AND hinged sample lid closed flush (<= 8 deg).
- **1.00**: All completion criteria met continuously with hands retracted clear (> 40 mm) for 20 consecutive steps (1.0 s).
