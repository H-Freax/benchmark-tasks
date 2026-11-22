# Seat floating microtube foam rack in benchtop ultrasonic water bath and toggle DEGAS

## Task Overview
- **Category**: Other (Wet-Lab Research Automation / Life Sciences Quality Control)
- **Workforce / Target Domain**: BLS SOC 19-4021 (Biological Technicians, >82k), 19-4031 (Chemical Technicians, >67k), 19-4010 (Agricultural & Food Science Technicians, >34k), 29-2010 (Clinical Laboratory Technologists & Technicians, >342k), 19-1021 (Biochemists & Biophysicists, >34k) — **>559,000 professional laboratory and life-sciences personnel**.
- **Instruction**: Transfer the floating microtube foam rack into the ultrasonic bath tank, close the hinged acrylic lid flush, and depress DEGAS.

## Setup & Apparatus
- **Benchtop Ultrasonic Bath**:
  - 230 x 190 x 80 mm brushed stainless steel chassis positioned at `(X=0.250, Y=-0.060, Z=0.748)` m on the table surface.
  - 150 x 120 x 45 mm internal fluid tank cavity centered at `(X=0.250, Y=-0.060, Z=0.783)` m filled with 650 mL degassed sonication water bath fluid (fluid resting deck at Z = 0.828 m).
  - Hinged transparent acrylic acoustic safety lid propped open at 70.0 degrees at rear pivot `(0.165, -0.060, 0.834)` m with counterbalance spring (`stiffness=0.10 N*m/rad`, Coulomb friction `0.08 N*m`).
  - Tactile membrane DEGAS pushbutton on front console at `(0.405, -0.045, 0.783)` m with return spring (`stiffness=20.0 N/m`, damping `1.0 N*s/m`). Stroke limit: 8.0 mm; electrical contact threshold: `>= 3.0 mm`.
- **Floating Microtube Foam Rack**:
  - Buoyant closed-cell EVA foam slab (`80 x 56 x 30` mm, mass 0.09 kg) holding 8 microcentrifuge tubes in a 2x4 array staged at `(X=0.250, Y=0.180, Z=0.763)` m.

## Scoring Progression
1. **0.25**: Foam rack lifted and transferred inside bath chamber perimeter (`|dx| <= 38 mm`, `|dy| <= 35 mm`, `Z < 0.843 m`).
2. **0.50**: Foam rack fully seated level inside tank well (`|dz| <= 12 mm` from fluid resting plane `0.828 m`, tilt `<= 8.0 deg`).
3. **0.75**: Acrylic lid closed flush (`angle <= 14.0 deg`) AND DEGAS membrane button depressed inward `>= 3.0 mm`.
4. **1.00**: DEGAS cycle triggered, rack seated level right now, lid closed flush right now, both robotic arms retracted clear (`>= 50 mm`) for `>= 20` control steps (1.0 s at 20 Hz).

### Defect Latch
- Tilting rack `> 15.0 deg` during handling or transfer causes sample well flooding / liquid spill, permanently latching score to `0.00`.
- Depressing DEGAS without rack seated level in tank triggers dry-run cavitation defect, locking score permanently to `0.00`.
