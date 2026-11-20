# Dock a Cordless Barcode Scanner into its Presentation Cradle

## Summary

In retail point-of-sale environments, pharmacy dispensaries, and warehouse distribution sorting hubs, cashiers, pharmacy technicians, and order pickers dock and undock handheld cordless 2D area imagers into counter-mounted presentation charging cradles. This bimanual operation requires steadying the desktop cradle base to prevent sliding or tipping while guiding the imager's contoured handle and nose downward into the hollow guide throat until securely seated against the bottom charging contacts.

## Occupational & Economic Foundation

- **Occupations**: Cashiers (BLS SOC 41-2011: 3,340,000 workers), Retail Salespersons (BLS SOC 41-2031: 3,760,000 workers), Pharmacy Aides (BLS SOC 31-9095: 50,000 workers), and Stockers and Order Fillers (BLS SOC 53-7065: 2,900,000 workers).
- **Workforce Total**: Over 7,500,000 workers in the United States regularly operate, dock, and charge commercial barcode scanning hardware.
- **Hardware Standard**: Benchmarked to commercial Zebra DS2278 handheld 2D cordless imager specifications and Zebra CR2278-PC presentation charging base (and equivalent Honeywell Voyager 1472g / CCB01 series).

## Physical Apparatus & Dimensions

- **Cordless Barcode Scanner (Zebra DS2278)**:
  - Total height: 175 mm
  - Head depth: 99 mm
  - Width: 66 mm
  - Weight: 214 g (0.214 kg) authentic commercial mass
  - Features: Contoured pistol-grip handle (36 x 32 x 110 mm), recessed optics window bezel, bottom charging contacts with retention detent notch.
- **Desktop Presentation Cradle (Zebra CR2278-PC)**:
  - Base footprint: 150 mm length x 100 mm width x 85 mm height
  - Weight: 380 g (0.380 kg) cast-iron ballasted base with non-skid rubber feet
  - Features: Contoured hollow receiver throat with 15 mm side guide cheeks, back support cup, front retaining lip, and bottom stop with charging pins.
- **Starting Setup**:
  - Presentation cradle sits on the checkout counter at `x = 0.36 m, y = -0.05 m`.
  - Scanner starts upright on counter at `x = 0.30 m, y = -0.10 m, z = 0.055 m`.

## Scoring Logic & Success Criteria

Scoring evaluates multi-stage contact progression:
- `Stage 0.25`: Left arm steadies cradle base (< 150 mm) while right arm lifts scanner (> 15 mm).
- `Stage 0.50`: Scanner aligned and guided above cradle throat (< 60 mm from seated target) while cradle remains stable (< 40 mm translation, > 0.90 vertical alignment).
- `Stage 0.75`: Scanner seated into cradle socket (< 25 mm from bottom stop, < 15° tilt).
- `Stage 1.00`: Both grippers release clear (> 40 mm) and scanner remains seated in equilibrium for >= 1.0 s (20 consecutive control steps at 20 Hz).

## Empirical Validation

- **Scripted Demo 1**: `1.0` (~230 steps, max IK error 0.50 mm)
- **Scripted Demo 2**: `1.0` (~230 steps, max IK error 0.50 mm)
- **Negative Control (unsteadied tip)**: `0.0` (cradle shifts/tips when unsteadied)
- **Negative Control (unreleased)**: `0.75` (gripper held clamped; fails hands-free dwell)
- **Dream-1 (1200 steps)**: `0.0`
