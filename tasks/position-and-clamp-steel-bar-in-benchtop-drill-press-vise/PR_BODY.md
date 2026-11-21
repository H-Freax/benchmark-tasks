# Position and Clamp Steel Workpiece Bar in Benchtop Drill Press Vise onto Parallels

## Summary
Implements bimanual mechanical fixturing for precision machining operations (BLS SOC 51-4041 Machinists, SOC 51-4111 Tool and Die Makers, >1.33M workforce).

The robot must transfer a rectangular high-carbon steel workpiece bar (100 x 28 x 25 mm, 0.55 kg) from a stock staging rack, seat it squarely onto two precision ground steel parallels inside a 4-inch benchtop drill press vise, register it against the fixed jaw and lateral end stop, and advance the vise screw handle to drive the moving jaw into high-load mechanical clamping before retracting both arms for a 1.0 s stationary dwell.

## Occupational Relevance
- **Primary SOC**: `51-4041 Machinists` (>340,000 US workers).
- **Secondary SOC**: `51-4111 Tool and Die Makers` (>50,000 US workers).
- **Tertiary SOC**: `49-9041 Industrial Machinery Mechanics` (>380,000 US workers).
- **Total Workforce**: Over 1,330,000 commercial manufacturing tradespeople.
- **Physical Manipulation**: Bimanual stock transfer, planar registration on parallels, end-stop datum alignment, lead screw mechanical clamping, and dual-arm clearance dwell.

## Validation Suite & Qualification Scores
- **demo_1** (yam_scripted, seed 0): **1.00**
- **demo_2** (yam_scripted, seed 1): **1.00**
- **neg_no_seat** (seed 0, `DEMO_VARIANT=no_seat`): **0.20**
- **neg_no_clamp** (seed 0, `DEMO_VARIANT=no_clamp`): **0.50**
- **neg_no_release** (seed 0, `DEMO_VARIANT=no_release`): **0.85**
- **dream_1** (policy=dream, 320 steps, seed 0): **0.00**

## Mechanical Setup & Coordinate Architecture
- **Benchtop Drill Press Vise Base (`vise_base`)**: Heavy cast iron body (180x120x25 mm base) at `(0.280, 0.020, 0.7475)` with fixed jaw plate and lateral workpiece end stop.
- **Precision Parallels**: Dual ground steel parallels (90x8x16 mm) resting flat on vise bed.
- **Moving Jaw Slider (`moving_jaw`)**: Prismatic slider (100x20x35 mm, 0.85 kg) initialized open at `x = 0.250 m`.
- **Vise Screw Crank Handle (`vise_handle`)**: Dual-wing brass-tipped crank at `(0.190, 0.020, 0.7675)`.
- **Workpiece Staging Rack (`staging_rack`)**: Stock rack at `(0.240, 0.220, 0.7475)`.
- **Workpiece Steel Bar (`workpiece_bar`)**: 0.55 kg rectangular steel bar (100x28x25 mm) with layout line.
