# Position and Clamp Steel Workpiece Bar in Benchtop Rapid-Action Sliding-Jaw Drill Press Vise onto Parallels

## Occupational Background
- **BLS SOC 51-4041**: Machinists (>340,000 workers)
- **BLS SOC 51-4111**: Tool and Die Makers (>50,000 workers)
- **BLS SOC 49-9041**: Industrial Machinery Mechanics (>380,000 workers)
- **BLS SOC 51-4121**: Welders, Cutters, Solderers, and Brazers (>430,000 workers)
- **BLS SOC 47-2211**: Sheet Metal Workers (>130,000 workers)
- **Total Relevant Workforce**: >1,330,000 workers in the United States alone.

## Physical Machine Apparatus
Industrial metalworking benchtop drill press vise apparatus (180 x 120 mm cast iron base slab, 25 mm height above station table) fitted with:
1. Precision ground fixed jaw block with hardened steel jaw face plate (100 mm width, 35 mm height) and lateral workpiece end stop.
2. Two precision hardened ground steel parallels (90 x 8 x 16 mm) resting flat inside the vise bed channel.
3. Prismatic moving jaw slider (100 mm width, 35 mm height, 0.85 kg) with hardened steel face plate, stroked by an integrated rapid-action push-lever sliding transmission.
4. Heavy rapid-action push-lever clamp handle rigidly bonded to the moving jaw at the front (-X) of the vise body.
5. Workpiece staging rack on the +Y table flank with locating side rails holding a raw rectangular steel workpiece bar (100 x 28 x 25 mm, 0.55 kg).

## Task Execution & Alignment Requirements
The operator must transfer the steel workpiece bar from the stock rack, seat it squarely down onto the precision parallels, register it against the fixed jaw and end stop, push the push-lever clamp handle forward to advance the moving jaw into high-pressure mechanical clamp engagement, and retract both hands for a 1.0 s stillness hold.

## Benchmark Scoring Breakdown
- **0.00**: Initial state or dropped off bench.
- **0.20**: Steel workpiece bar lifted clear of staging rack (z >= 0.805 m).
- **0.50**: Workpiece bar seated flush on precision parallels inside the vise channel (dx <= 5 mm, dy <= 8 mm, dz <= 4 mm, tilt <= 3.0 deg).
- **0.85**: Push-lever clamp handle pushed forward and moving jaw clamped tightly against the workpiece bar (effective stroke >= 16.5 mm (moving jaw face contacts 28 mm bar at 17.0 mm stroke)).
- **1.00**: Both robot arms retracted clear (TCP >= 60 mm, collision hull >= 20 mm) for a sustained 20-step (1.0 s at 20 Hz) stillness hold.
