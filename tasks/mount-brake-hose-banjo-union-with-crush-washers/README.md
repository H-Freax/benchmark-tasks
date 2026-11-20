# Mount an Automotive Brake Hose Banjo Union with Copper Crush Washers and Thread Hollow Bolt into Caliper Bore

## Summary
An automotive brake maintenance and chassis hydraulic assembly procedure compliant with SAE J1401 and DOT standards. The robot mounts a flexible hydraulic brake line eyelet fitting onto a disc brake caliper casting using two copper crush washers (DIN 7603A / SAE standard) and an M10 hollow banjo bolt.

1. **Left Arm** approaches the staging cradle, grasps the reinforced collar of the brake hose banjo fitting (20 mm OD eyelet block, 10.2 mm bore, 10 mm thickness), lifts it, and positions the eyelet concentrically over the caliper's threaded fluid inlet port boss (resting on the lower pre-seated 1.5 mm copper crush washer).
2. **Right Arm** grasps the M10 hollow hex-head banjo bolt (fitted with the captive upper copper crush washer under its flange) from the bolt caddy, lifts it, aligns it with the banjo eyelet, and guides the bolt downward through the eyelet into the caliper threaded bore.
3. **Right Arm** threads the M10 bolt clockwise home into the caliper port boss against the upper crush washer.
4. **Left Arm** holds the brake hose against rotation and applies an axial retention check along the line.
5. **Both Arms** release their grasps and retract clear to allow an automated 10 N axial retention proof pull verifying fluid-tight mechanical retention without line detachment or slip.

## Target Audience & Occupational Relevance
- **Automotive Service Technicians and Mechanics** (SOC 49-3023): >760,000 US workers.
- **Heavy Vehicle and Mobile Equipment Service Technicians** (SOC 49-3042): >210,000 US workers.
Totaling approximately 1,000,000 US workers. Brake hose replacement and caliper union fastening is a universal, critical safety procedure performed daily across automotive repair facilities, dealerships, and fleet maintenance hubs.

## Apparatus & Geometry
- **Tabletop Workbench**: Surface at z = 0.7475 m (station scene).
- **Caliper Fixture Plate**: Rigid steel fixture plate (140 x 160 x 25 mm) bolted to the workbench at (0.280, 0.000, 0.760 m).
- **Disc Brake Caliper Casting**: True-scale single-piston cast-iron automotive caliper body with machined fluid inlet boss (height 40 mm, z = 0.8125 m) and M10x1.0 internal threaded port.
- **Lower Sealing Washer**: Annealed copper crush washer (DIN 7603A / SAE standard, 10.2 mm ID, 15.0 mm OD, 1.5 mm thickness) pre-seated on the port boss (z = 0.81325 m).
- **Brake Hose Banjo Eyelet Assembly**: Reinforced automotive hydraulic line (SAE J1401) terminating in a steel banjo eyelet block (OD 20 mm, ID 10.2 mm, axial height 10 mm) and crimped collar, mass 85 g. Starts staged in a support cradle at (0.280, 0.090, 0.7725 m).
- **M10 Banjo Bolt**: High-tensile steel hollow fluid bolt (14 mm hex head, 8 mm head height, M10 hollow shank 18 mm length, mass 45 g) fitted with an upper 1.5 mm copper crush washer captive under the hex flange. Starts in a bolt caddy at (0.280, -0.090, 0.7725 m).

## Scoring Criteria
Multi-stage cumulative progression with zero proximity shortcuts:
- **0.25**: Banjo fitting picked up and positioned concentrically over the caliper inlet port boss (radial error <= 10 mm, z error <= 15 mm, tilt <= 12 deg).
- **0.50**: Banjo seated flat on the lower crush washer and M10 banjo bolt inserted through the eyelet into the caliper bore (radial error <= 6 mm, z error <= 12 mm).
- **0.75**: Bolt fully threaded home into the caliper boss against the upper crush washer (radial error <= 2.5 mm, z error <= 2.5 mm, tilt <= 12 deg).
- **1.00**: Both robot grippers released and retracted clear (> 25 mm margin) and assembly passes a 10 N axial retention proof pull along the brake hose line (< 2.0 mm slip).
- **Failure Latch (0.00)**: Any component dropped below the table surface (z < 0.7275 m) or severe angular cross-threading tilt (> 25 deg).
