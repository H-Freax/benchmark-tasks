# Reassemble Commercial Citrus Juicer

## Overview

In commercial kitchens, juice bars, hotel pantries, and high-volume cocktail lounges, heavy-duty commercial manual citrus presses (such as the Hamilton Beach Commercial 932) are the primary equipment for fresh citrus extraction. Under FDA Food Code §4-602.11 and NSF/ANSI sanitation standards, all food-contact parts must be completely disassembled, sanitized, and reassembled between operational shifts to eliminate bacterial accumulation.

This task evaluates bimanual robotic manipulation for precision multi-mechanism reassembly of a commercial citrus press:
1. Counter-torque bracing of the heavy cast iron base frame.
2. Alignment and insertion of the stainless steel juice collector funnel into the 110 mm receiver collar, followed by a 30° clockwise rotation to engage the three bayonet locking tabs into internal base detents.
3. Coaxial insertion and flush seating of the perforated stainless steel reamer strainer cone onto the collector bowl's internal annular shelf.
4. Clearance test stroke actuation: depressing the sprung rack-and-pinion press lever downward >= 12° to verify that the upper compressor cup plunges cleanly into the seated strainer cone without collision, and allowing the lever to return to its rest stop.
5. Retracting both manipulator arms clear (>= 25 mm clearance) and maintaining stationary stability for 1.0 second.

## Occupational Relevance

- **Primary SOC**: `35-2021 Food Preparation Workers` (1,024,400 US jobs, BLS OEWS May 2023).
- **Secondary SOC**: `35-3023 Fast Food and Counter Workers` (3,546,800 US jobs, BLS OEWS May 2023).
- **Total Workforce**: Over 4,500,000 US workers regularly operate, sanitize, and maintain commercial food and beverage preparation machinery.
- **Physical Difficulty**: Coordinated bimanual manipulation requiring axial alignment, bayonet rotational locking, concentric conical nesting with sub-centimeter tolerances, and kinematically constrained rack-and-pinion lever actuation.

## Mechanical Components

1. **Cast Iron Base Frame (`juicer_base`)**:
   - Heavy cast horseshoe base (240 x 200 x 30 mm) resting securely on table surface.
   - Upright column spine (60 x 60 x 320 mm) providing vertical mounting for rack housing and collar arm.
   - Receiver collar ring with 110 mm internal opening and 3 internal bayonet lug recesses spaced 120° apart.
2. **Juice Collector Bowl (`collector_bowl`)**:
   - Stamped 304 food-grade stainless steel funnel (outer diameter 104 mm, height 65 mm, mass 0.34 kg).
   - Three bayonet locking tabs on upper rim (116 mm diameter).
   - Forward pouring spout and internal 92 mm diameter annular support ledge.
3. **Perforated Strainer Cone Insert (`strainer_cone`)**:
   - Machined 304 stainless steel conical reamer insert with seed slots (mass 0.19 kg).
   - 90 mm diameter perimeter seating flange matching the internal bowl ledge.
4. **Sprung Press Lever & Plunger (`press_lever`)**:
   - Chrome-plated cast steel lever bar with top ergonomic grip knob.
   - Upper stainless compressor cone cup (80 mm diameter).
   - Revolute hinge pivot with spring return (stiffness 1.5 N·m/rad, damping 0.4 N·m·s/rad, limits -35° to 0°).

## Scoring Breakdown

- **0.25**: Collector bowl placed within receiver collar opening (radial horizontal offset <= 20 mm, vertical offset <= 15 mm).
- **0.50**: Collector bowl bayonet rotated clockwise >= 15° into locking detents.
- **0.75**: Strainer cone coaxially nested flush inside collector bowl (radial offset <= 12 mm, vertical gap <= 10 mm).
- **1.00**: Press lever stroked downward >= 12° to verify stroke clearance, returned to top rest stop, both arms cleared >= 25 mm, and held stationary for 1.0 s (20 consecutive control steps at 20 Hz).
- **0.00 (Fail)**: Base tipped or displaced > 20 mm, or components dropped off bench.
