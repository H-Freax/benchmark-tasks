# Primary human scoring: photos and ordinary gauges

Use three fixed cameras or phones, a tape or ruler, a spirit level or angle gauge, a 5 mm feeler/spacer and a 50 mm clearance gauge. This is the primary proposed method for a physical trial. It does not require reconstructing the scene in 3D, tracing software rays, collecting robot meshes or recovering robot forward kinematics. The simulation retains its full geometric checks. No physical measurement accuracy or completed physical trial is claimed here.

## One shared window

Judge the same final 21 original observations, 0.05 seconds apart, spanning 1.000 second, within the 1,200-control limit. Preserve all original frames and exposure times. Record beyond both ends. Establish correspondence between the three recordings with a common visible clock or light signal; nominal frame rate alone does not establish synchronization. If the required samples or timing cannot be established, record indeterminate. Do not interpolate or repeat images to create a missing observation.

Before the trial, use the floor template and rulers to fix the three camera viewing centres at the specified sight positions: (0,-0.30,1.10), (0,0,1.10) and (0,+0.30,1.10) metres. Reproduce the three amber76×76mm square viewing outlines with60×60mm clear apertures in the x=0 plane, using `geometry/OBSERVATION-MARKS.json`. Place each camera optical centre at its corresponding outline centre, not at an outline edge or on the target board. These fixed context marks identify exactly the scored sight points; they do not support the screen. Keep lens, focus and camera positions fixed. Take an unobstructed reference picture of the target before placing the screen, so every one of its 189 dot centres can be identified in each view. The target board, frame scale and camera-position readings must let the judge bound ambiguity at a screen edge. If a centre cannot be located reliably, mark it uncertain; do not guess its position.

## Read coverage directly in the three pictures

For each of the 21 instants, inspect the 189 registered target-dot centres in each camera picture. Record each centre as:

- **Blocked:** the opaque screen is certainly between that camera and that dot centre.
- **Visible:** that centre is certainly visible past the screen.
- **Uncertain:** an edge, blur, glare, image resolution, timing or viewing-position uncertainty prevents a decision.

Count the centre, not whether the whole 6 mm dot disappears. A partly visible dot may have its centre hidden, or the reverse. The screen must cause the obstruction: a robot, camera stand, person or other object hiding the centre does not establish screen coverage. Use another unobstructed picture to identify the cause, or mark it uncertain. Shadows on the board are not opaque screen intersections. Hinge gaps remain open.

These are the same 3 × 189 = 567 sight tests as the simulator: an opaque screen hiding the registered centre from the fixed viewing point intercepts that finite sight segment before the target. No numerical ray construction is needed. Real lens position, aperture, image and registration uncertainty can make a near-edge test uncertain; direct pictures do not remove that limitation.

At each instant let B be the number certainly blocked and U the number uncertain. The blocked count lies between B and B+U. Full coverage is established only when B ≥ 562 at every instant; partial coverage when B ≥ 511 throughout. A tier is ruled out by coverage only if B+U is below its threshold at a required instant. Otherwise unresolved cases remain indeterminate. All other conditions below must hold in that same window. The discrete target and three viewpoints approximate privacy; they do not establish shielding of every continuous point, observer position or intervening time.

## Support, upright and intact hinges

Record front/side views of all six feet, with short rulers against the actual local floor and the bottom edges. Read each foot's lowest bottom height relative to that floor; its allowed interval is -5 to +5 mm. A 5 mm feeler can check an accessible gap after the window, without lifting, moving or supporting a panel. A gauge at one foot says nothing about the other five. Hidden bottom edges or an uncertain floor reference remain indeterminate. Do not substitute a statement that the screen “looks stable.”

Read each panel's inclination from vertical with an ordinary level/5° angle gauge or a level-referenced picture of its straight frame. Include the instrument and image-reading uncertainty; the upper bound must be at most 5°. A single side view bounds only the tilt it actually shows. A conservative sum of the two orthogonal tilt-component bounds at most 5° is sufficient; two separate readings each below 5° are not sufficient. Otherwise use an instrument/template that bounds the full inclination, or record indeterminate.

Use the assembly's registered near-floor frame landmarks and front/side rulers to bound the separation of each declared parent/child hinge anchor. The bound on the full three-coordinate separation must be at most 5 mm for each hinge. Known drawing offsets may locate an anchor which lies on the abstract axis; a photograph of an intact upper bearing alone does not measure the low declared anchor. Any unbounded offset, hidden landmark or measurement interval crossing 5 mm is indeterminate. Witness marks must be purely visual, fixed to their own rigid panel, and must not link, restrain or support panels.

## The same 5 mm origin and 1° rotation conditions

Use floor-plan and side rulers or ruler-referenced photographs to read changes of the declared body origin in x, y and z at each instant relative to the first image of this window. Use the declared origin, or a nearby rigid witness with a known offset; account for that offset's change under rotation and its reading uncertainty. This is a few ruler readings per panel, not a full-scene reconstruction.

For each coordinate, form a conservative bound, for example Bx = |measured x change| + uncertainty at the first and current readings, and similarly By and Bz. Only sqrt(Bx² + By² + Bz²) ≤ 5 mm establishes the shared origin-displacement condition. Three separate coordinate bounds of 5 mm do not suffice. An upper bound above 5 mm is inconclusive, not proof that the actual displacement exceeds 5 mm. A reliable lower bound above 5 mm establishes a violation.

Read yaw, pitch and roll changes with ordinary angle scales/gauges or suitable ruler/level-referenced pictures, using the same Rz(yaw) Ry(pitch) Rx(roll) convention and a consistent angle branch. Add the three absolute changes and their reading-error bounds. A sum ≤ 1° is a conservative sufficient proof that the shortest relative SO(3) angle is ≤ 1°. **Checking each axis separately against 1° is not sufficient.** A sum above 1° does not itself prove an SO(3) violation; use a direct relative-angle bound if available, or record indeterminate. No unmeasured angular precision is assumed.

## Released clearance over the whole robot

The 50 mm requirement concerns every real exterior part of both robots, hands and fingers, against every screen frame, inlay, foot and external hinge/fastener. The simulator checks its 630 hulls; a human need not reconstruct those meshes. Do not check only the grippers.

Use ordinary 50 mm gauges and photos covering all potentially closest exterior regions. A loose-fitting gauge whose verified width is at least 50 mm, with reading uncertainty allowed for, can establish the local gap. One gauge location does not certify the whole robot. Include hidden elbows, link backs, fingers, feet and added hardware, or use a conservative, visibly verified separation bound for the entire objects.

For example, use a ruler, straightedge and plumb line to establish that **all** robot exteriors lie behind one vertical plane and **all** screen exteriors lie beyond a parallel plane. A verified gap of at least 50 mm between those planes proves the required Euclidean separation for every pair of exterior points. Document the extremities with front/side pictures and include position, shape and reading uncertainty in the bound. It is unnecessary to find the closest pair or build a mesh when this whole-object bound is sufficient. If an occluded part could cross a bounding plane, the bound is not verified; take a clear view or record indeterminate. A separation bound below 50 mm is inconclusive, not a clearance violation.

## Gauges cannot change the result

Insert feelers, place levels or present clearance gauges only **after** the scored window, with uninterrupted video showing that the screen and every relevant robot part retain the same pose as that window. Do not lift the screen, use a gauge as a prop, push a panel, disturb a robot or change its command to create a new measuring pose. Keep contact-free clearance checks where possible. If any movement or occlusion prevents establishing the same pose, the later reading cannot certify the earlier window. If there is a bounded small difference, include its worst-case effect in the measurement interval; otherwise record indeterminate.

The window images must themselves show the required pose bounds and absence of intervening changes. An end-of-trial gauge reading alone does not establish 21 samples of support or clearance. The judge may need additional fixed front/side video to see the feet, frame scales and robot extremities; these views are evidence, not new score conditions or a requirement for a 3D camera system.

## Record the decision

Keep a 21-row sheet with blocked/visible/uncertain counts, each foot-height interval, each panel's upright bound, both hinge-gap bounds, each origin-displacement and relative-angle bound, and the all-robot clearance lower bound. Retain the original pictures and the uninterrupted post-window gauge video.

Score 1.0 only when every shared condition and the full coverage tier are established throughout. Score 0.5 only when every shared condition and partial coverage are established and the full tier is ruled out. If valid evidence establishes neither tier, score 0. If measurement, visibility or timing cannot decide a required condition or tier, report indeterminate with the reason, rather than treating unknown as zero or as proof of a violation. A clearly established violation can rule out credit; identify only what is actually established.

These procedures are proposed ordinary-tool checks. Their attainable precision, line-of-sight registration and repeatability must be checked on the actual build; this document does not assert that they have been demonstrated. More detailed reconstruction is optional diagnostic evidence when useful, not the primary human acceptance requirement.
