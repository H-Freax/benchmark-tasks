### Task: Set Cordless Jigsaw Shoe to 45-Degree Bevel and Clamp

In standard carpentry and woodworking operations (BLS SOC 47-2031, >1,010,000 workers; BLS
Occupational Outlook Handbook), carpenters regularly make bevel cuts on trim, stair parts,
cabinetry, and framing. Setting a cordless orbital jigsaw's shoe angle to a 45-degree bevel is an
essential, fundamental power-tool setup procedure.

### Manipulation Sequence
1. The jigsaw (commercial DeWalt DCS331 20V MAX bare tool, 2.45 kg, 286 mm long, aluminum shoe)
   rests on its left side on a workbench rubber mat with the shoe square (0°) and locked.
2. The robot's right gripper approaches the keyless bevel cam lever's finger tab (recessed in a 22
   mm gap beside the motor housing) and pinches/swings the lever 60° along its arc to unlock the
   base.
3. Because the shoe engages an internal positive 0° stop, tilting is blocked until the robot slides
   the shoe 6 mm forward along its slotted guide trunnion.
4. The robot tilts the aluminum shoe 45° across intermediate detents (15° and 30°) until it seats
   into the positive 45° bevel detent notch.
5. The robot swings the bevel cam lever back flush (< 10° from housing, 22 N toggle clamp) to lock
   the shoe firmly at 45°.
6. Both grippers release and clear (> 5 cm); the assembly remains completely stable and locked in
   equilibrium for a 1.0 s hold check (20 consecutive steps at 20 Hz).

### Mechanics & Scoring
- Complete 1:1 tabletop commercial power tool: 2.45 kg DeWalt DCS331 jigsaw with authentic 170 x 90
  x 4 mm aluminum shoe plate and die-cast guide trunnions. Zero miniaturization, zero cropped stubs.
- True slotted trunnion arc and index pin geometry with realistic mechanical resistance: 12 N
  release, 15 N detent seating, and 22 N over-center cam latching clamp.
- Zero added grasp bosses, zero proximity-triggered weld cheats, 100% collision-enabled assembly.
- Scoring ladder:
  - 0.25: Unlocked and slid forward past 0° stop.
  - 0.50: Tilted to >= 28° (30° detent reached).
  - 0.75: Tilted to 45° ± 2°, but lever unclamped.
  - 1.00: 45° ± 2° bevel, lever clamped shut (< 10°), both hands cleared (> 5 cm), and stillness
    maintained in equilibrium for >= 1.0 s.
- Failure safeguards: Real drops, falls (> 1 m/s descent), or leaving bench latch 0.0. Safe
  repositioning on bench is not penalized.

### Evidence & Verification
- 2x scripted demos scoring 1.0 (`yam_scripted`, seeds 0 and 1, 1055 steps).
- 2x negative controls: `no_unlock` (0.00) and `lever_left_out` (0.75).
- Dream-1 baseline evaluation (0.00, 1200 steps).
- Full 79-test unit test suite passed. Playable H.264 media package uploaded (Media ID `M-RR97JT`).

