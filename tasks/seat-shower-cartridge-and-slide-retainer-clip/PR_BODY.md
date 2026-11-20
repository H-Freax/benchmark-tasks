### Task: Seat Shower Valve Cartridge and Slide Retainer Clip

Under standard plumbing and facility maintenance practices (BLS 47-2152, >480,000 workers), replacing a leaking pressure-balanced shower cartridge (Moen 1222 Posi-Temp) requires precise bimanual coordination:
1. The left arm reaches the cartridge stem/face and pushes it inward (-X) against dual lubricated elastomeric O-rings (~14 N resistance) until it seats flush against the internal brass valve body shoulder stop (< 0.8 mm proud).
2. The right arm grasps the stamped brass U-shaped retainer clip from its staging block, carries it over the valve body, aligns the clip legs with the machined 1.8 mm vertical guide slots, and slides the clip down into engagement across the cartridge retaining flats.
3. If the cartridge is released early, O-ring elastomeric relaxation pushes it outward by 1.2 mm, causing the clip legs to collide with the cartridge body and fail to seat.
4. Once the clip is fully seated down onto its stops, both arms release and clear (>= 60 mm); mechanical retention holds the cartridge seated in equilibrium under backpressure for >= 1.0 s.

### Mechanics & Scoring
- Valve body modeled with true hollow cylindrical chamber (32 mm bore) and dual vertical guide slot channels (0.40 mm clearance) across cast brass ears.
- Authentic 1:1 tabletop hardware: 520 g cast brass rough-in valve casting, 120 g Moen 1222 cartridge (32 mm OD, 105 mm length), and 25 g stamped brass U-clip (32x24x1.5 mm).
- Zero added grasp bosses, zero proximity-triggered weld cheats, 100% collision-enabled assembly.
- Scoring progression:
  - 0.00: Unpushed (> 8 mm proud).
  - 0.20: Pushed halfway into bore (< 6 mm proud).
  - 0.45: Pushed flush home against shoulder stop (< 0.8 mm proud).
  - 0.70: Retainer clip entered into guide slots while cartridge held flush.
  - 0.90: Retainer clip fully seated down onto ear stops.
  - 1.00: Hands cleared (>= 60 mm) and assembly retained in equilibrium for >= 1.0 s.

### Evidence & Verification
- 2x scripted demos scoring 1.0 (`yam_scripted`, seeds 0 and 1).
- 2x negative controls: `no_push` (0.00) and `no_clip` (0.45, cartridge creeps out when released).
- Dream-1 baseline evaluation (0.00).
