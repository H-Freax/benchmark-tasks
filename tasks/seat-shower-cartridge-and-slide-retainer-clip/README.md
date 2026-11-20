# seat-shower-cartridge-and-slide-retainer-clip

## Task Overview
Push a single-handle pressure-balanced shower valve replacement cartridge (Moen 1222 Posi-Temp) flush into its valve body against O-ring compression resistance, slide the brass U-shaped retainer clip down into its slots, and release.

Shower cartridge replacement is routine maintenance for plumbers and facility technicians (BLS 47-2152, >480,000 workers). Under water pressure and mineral buildup, cartridges wear out and cause leaks or temperature imbalances:
1. The left arm reaches to the brass stem / face of the cartridge and pushes it inward (-X) against dual lubricated O-ring elastomeric resistance until it seats flush against the internal shoulder stop.
2. The right arm grasps the brass U-clip from its staging block, lifts it above the valve body, aligns the clip legs with the machined body slots, and slides the clip down onto its stops across the cartridge retaining flats.
3. If the cartridge is released early, O-ring elastomeric relaxation pushes it outward by ~1.2 mm, blocking clip insertion.
4. Once the clip is fully seated, both hands release and clear (>= 60 mm); mechanical retention holds the cartridge seated against outward forces.

## Mechanism & Physics
- **Valve Body**: Heavy cast-brass rough-in valve housing mounted rigidly to the bench fixture at X = 0.280 m, Y = 0.000 m. Machined with a 32 mm bore cavity and dual vertical retaining clip guide slots (0.40 mm clearance).
- **Cartridge**: Moen 1222 replacement cartridge assembly on a prismatic slider joint along X, starting 12 mm proud of the seated flush stop. Features 1100 N/m spring stiffness with 12 mm stroke (producing ~13.2 N resistance at flush seating), 8 N/(m/s) damping, and 2 N friction simulating lubricated O-ring drag.
- **Retainer Clip**: Solid stamped-brass U-shaped clip (32 mm tall, 26 mm wide, 1.5 mm thickness) staged adjacent to the valve at Y = -0.120 m.
- **Retaining Slots & Stop Alignment**: The clip enters two vertical guide slots in the valve body and seats across the cartridge neck groove when the cartridge is within 0.8 mm of flush home.

## Scoring
- **0.00**: Cartridge remains unpushed (> 8 mm proud).
- **0.20**: Cartridge pushed halfway into bore (< 6 mm proud).
- **0.45**: Cartridge pushed flush against internal shoulder (< 0.8 mm proud).
- **0.70**: Retainer clip aligned and entered into body guide slots while cartridge held flush.
- **0.90**: Retainer clip fully seated down onto stops across cartridge retaining groove.
- **1.00**: Both hands cleared (>= 60 mm); assembly verified retained under spring-back for >= 1.0 s.

## Reproduction Commands
```bash
# Offline self-test & kinematic verification:
python tasks/seat-shower-cartridge-and-slide-retainer-clip/seat_shower_cartridge_and_slide_retainer_clip.py

# Scripted policy demonstration:
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-shower-cartridge-and-slide-retainer-clip/seat_shower_cartridge_and_slide_retainer_clip.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Push the shower valve cartridge flush against its O-rings, slide the retainer clip down into its slots, and release." \
  --scorer task_score --max-steps 800 --seed 0 --no-prompt --store-frames --log-dir runs/demo_1
```
