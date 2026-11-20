# Clamp a split-core current transformer around an insulated conductor and verify latch lock

Clamp the split-core current transformer around the insulated conductor until the hinged keeper clicks shut and locked, then clear both arms.

## Occupational Relevance
Installing split-core current transformers (CTs) for power submetering, energy audits, and motor load monitoring is an everyday core task for electricians and industrial machinery mechanics (BLS SOC 47-2111 and 49-9041, over 1.2 million US workers; NFPA 70 National Electrical Code Article 750). Commercial split-core CTs clamp around existing insulated power feeder cables without interrupting electrical service. The technician steadies the open U-core housing around the conductor, compresses the spring-loaded upper keeper down flush against the lower core faces (air gap < 0.5 mm) until the mechanical retention latch clicks locked, and clears all tools. An active 15 N verification tug along the cable confirms that the clamped CT maintains core closure and mechanical retention without unseating.

## Physics and Mechanism
- **Feeder Cable & Standoffs**: Heavy insulated 350 kcmil copper feeder cable (22 mm OD x 260 mm length, oriented along Y at x = 0.320 m, z = 0.8275 m, 80 mm above bench) rigidly supported by two steel terminal standoffs (30 x 40 x 78 mm) at y = -0.110 m and y = +0.110 m. A high-contrast yellow witness mark at y = 0.000 m denotes the submetering measurement position.
- **Split-Core Current Transformer**: Standard commercial 1:1 scale split-core CT (56 x 24 x 62 mm overall outer envelope, 24 x 24 mm central aperture window, total mass 0.27 kg):
  - `ct_base`: Lower U-core housing forming a cradle under the cable (0.18 kg). Rests positioned at the center witness mark on the cable at y = 0.000 m.
  - `ct_keeper`: Hinged upper core keeper (0.07 kg) on a revolute hinge at the rear upright leg, biased open to 45 degrees by an internal spring.
  - `keeper_latched`: Mechanical spring-retention catch locking the keeper shut against the base once pressed flush (<= 5.0 deg).
- **Bimanual Manipulation**:
  - Right arm: Approaches from -Y, descends beside the CT base flange, and steadies the base against cable sliding during clamping.
  - Left arm: Approaches from +Z above the keeper, descends with closed fingertips, and compresses the hinged keeper flush shut until the retention catch engages.
  - Both arms release and clear to park positions (>= 20 mm clearance to all collision hulls).
- **Active 15 N Verification Tug**: Once clamped with arms clear and settled for 0.3 s, an automated 15 N verification tug is applied along the cable for 0.5 s. The test passes if the CT remains within the alignment zone and the core remains securely shut.

## Scoring
- **0.25 (Stage 1)**: Upper keeper partially depressed past halfway mark (<= 30.0 deg) within witness zone (|y| <= 20 mm).
- **0.50 (Stage 2)**: Upper keeper swung shut flush against lower core face (<= 5.0 deg).
- **0.75 (Stage 3)**: Mechanical retention catch locked.
- **1.00 (Stage 4)**: Core locked shut, 15 N verification tug passed, both arms parked clear >= 20 mm, held stable for 1.0 s.
- **0.00**: Dropped part, unaligned, or failed proof test.

## Execution
```sh
inspect-robots run --embodiment yam_arena \
  -E task=tasks/clamp-split-core-current-transformer/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --max-steps 600 --store-frames \
  --instruction "Clamp the split-core current transformer around the insulated conductor until the hinged keeper clicks shut and locked, then clear both arms." \
  --scorer task_score
```
