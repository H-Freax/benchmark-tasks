# Submerge an ice cooling paddle into a commercial stockpot of hot soup, agitate to chill, and hang the paddle on the rim bracket

Instruction given to the policy: `Steady the stockpot handle, pick up the frozen cooling paddle from the stand, immerse it into the hot soup, agitate in circular revolutions to chill the broth, lift to drain, and hang the paddle securely on the rim bracket. Move both hands clear.`

The instruction specifies the cooling, agitation, and storage hanging goals. Unstated occupational requirements scored by the benchmark under FDA Food Code 3-501.14:
1. The frozen cooling paddle (San Jamar Rapi-Kool style rapid chill cooling wand) must be lifted vertically clear of the sanitary countertop stand without knocking or dropping it onto the table.
2. The right arm must steady the stockpot by its welded side loop handle to prevent tipping or sliding against fluid shear forces during agitation.
3. The paddle blade must be submerged into the hot broth simulant (depth >= 20 mm) without contacting the pot bottom.
4. Active fluid agitation must be conducted in steady circular revolutions around the pot axis (>= 3 revolutions for partial credit 0.60; >= 5 revolutions for full credit 0.80). Excessive linear velocity (> 0.45 m/s) or acceleration (> 4.0 m/s^2) while submerged causes liquid splashing and latches a permanent scalding splash fault (0.0).
5. The paddle must be raised above the liquid level inside the pot for a drainage dwell (>= 0.8 s) so excess broth drips back into the pot.
6. The molded collar hook of the paddle must be securely seated onto the stockpot rim hanging bracket, both robot hands retracted clear (TCPs >= 30 mm, link collision hulls >= 5 mm), and the suspended paddle must remain stable for >= 1.0 s.

## The rig

In layout (all coordinates in metres from arm-base line midpoint):
- 8 qt Commercial Stainless Stockpot (Vollrath / Winco style): outer diameter 220 mm across flats, inner diameter 204 mm across flats, height 80 mm, total mass 2.40 kg. Sits on the counter at (0.260, 0.080) m. Pot floor at Z = 0.7535 m, rim top at Z = 0.8275 m. Contains hot broth simulant (depth 40 mm, liquid surface at Z = 0.7935 m, 34 mm clear headspace to rim).
- Stockpot Fixtures: welded front-left stainless rim bracket with a horizontal resting saddle at (0.240, 0.170, Z = 0.8275 m). Right side welded tubular loop handle at (0.260, -0.045, Z = 0.7925 m).
- Sanitary Stand: weighted stainless steel stand (80 x 80 x 30 mm, 400 g) at (0.200, 0.260) m with an 18 mm deep vertical socket. Stand floor at Z = 0.7595 m.
- Commercial Ice Paddle (San Jamar Rapi-Kool style): 120 g food-grade polyethylene wand. Finned blade (65 mm long, 55 mm wide, 26 mm thick, heat-dissipating cooling ribs), 18 mm collar with radial hanging hook projecting 25 mm along +Y with a 10 mm downward retaining lip, and 45 mm cylindrical grip handle (diameter 24 mm). Starts standing upright in the stand cradle (blade bottom at Z = 0.7605 m, handle grip centre at Z = 0.8655 m).

## Environment

Verified locally with Isaac Sim 6.1.0, `isaaclab-arena` distribution 0.3.0, `yam-arena` 0.1.0 and `inspect-robots` 1.0.0 on an RTX 3090. Physics runs at 240 Hz and control at 20 Hz. Activate runtime with:

```bash
source ~/IsaacLab-Arena/yam.env
```

## Offline check, no GPU

From the root of this repository:

```bash
~/IsaacLab-Arena/.venv/bin/python tasks/rapid-cool-soup-ice-paddle/rapid_cool_soup_ice_paddle.py
```

Asserts all 10 scoring self-test cases (including stand lift, broth immersion, agitation revolutions, broth drainage, rim hang, splash velocity violation, pot tipping, and premature hang bypass) and verifies the bimanual demonstration trajectory through inverse kinematics (both arms worst IK error < 0.085 mm, worst orientation error < 0.013 deg, all joints inside limits).

## Model and demo evaluation

Run with `inspect-robots`:

```bash
# Scripted bimanual demonstration
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rapid-cool-soup-ice-paddle/rapid_cool_soup_ice_paddle.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Steady the stockpot handle, pick up the frozen cooling paddle from the stand, immerse it into the hot soup, agitate in circular revolutions to chill the broth, lift to drain, and hang the paddle securely on the rim bracket. Move both hands clear." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames \
  --log-dir evidence/demo

# Hosted model policy evaluation (Dream-1)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rapid-cool-soup-ice-paddle/rapid_cool_soup_ice_paddle.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Steady the stockpot handle, pick up the frozen cooling paddle from the stand, immerse it into the hot soup, agitate in circular revolutions to chill the broth, lift to drain, and hang the paddle securely on the rim bracket. Move both hands clear." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames \
  --log-dir evidence/dream
```

## Real-World Trial Protocol, Sourcing and Shared-Rig Protection

### Sourcing & Equipment Cost
- **Stockpot**: Standard commercial 8 qt heavy-duty stainless steel stockpot / brazier ($35-$50 from WebstaurantStore / Winco). Standard stainless rim hook bracket ($12).
- **Cooling Wand**: San Jamar Rapi-Kool 64 oz rapid chill cooling wand ($35-$45). Food-grade durable HDPE filled with tap water and frozen overnight in standard freezer.
- **Stand & Heat Mat**: Commercial stainless utensil / wand stand ($18); 280 x 280 mm silicone heat-resistant counter mat ($6).
- Total fixture hardware cost is ~$110-$130, well within benchmark constraints.

### Broth Simulant & Shared-Rig Protection
- **Repeatable Simulant**: Water tinted with food-grade vegetable broth coloring and thickened with 0.4% carboxymethylcellulose (CMC) to match commercial chicken/vegetable broth viscosity (~10-15 cP at 65°C). Non-staining, non-toxic, and wipes completely clean with warm water.
- **Robot Protection**: Gripper pads are fitted with food-grade silicone fingertip covers. The robot never touches liquid: left arm grasps only the dry, sanitary upper handle of the paddle above the liquid line; right arm grasps only the dry external side loop handle of the pot.

### Practical Measurement Protocol
Real-world scoring uses standard kitchen tools without simulator-only instrumentation:
1. **Fluid Immersion & Agitation**: A digital immersion thermocouple and wireless logging probe track fluid convection. Manual/video inspection verifies >= 5 full circular revolutions without rim splashing.
2. **Splash Containment**: A pre-weighed perimeter absorbent liner around the pot detects spills. Liquid displacement > 20 mL over the rim registers a splash failure (0.0).
3. **Rim Bracket Hang**: Evaluator confirms the paddle collar hook is seated in the bracket saddle and applies a downward retention test; the paddle remains securely suspended without slipping or contacting the broth floor.

## Third-Party Assets and Licensing
All collision and visual geometry are synthesized procedurally in Python using standard geometric primitives (boxes, cylinders) via `mechkit.py` and `yam_arena`. No proprietary or third-party 3D meshes are used. All authored code and assets are licensed under Apache 2.0.

## Run limit, runtime and seeds
Run limit is 1,600 steps (80 seconds at 20 Hz). The scripted demonstration finishes in ~26.7 seconds (533 steps).
