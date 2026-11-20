# install-engine-thermostat-with-jiggle-valve-and-housing-cover

Install an automotive engine thermostat into a coolant housing counterbore recess with its brass air-bleed jiggle valve clocked to the 12 o'clock position (+-15 degrees), press the molded perimeter rubber gasket into the sealing groove, and lower the water neck housing cover flush onto dual guide studs.

## Task Details

- **Title:** Install an engine thermostat clocked to 12 o'clock, seat the perimeter gasket, and fit housing cover
- **Category:** Impact on jobs
- **Keywords:** `thermostat`, `engine cooling`, `jiggle valve`, `gasket`, `water neck`, `automotive service`, `bimanual`
- **Instruction:** Take the engine thermostat from its staging tray, seat it in the coolant housing counterbore with its brass jiggle valve clocked to the 12 o'clock position, press the perimeter rubber gasket into the sealing groove, and lower the water neck housing cover flush onto the guide studs.
- **Run Limit:** 800 steps (40 s at 20 Hz)

## Occupational Relevance

Automotive service technicians and mechanics (SOC 49-3023; 760,000+ US jobs, BLS OEWS May 2023) regularly replace engine thermostats during routine cooling system service, water pump replacements, and diagnostic overheating repairs. OEM service manuals (AllData, Mitchell, factory repair procedures) strictly mandate clocking the thermostat's brass air-bleed jiggle valve to the highest vertical orientation (12 o'clock position within +-15 degrees) so trapped air bleeds past the closed thermostat during coolant refill; misorientation traps air pockets in the cylinder head, causing localized hot spots, thermal shock, and head gasket failure. The perimeter rubber gasket must be seated flush in the counterbore groove, and the aluminum water outlet cover must be lowered over dual guide studs without cocking to ensure a leak-tight seal before torquing fasteners.

## Setup & Real-World Hardware Reproduction

All items are standard, low-cost, off-the-shelf automotive components with total bill of materials under $28:
1. **Coolant Housing Base Fixture:** Cast aluminum engine block thermostat housing section ($18 off-the-shelf casting section). Measures 120 x 120 x 40 mm, mounted rigidly at (0.26, 0.0, table deck z=0.7475 m) with a central 56 mm counterbore recess (6 mm deep, recess floor at z=0.7815 m, top deck at z=0.7875 m) and dual vertical M8 guide studs at x = 0.260 +- 0.035 m (extending 24 mm above deck to z=0.8115 m).
2. **Automotive Thermostat:** Standard 54 mm automotive thermostat (Stant 13359 / Gates 33008, $6, 85 g): brass/copper wax pellet lower capsule (28 mm OD, extends 22 mm below flange), stainless steel mounting flange (54 mm OD, 2 mm thick), upper arch bridge (16 mm wide, extends 18 mm above flange), and a brass jiggle pin / air bleed valve offset 18 mm radially on the flange. Starts upright in a pocket staging tray at (0.22, -0.16 m).
3. **Molded Nitrile Perimeter Rubber Gasket Ring:** OEM-style pre-formed annular thermostat seal (Fel-Pro 35062, $2, 15 g): 54 mm ID, 62 mm OD, 3 mm thick. Starts on a stand at (0.22, 0.16 m).
4. **Cast Aluminum Water Outlet Neck Cover:** GM / Honda style 90-degree water neck cover ($8, 120 g): circular base flange with matching 9 mm clearance holes at +-35 mm and 54 mm pilot lip, and 32 mm OD outlet pipe. Starts on a cradle at (0.28, 0.16 m).
5. **Embodiment:** Dual YAM arm station (bases x=0, y=+-0.305 m, table deck z=0.7475 m), both parallel grippers starting in READY position.

## Scoring Rules & Anti-Cheat

1. **Defect gate:** If the thermostat touches the table deck outside the staging tray and housing recess, or is placed inverted (wax pellet pointing upward), `thermo_dropped_defect` or `thermo_inverted_defect` latches permanently to 0.0.
2. **0.25 (Lift & Carry):** Thermostat is lifted from the staging tray and carried over the housing.
3. **0.50 (Thermostat Seated & Clocked):** Thermostat is seated flush in the housing counterbore recess with its brass air-bleed jiggle valve verified clocked within +-15 degrees of the 12 o'clock position (+Y axis).
4. **0.75 (Perimeter Gasket Seated):** Molded rubber perimeter gasket is seated flush over the thermostat flange in the counterbore groove.
5. **1.0 (Full Assembly Complete):** Water outlet housing cover is lowered over dual guide studs flush onto the gasket (<1.0 mm gap from housing deck), both grippers are released and retracted clear (>20 mm), held motionless for 1.0 s.

Scoring is observable with standard visual inspection: top-down camera view verifies flush seating of the water neck housing cover over both guide studs (zero daylight gap under flange); overhead and wrist cameras confirm jiggle valve angular orientation (+-15 degrees of 12 o'clock) and perimeter gasket groove seating prior to cover placement.

## Environment & Reproduction

Run with IsaacLab-Arena / YAM embodiment:

```bash
inspect-robots run --embodiment yam_arena \
  -E task=tasks/install-engine-thermostat-with-jiggle-valve-and-housing-cover/install_engine_thermostat_with_jiggle_valve_and_housing_cover.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the engine thermostat from its staging tray, seat it in the coolant housing counterbore with its brass jiggle valve clocked to the 12 o'clock position, press the perimeter rubber gasket into the sealing groove, and lower the water neck housing cover flush onto the guide studs." \
  --scorer task_score --policy yam_scripted --store-frames --log-dir runs/demo1
```

Offline verification without GPU:

```bash
python tasks/install-engine-thermostat-with-jiggle-valve-and-housing-cover/install_engine_thermostat_with_jiggle_valve_and_housing_cover.py
```
