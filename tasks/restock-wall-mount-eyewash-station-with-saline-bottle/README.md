# Restock a wall-mount emergency eyewash station with a sealed saline eyewash bottle

## Summary
A wall-mount secondary emergency eyewash station stands on the workbench with its transparent dust cover door shut over an expired eyewash bottle. A fresh 32 oz sealed saline bottle stands on the staging area. Restocking emergency eyewash units is a standard compliance duty under OSHA 29 CFR 1910.151(c) and ANSI Z358.1 across maintenance, laboratory, and manufacturing facilities (>1.6M workers, SOC 29-9011 and 49-9071). To complete the task:
1. Open the hinged clear dust cover door past its detent (>= 75 deg).
2. Remove the expired bottle from the station's cradle cavity and set it on the bench discard area.
3. Pick up the fresh 32 oz sealed saline bottle (980 g), seat it upright into the hollow cradle cavity with its contoured eyecup nozzle oriented forward toward the user (yaw within 25 deg).
4. Swing the dust cover door shut until the detent latch snaps (< 6 deg from shut).
5. Clear both arms (>= 50 mm); the compliant state must be maintained in equilibrium for >= 1.0 s.

## Environment & Files
- `task.json`: Task metadata and schema specifications.
- `restock_wall_mount_eyewash_station_with_saline_bottle.py`: Complete scene setup, USD Physics articulation, scoring functions, and bimanual demonstration trajectory.
- `mechkit.py`: Modular USD Physics articulation helper.

## Offline check, no GPU
```bash
/home/freax/IsaacLab-Arena/.venv/bin/python tasks/restock-wall-mount-eyewash-station-with-saline-bottle/restock_wall_mount_eyewash_station_with_saline_bottle.py
```

## Qualification Runs
- Run limit: 800 steps (40 s at 20 Hz).
- Demo 1 & 2: `yam_scripted` full-score demonstrations scoring 1.00.
- Negative controls:
  - `door_left_open`: scores 0.85 (door not latched).
  - `reversed_eyecup`: scores 0.70 (eyecup turned away from user).
  - `unremoved_old_bottle`: scores 0.20.
- Zero-policy baseline: `Dream-1` scores 0.00.
