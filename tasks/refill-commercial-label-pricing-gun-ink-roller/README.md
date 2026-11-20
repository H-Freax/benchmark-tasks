# Refill commercial pricing gun ink roller

## Summary
A commercial handheld label pricing gun (Monarch / Garvey style) is held in a bench fixture with its front ink roller access door latched shut over a spent ink roller. A fresh ink roller sits in the staging zone. Replenishing label gun ink rollers is standard operating maintenance across retail, supermarket, and logistics operations (>3,500,000 workers under BLS SOC 41-2031 and 53-7065). To complete the task:
1. Swing open the front ink roller access door past its retention detent (>= 55 deg).
2. Remove the spent ink roller from the carrier cradle and place it into the discard waste tray.
3. Pick up the fresh cylindrical ink roller (12 mm OD x 28 mm L, 12 g), transport it to the pricing gun, and seat it firmly into the carrier journals (< 15 mm horizontal, < 12 mm vertical error).
4. Swing the front ink door shut until the detent latch snaps flush (< 5 deg from closed).
5. Clear both arms (>= 50 mm); the secured assembly must remain in equilibrium for >= 1.0 s.

## Environment & Files
- `task.json`: Task metadata and schema specifications.
- `refill_commercial_label_pricing_gun_ink_roller.py`: Scene geometry, USD Physics articulation, reversible scoring logic, and bimanual demonstration trajectory.
- `mechkit.py`: Modular USD Physics articulation helper.

## Offline check, no GPU
```bash
/home/freax/IsaacLab-Arena/.venv/bin/python tasks/refill-commercial-label-pricing-gun-ink-roller/refill_commercial_label_pricing_gun_ink_roller.py
```

## Qualification Runs
- Run limit: 800 steps (40.0 s at 20 Hz).
- Demo 1 & 2: `yam_scripted` full bimanual demonstrations scoring 1.00.
- Negative controls:
  - `door_left_open`: scores 0.75 (door unlatched).
  - `unremoved_old_roller`: scores 0.20.
- Zero-policy baseline: `Dream-1` scores 0.00.
