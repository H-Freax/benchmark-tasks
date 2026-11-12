# Uncover a cake slice and serve it onto a dessert plate

A slice of layer cake stands on a glass cake plate under a clear dome. Lift the dome off by its knob, set it down on
the table, and move the slice onto the dessert plate. Scored from the end state: dome set aside 0.3, slice upright
on the plate 0.7; a slice that tips over or falls, or a dropped dome, scores 0.

Files: `kitchen_cake_under_clear_dome.py` (scene, `score()`, `success()`, `demo()`), `scenekit.py` (multi-shape
parts: glass dome shell, wedge layers, rings), `task.json`.

## Run the scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/kitchen-cake-under-clear-dome/kitchen_cake_under_clear_dome.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Lift the clear dome off the cake plate by its knob and set it down on the table, then move the slice of cake onto the dessert plate." \
  --scorer task_score --max-steps 1200 --seed 0
```

Swap `--policy` for the model under test. Seed 0, no object jitter. The demo reaches 1.0 in about 28 s of sim
time (about 570 steps); the run limit is 1,200 steps.
