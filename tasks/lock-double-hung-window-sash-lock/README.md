# Shut a double-hung window against its sill weatherstrip and lock its cam sash lock

Instruction given to the policy: `Shut the window's lower sash and lock it with its cam sash lock.`

Files: `task.json`, `lock_double_hung_window_sash_lock.py` (scene, staged `score()`, `success()`, active 30 N lift test, scripted `demo()` with diagnostic wrong-way variants, and offline self-test), `mechkit.py` (multi-box rigid parts, sprung prismatic joints and hinges with Coulomb friction, steady test force, and full URDF robot collision hulls), `README.md`.

## What is modelled

A complete compact wood double-hung utility window (600 mm wide x 489.4 mm tall overall, nominal 24 x 20 in unit) standing upright across the bench on a 10 mm steel mounting plate:

- **Frame and upper sash.** Painted pine sill, two 38 mm side jambs (`2 mm` running clearance each side of the sash) and 38 mm head jamb, with a fixed glazed upper sash (`520 x 235 x 34.9 mm`, `4 mm` polycarbonate pane) in the upper half whose meeting rail top lies at `z = 245 mm` above the bench.
- **Lower sash.** A `1.50 kg` glazed sash (`520 x 205 x 34.9 mm`, `4 mm` polycarbonate pane) riding vertically in the jamb liners. A `100 N/m` balance spring carries its weight at the sill and `3.0 N` of Coulomb friction holds it wherever it is left. Shut (`0 mm`), its stiles stand on steel sill stops (`z = 40 mm`) and its meeting rail top is flush with the upper meeting rail (`z = 245 mm`). It starts `12 mm` ajar (`z = 257 mm`).
- **Sill bulb weatherstrip.** A `25 g` black EPDM bulb strip on a `600 N/m` spring whose free top stands `10 mm` above the sill stops (`z = 50 mm`). Pushing the sash shut compresses it fully (`6.0 N` upward reaction), so an unlocked sash pushed shut and let go rebounds about `4.3 mm`.
- **Cam sash lock and bridge keeper.** Brass-finished die-cast zinc checkrail lock (`2-1/16 in` / `52.4 mm` pattern, Prime-Line F 2588 / F 2574 class): `50 g` hub, `3.0 mm` thick flat cam (`3.0 to 6.0 mm` above the lower rail) and upright finger lever tab (`5 mm` thick, `12 to 26 mm` above the rail) turning `150 deg` against `0.04 N m` friction. On the upper meeting rail sits an `18 x 24 x 3 mm` bridge keeper (`underside 7.5 mm`, `top 10.5 mm` above `z = 245 mm`). All three regimes arise from rigid contact between the cam and keeper:
  1. Within `1.5 mm` of shut, the cam swings cleanly under the bridge.
  2. Between `1.5` and `7.5 mm` ajar (including the `~4.3 mm` weatherstrip rebound), the cam strikes the keeper bridge's edge and stalls short.
  3. At `7.5 mm` or more ajar (including the `12 mm` start), the cam swings clear over the top of the keeper (a false lock).
- **Active 30 N lift test.** Once the cam is under the keeper bridge (`sash <= 2.4 mm`, `lever >= 145 deg`, 3D cam tip verified inside and beneath the bridge) and every URDF collision hull of both arms and grippers has stayed at least `20 mm` clear with the sash settled (`within 0.3 mm`) for `0.5 s`, `30 N` straight upward is applied to the middle of the lower meeting rail for `0.5 s`. It passes only if the sash rises at most `1.5 mm`, remains within `2.0 mm` of the sill stops, and both arms stay `>= 20 mm` clear throughout.

Not modelled: sideways draw of the rails by the cam, sash racking under off-centre load, wear.

## Score

| End-of-trial state | `task_score` |
| --- | ---: |
| Sash shut (`<= 2.4 mm`), cam locked under keeper (`>= 145 deg`), arms `>= 20 mm` clear, 30 N lift test passed, sash & lock at rest (held `1.0 s` for `success`) | `1.0` |
| Sash shut and lever thrown (`>= 145 deg`) with cam under keeper, before arm clearance or lift-test completion | `0.7` |
| Sash shut and cam engaged partway under keeper (`120..145 deg`), retaining sash against weatherstrip | `0.5` |
| Sash shut (`<= 2.4 mm`), lock not engaged under keeper | `0.3` |
| Sash not shut (including false lock with cam over keeper, or early release rebounded on weatherstrip) | `0.0` |

## Environment

Isaac Lab-Arena 0.3.1 with `yam-arena` and `inspect-robots`.

## Offline check (no GPU)

```bash
python tasks/lock-double-hung-window-sash-lock/lock_double_hung_window_sash_lock.py
```

Verifies 18 scoring, anti-cheat and lift-test cases on hand-built states, checks the physical contact/rebound invariants, replays all 7 trajectory variants through 20 Hz inverse kinematics (`< 0.1 mm` IK error, all joints inside URDF limits), and checks 3D URDF collision-hull clearances between both arms and both window panes.

## Run and score one trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lock-double-hung-window-sash-lock/lock_double_hung_window_sash_lock.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Shut the window's lower sash and lock it with its cam sash lock." \
  --scorer task_score --max-steps 600 --seed 0 --no-prompt --store-frames \
  --log-dir runs/lock-double-hung-window-sash-lock
```

For Dream-1 use `--policy dream -E cam_width=320 -E cam_height=176` with the same arguments. `SASH_DEMO=false_lock | early_release | hold_only | partial_throw | hand_on | idle` runs the diagnostic wrong-way controls.

## Results (seed 0, `object_jitter=0`, 20 Hz, max 600 steps)

RESULTS_PENDING
