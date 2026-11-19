# Brush the inner bottom and shoulder of a narrow-neck bottle

Instruction given to the policy:
`Brush the film marks off the inside bottom and shoulder of the bottle, then take the brush out and put both down.`

Files: `task.json`, `task.py` (the task file: objects, `score()`, `success()`, the scripted `demo()` and an offline
self-test), `geometry/geometry.json` (every shape, pose, material and film mark) and `native/`:
`bottle_scene.py` (per-step observation and the stage score), `cleaning.py` (film model), `grader.py` (end state),
`pad_contacts.py` (robot-to-item contacts), `reference.py` (the scripted reference), `native_geometry.py` and
`partkit.py` (multi-shape rigid bodies and per-step hooks), `vertex_helper.py`, `cooking_local.py`.

## What is modelled

- The bottle is a loose 209 g hollow glass body built from 97 convex pieces: a 4 mm base, a 59 mm bore body, a
  20 mm conical shoulder and a 44 mm bore, 25 mm long neck. Nothing holds it down; the other hand has to keep
  it in place while the brush works.
- The brush is one 39 g rigid body: a 42 mm x 40 mm head, a 5 mm steel stem and a 20 mm x 40 mm soft-grip
  handle (friction 1.2, the same as the platform's other graspable objects), all colliding. The head is a soft
  contact envelope (150 N/m and 1 Ns/m per contact point, friction 0.45) standing in for a bristle bundle: it
  gives by a few millimetres against the glass at a few tenths of a newton. Individual bristles are not
  simulated, and the head is 2 mm narrower than the neck rather than squeezing through it.
- 24 orange marks on the inner bottom (5 mm squares on rings of radius 10 and 22 mm) and 24 on the inside of
  the shoulder (4 mm squares at 99 and 109 mm height) stand in for dried film. A mark comes off only after the
  brush head has slid 20 mm across it in total while the head presses on the glass with 0.1 N or more
  (summed over the head's contact points) and the mark lies within 0.5 mm of the head. Touching without sliding, the stem, or a pass through the air
  removes nothing. Removed marks disappear from view; the glass itself never changes.
- The shoulder marks sit behind the neck: the head passes the 44 mm neck with 1 mm to spare on each side, and
  has to be moved off the bottle axis and up under the shoulder to reach them.
- Nothing is attached to the robot, no object pose or velocity is written and no force is applied by the task;
  the robot's drives are the stock ones.

## Scoring

Read from the last step of the episode (`native/bottle_scene.py: stage_score`):

| Stage | Score |
| --- | --- |
| Each region (bottom, shoulder): 40 % of its marked area removed | 0.15 each |
| Each region: 80 % of its marked area removed | 0.30 each |
| Both regions at 80 %, 90 % of all marked area, and the brush head out of the bottle | 0.80 |
| ... and both items put down for 2 s | 1.0 |

"Out of the bottle": every point of the head is at least 5 mm above the mouth, or beside the bottle.
"Put down": bottle and brush on or above the tabletop, slower than 20 mm/s and 0.1 rad/s, the bottle within
10 deg of upright, every gripper point at least 30 mm from both items and none of the 18 robot bodies touching
either. A missing contact reading counts as touching.

## Offline check, no GPU

```bash
python tasks/brush-narrow-neck-bottle/task.py
```

It asserts the film rules, the end-state checks and the stage table on hand-built states, then replays the
reference and its two incomplete variants on the arm kinematics. Expected output:

```
scoring: 31 cases pass
full (nominal bottle): 2094 steps = 104.7 s, worst IK 0.10 mm / 0.02 deg, path coverage bottom 1.00 shoulder 1.00
full (bottle shifted 20 mm and tilted 2 deg in the hand): 2094 steps = 104.7 s, worst IK 0.10 mm / 0.02 deg, path coverage bottom 1.00 shoulder 1.00
bottom_only (nominal bottle): 1094 steps = 54.7 s, worst IK 0.09 mm / 0.02 deg, path coverage bottom 1.00 shoulder 0.00
leave_brush_in (nominal bottle): 1854 steps = 92.7 s, worst IK 0.09 mm / 0.02 deg, path coverage bottom 1.00 shoulder 1.00
```

## Run and score one trial

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots; NumPy and SciPy.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/brush-narrow-neck-bottle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Brush the film marks off the inside bottom and shoulder of the bottle, then take the brush out and put both down." \
  --scorer task_score --max-steps 3000 --seed 0 --no-prompt --store-frames --log-dir runs/brush-narrow-neck-bottle
```

For Dream-1 use `--policy dream` with the same arguments. `BOTTLE_DEMO=bottom_only` or
`BOTTLE_DEMO=leave_brush_in` makes `yam_scripted` play an incomplete variant. `BOTTLE_TRACE_DIR=<dir>` writes
every step's poses, contacts and film state (about 50 MB per run). `BOTTLE_LEGACY_COOKING=1` selects the
non-UJITSO collision cooking backend on runtimes that need it; geometry and physics are the same.

A trial is 3,000 control steps at most (150 s at 20 Hz); the reference needs 2,034. Wall time for the reference
is about 8 minutes on an RTX 3090 with nothing else running, and 13 minutes when the GPU is shared.

## The scripted reference

`native/reference.py`. The left hand takes the bottle by its body from the side and keeps holding it. The right
hand picks the brush up by its handle, stands it upright, and lowers the head through the neck on the bottle's
axis as observed (the held bottle ends up about 20 mm from where it stood and leaning about 2 deg). It scrubs
the bottom with three 7 mm circles, then the lower ring of the shoulder with five circles and the upper ring
with six, the head's top edge pressed lightly under the cone, lifts the brush out, and lays it back on its tray
after the left hand has let the bottle go. While the brush is at or inside the bottle the right arm's joint
targets are corrected from the observed brush pose, and while scrubbing the tip height has its own correction:
the stock wrist is soft, and the cone otherwise pushes the head 2 mm up and down around each circle.

The reference stops early only on a real hazard: the bottle leaning more than 25 deg, the handle grasp lost,
or the steel stem pressing on the glass with more than 15 N. Contact between the soft head and the neck or
walls is ordinary use and does not stop it. The scorer never reads the reference's phases.

## Results (seed 0, no jitter)

| Run | task_score | Steps |
| --- | --- | --- |
| Scripted reference, run 1 (320 x 176 cameras) | 1.0 | 2,034 |
| Scripted reference, run 2 (640 x 352 cameras) | 1.0 | 2,034 |
| `BOTTLE_DEMO=bottom_only`: bottom scrubbed, shoulder skipped, brush out, both put down | 0.3 | 1,094 |
| `BOTTLE_DEMO=leave_brush_in`: both regions scrubbed, brush left standing in the bottle | 0.6 | 1,854 |
| Dream-1 | 0.0 | 3,000 |

In the full runs every mark is removed: each bottom mark has 64 to 169 mm of sliding under the head and each
shoulder mark 28 to 42 mm, against the 20 mm needed. The episode ends on success 2 s after the right hand has
let go of the brush. Dream-1 did not pick up the brush or touch the bottle.

## Reset and a physical version

Reset restores the documented poses and all 48 marks. For a physical trial use washable orange poster paint
thinned 1:4 with water by mass, applied through a stencil as 5 mm squares on the bottom and 4 mm squares on the
shoulder at the positions in `geometry/geometry.json`, dried 10 minutes at 20-25 C, and photograph the bottom
from above and the shoulder with a mirror or a small inspection camera before and after. The 20 mm sliding
threshold is an engineering stand-in and has not been calibrated against paint removal. This is a visible-film
test only: no claim about microbes or feeding safety is made, and the marked bottle must not be used for
feeding.

Related public work: a `wash_cup` policy that brushes the inside of an open cup held in the other hand
(https://huggingface.co/dddddsds31341/pi05-wash_cup_100-10k). This task differs in the narrower neck and in
the shoulder behind it, which a straight-in stroke does not reach.

## License

Task code and geometry: MIT (`LICENSE`). `native/partkit.py` keeps its MIT notice (`native/LICENSE`). The YAM
robot assets are external runtime dependencies under their own licenses and are not redistributed here.
