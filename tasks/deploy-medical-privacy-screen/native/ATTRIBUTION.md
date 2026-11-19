# Component provenance

`ordinary_arm.py`: exact `Arm`, `wait`, `width` AST spans from
[PR 2036](https://github.com/robocurve/benchmark-tasks/pull/2036),
merge `9c7c8af6a43b230d789754a4642fd712a60116e6`,
`tasks/adjust-monitor-to-portrait/demo.py`.
Source SHA256: `e0c34b3a53cb74c4d8e0955181b71931b19f5459623c0c5c1c8359115916f61d`.

`tablet-reference-snippets.txt`: exact method spans from
[PR 2165](https://github.com/robocurve/benchmark-tasks/pull/2165),
merge `d6ff5edc6dc96e7d6a1db6196f3fe8c8bb73f490`,
`tasks/stand-tablet-back-support/demo.py`.

Both task directories provide the identical MIT notice copied as `LICENSE`.
Keep that notice when redistributing substantial excerpts. These are upstream
reference controllers; no independent native result is asserted here.

## Integration

1. Copy `ordinary_arm.py`, `LICENSE`, and this attribution into the new task.
2. Instantiate `Arm(side, actual_base_position)` using the deployed station.
   The donor's raised robot pedestals and task coordinates are not copied.
3. Each yielded tick exposes `arm.cmd` (6 arm joints + normalized opening).
   Submit via the task's existing ordinary robot-action path. This module
   does not step physics, access task body views, attach objects or change PD.
4. Verify full 630-hull path clearance and both position/orientation IK error;
   the original controller only reports position residual. Check native actual
   jaw capture, both pad contacts and object-relative grip drift separately.
5. The tablet snippets illustrate actual-frame planning. Their 7mm thickness,
   contact thresholds, 180deg wrist choice, seeds and API paths are task-specific.
   `right_top_grasp` by itself checks any right-hand contact, not both pads.
   Native checks from an increased-PD donor do not establish stock-PD success.

## Excluded donor runtime

PR2165 runtime.py lines200-205 multiplies PD stiffness by8/4.
PR2122 README documents the same gain changes.
PR2190 model.py line25 writes task-body root poses during runtime.
These mechanisms are excluded from this ordinary-controller kit.

This task copies ordinary_arm.py only. No tablet snippets are bundled, and no pram dynamics/capture/runtime were copied.
