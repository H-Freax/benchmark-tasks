# Passive PTFE retention approximation (V7)

V5 and V6 native references recovered over 98 mL water with no oil carryover,
spill or withdrawal recontact, but their released key drifted to about 13 degrees
and continued flowing. V6's first completely unloaded observation had angle
0.00033444 rad and angular Y velocity +0.03222429 rad/s. The old 0.005
`physxJoint:jointFriction` was a dimensionless declaration ignored for this
standalone joint, as the actual backend warning states. It was never effective
0.005 N m resistance. Repeating controller-only settling does not repair that
missing physical mechanism. The final source has two complete native references with score 1.0 and a separate Dream-1 attempt with score 0.0. See README for endpoint results and rendering differences.

## Real mechanism and reproducible apparatus

[DWK PYREX](https://www.dwk.com/pyrex-stopcock-straight-bore-plain-arms-ptfe-key)
describes a polished tapered barrel, self-lubricating PTFE key, silicone O-ring
and PP retaining device. [DWK's replacement key](https://www.dwk.com/ptfe-key-with-thread-and-three-part-retaining-device)
has a threaded retaining assembly. [SGC LABS](https://labsglass.com/product-category/scientific-glass-blowing-supplies/stopcocks/ptfe-key-stopcocks-for-glass-blowing/ptfe-key-stopcock-straight-bore/)
states that its threaded nut adjusts turning tension, with a washer in the
retention assembly. Sources checked 2026-11-18 UTC. These sources support the
retention mechanism, not a numerical torque curve or this model's parameters.

Reconstruct using ordinary full-size 250 mL glassware with a matching tapered
PTFE plug, retaining nut/washer/O-ring, and approximately 50 mm normal key handle.
Follow that product's assembly instructions; keep the barrel supported in the
visible stand. Seat the matching plug, fit its original retainer, and adjust
finger-tight for smooth turning and sealing. Do not glue, latch or lock it at a
chosen angle. Use room-temperature water/oil only. The simulator retains the
existing cylindrical shaft geometry and omits taper/nut/O-ring contact details;
their rotational resistance is an explicit lumped approximation, not resolved
tribology. The retainer's axial retention is represented by the ordinary
revolute joint. Geometry is an author-made engineering equivalent, not OEM CAD.

## Selected law, parameters and limits

The intended native torque is `tau = -clip(b * omega, -tau_cap, tau_cap)` with
`b = 0.020 N m s/rad`, `tau_cap = 0.010 N m`, stiffness zero and target velocity
zero. These two values are **author-selected, uncalibrated engineering settings**.
They provide a modest finite rotational resistance: cap torque corresponds to
0.40 N tangential force at a measured 25 mm handle radius. This force is a
conversion, not a measured operating force. The regularization reaches its cap
at 0.5 rad/s. It permits normal turning under contact and has no preferred angle.

| Quantity | Value / authority |
| --- | --- |
| Torque cap | 0.010 N m, author-selected, not measured |
| Velocity slope | 0.020 N m s/rad, author-selected, not measured |
| Material coefficient of friction | Unknown; existing collider materials unchanged |
| Nut axial preload | Unknown; not equal to integrated normal force on a taper |
| Integrated normal contact load | Unknown |
| Effective friction radius | Unknown; shaft's geometric radius is 6 mm, not a measured friction radius |
| Real breakaway and running torque | Unknown; require physical measurement |
| Native realized torque | No direct torque time series; exact realized torque remains unverified |

`tau ≈ mu * N_integrated * r_effective` is only a dimensional relationship.
Neither torque nor preload is inferred from an invented friction coefficient.
The obsolete ignored 0.005 joint attribute is removed; body angular damping,
collision friction, masses, inertia configuration, gravity, joint limits,
geometry, robot capability, controller, fluid model and grader are unchanged.

This is velocity-regularized resistance, **not exact static Coulomb friction**.
At rest it supplies zero torque at every angle. Under a constant external torque
below the cap it can creep at `omega = tau_external/b`; above the cap it cannot
balance the load. It does not enforce a seal, correct an angle, erase momentum,
hold an arbitrary payload or close the valve automatically. Native solver
discretization must be checked separately. The only implementation is a native
joint drive authored once; CPU `torque_nm` is an audit formula, never a runtime
force callback.

## Units and backend mapping

The [official NVIDIA drive schema mapping](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/population/PhysicsDriveAPI.html)
documents revolute `angular` drives, force type, rotational gains in degree
units, and maxForce in torque units. Therefore authored damping is
`0.020*pi/180 = 0.00034906585 N m s/degree`, maxForce is `0.010 N m`, stiffness
is explicitly zero and targetVelocity is explicitly zero. No targetPosition is
authored; its fallback value has no effect at zero stiffness. This is not an
angle servo or powered opening motor. The newer documentation supports the
mapping; actual installed USD author/readback is checked offline, while the complete native references establish contact operation and release under this implementation. They do not calibrate its exact torque law.

## Practical real calibration

With the funnel clamped and empty, photograph/measure the handle contact radius.
Use a small calibrated force gauge or spring scale tangential to the key, with
the pull perpendicular to the radius. Record radius, pull direction, first-motion
force and steady slow-turning force in both directions at approximately 0, 30,
60 and 90 degrees, for at least three repeats. Torque is the measured tangential
force times the measured perpendicular radius; account for handle weight and
gauge resolution. Record retainer adjustment, plug model, lubrication condition
and temperature. Never infer axial nut preload from that torque alone.

Then film free release at several angles and gentle signed initial velocities
with a scale/angle marker and timestamp. Estimate angular decay, not just final
position. Select the lumped cap/slope from those observations and repeat the
full water/oil drainage and 30-second no-drop check. If static retention or
angle-dependent torque materially controls the task, replace this regularized
law with a supported physical equivalent and disclose the residual mismatch.
These are future instructions; no physical measurements are claimed here.

## Validation boundary and root's native checklist

CPU tests cover signed power `tau*omega <= 0`, finite cap, degree conversion,
non-finite rejection, no selected-angle attractor, analytical unforced decay
and the deliberate constant-load creep counterexample. USD in-memory authoring
reads the actual attributes. None of this runs a physics simulator or proves
native release stability, calibrated torque or task success.

The two complete native references turn and close the key through ordinary robot contact, then release it. Independent reconstruction verifies actual no-flow and zero robot contacts at the terminal state. A stationary 45-degree probe remained at 45 degrees for 121 observations without robot contacts, providing one no-attractor check. Other initial angles, signed velocity decay and a directly measured native torque/energy curve have not been validated. The no-vent negative retained the stopper and recovered only 0.781 mL while opening the actual key to 48.26 degrees. Those probes used the same physical implementation before the rendering-only update.

These simulator observations do not establish real PTFE friction, static breakaway torque or hardware calibration. Actual pose and raw angular velocity disagreement in earlier runs was retained rather than replacing a channel with zero. The final endpoint requires the actual closed key, no flow, released apparatus and fresh stability observations.
