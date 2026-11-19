# Observable physical scoring protocol

This document preserves the shared sampled acceptance conditions below. For a physical trial, the primary proposed procedure uses direct photos, rulers, a level and ordinary gauges; it does not require reconstructing the scene or robot meshes. Simulation success is distinct from validated physical measurement accuracy.

## Exact shared acceptance rule

Use a final window of **21 consecutive fresh canonical 20 Hz observations**, indexed k=0,...,20 at t[k]=t[0]+k/20 seconds, spanning 1.000 s. Each observation is an actual state readback or directly observed physical sample. The window must finish within the 1,200-control-step limit. An earlier successful window cannot certify a later failing final state. Missing, duplicate, stale or out-of-order samples break a window; collect a new complete window. Neither interpolated poses nor repeated frames manufacture missing observations.

For every panel j, let p[j,k] be its declared body origin and R[j,k] its rigid-body orientation. At every sample require:

- ||p[j,k]-p[j,0]|| <= 0.005 m.
- angle(R[j,k] R[j,0]^T) <= 1 degree, using the shortest SO(3) rotation angle in [0,180 degrees]. Quaternion sign changes do not constitute motion.

The reference is the first sample of this same window. This is neither a pairwise-spread rule nor a bound on displacement of every panel edge. Instantaneous native linear and angular speed magnitudes are **not** score criteria. Required native velocity records still must have the correct shape and finite values; missing or nonfinite data invalidate evidence. No speed threshold, finite-difference speed threshold or hidden fast-motion penalty is substituted. Small rapid motion between samples is not excluded by the sampled envelope.

Every one of the same 21 samples must also meet all applicable terminal conditions:

| Condition | Shared numerical definition |
|---|---|
| Floor support | All six declared foot bottoms, each measured as the minimum world z of its finite foot solid, lie within +/-5 mm of the actual floor top (the current scene uses z=-0.0125 m). |
| Upright | Each panel local up axis is within 5 degrees of world up. |
| Hinge integrity | The Euclidean separation of each joint's declared parent/child anchor points is <=5 mm, for both hinges. |
| Robot release/clearance | Minimum finite-solid Euclidean separation of the screen and **all 630 current robot collision hulls**, using their current link poses, is >=50 mm. |
| Coverage | Evaluate all 567 finite sight segments (three sight points x 189 target dot centres). A segment counts only if it intersects a declared finite opaque screen solid before reaching its dot. Hinge air gaps remain open. |

The screen clearance union includes its declared frames, inlays and feet; the physical survey must account for external hinge hardware and other exterior deviations. Robot geometry includes every declared link, hand and finger hull of both robots, rather than only tips or end effectors. An incomplete 630-hull set cannot certify release.

**Score 1.0:** all conditions hold and coverage is at least 562/567 (>=99%) at every sample. **Score 0.5:** all conditions hold and coverage is at least 511/567 (>=90%) at every sample, but the full tier is not met throughout. **Score 0:** complete valid evidence establishes that neither tier is met. Coverage and all geometric conditions must persist through the same window; measurements from separate windows cannot be combined.

Indeterminate geometry, absent evidence and measurement uncertainty establish neither a pass nor a physical violation. Report `indeterminate`/`invalid`, its reason and any diagnostic no-credit return separately from a certified physical score of zero. A conservative solver lower bound below 50 mm is not evidence that physical clearance is below 50 mm. A known violation can rule out credit even if some other conditions remain unknown; identify only the violation actually established.

There is no bimanual-use, fixed panel-angle, hand-assignment, operation-order or return-to-READY gate. The unchanged initial U layout blocks486/567 and receives0 after a complete valid window. Partial credit requires a measurable shielding improvement to at least511/567; full credit still requires562/567 throughout.

## Primary human procedure

Follow [PRIMARY-HUMAN-SCORING.md](PRIMARY-HUMAN-SCORING.md). Three fixed viewpoint recordings directly classify the 189 target-dot centres at each of the same 21 instants, giving the same 567 sight tests without building rays in software. Count certain screen occlusion, not shadows, robot occlusion or merely disappearing dot edges. Record uncertain centres explicitly and use count bounds against the unchanged 562/511 thresholds.

Use floor/side rulers and a 5 mm feeler for the six foot-bottom intervals; a level/angle gauge for the 5° upright limit; registered frame landmarks and rulers for both 5 mm anchor-gap bounds. Bound origin movement from x/y/z ruler changes by the Euclidean upper bound, which must be at most 5 mm. A sum of absolute yaw/pitch/roll changes, including reading uncertainty, of at most 1° is sufficient for the same SO(3) condition; testing each axis separately against 1° is not. A bound which is insufficient is indeterminate, not a violation.

For release, check the real exteriors of every robot link, hand and finger against the complete screen with ordinary 50 mm gauges and views of all potentially closest regions, or a visibly verified whole-object separation bound. The 630-hull inventory describes the simulation representation; humans are not required to rebuild it. Parallel ruler/plumb-line bounding planes separated by at least 50 mm can prove the same Euclidean clearance without finding a closest mesh pair. Hidden or unbounded extremities cannot be omitted.

Take any contact or inserted-gauge readings after the scored window only. Continuous video must establish that the window pose is retained, with any bounded difference included in the measurement error. No gauge may support or move the screen, disturb a robot or create a new measuring pose. Without that correspondence, later measurements do not certify the window.

## Uncertainty and optional geometric diagnostics

For a measured quantity x with uncertainty U, a limit x<=L is confirmed only if estimate+U<=L; a lower limit x>=L only if estimate-U>=L. Use signed intervals for the two-sided foot condition. For coverage, certainly blocked and possibly blocked counts provide lower and upper bounds. A boundary, occlusion, insufficient reading accuracy or unresolved time correspondence is indeterminate. Unknown is not a certified score of zero. No numerical accuracy is claimed for an untested camera, ruler, gauge or registration method.

Detailed pose reconstruction and finite-convex closest-point solvers remain optional diagnostics. In simulation the frozen scorer retains all 630 actual collision hulls and finite solids. Conservative lower bounds prove clearance only when at least 50 mm; smaller lower bounds require refinement or unknown. In physical judging the same clearance concerns actual exteriors, including the disclosed added hardware; ordinary verified whole-object bounds or gauges may establish it directly. No software mesh reconstruction, OEM CAD or forward-kinematic survey is a mandatory human acceptance step.

## Evidence and measurement limits

Previous-revision historical diagnostics: reference1.0 after824 controls and model0.5 after1,200 controls used the former70% partial threshold and lacked visible viewing marks. They are not current scores or matched media. Current-revision native reset, visibility/setup evidence, complete references, demonstration and model run are pending; task.json clears the previous media ID.

The setup image records a separate settled reset using the same physical scene. Source inspection, CPU checks and simulation results do not establish calibrated physical accuracy, a built assembly or real-world task success. The measurement and uncertainty procedures above remain proposals until their bounds are validated on the physical setup.
