"""Offline verification suite for rearrange-wall-wart-to-end-of-crowded-power-strip:
- Longitudinal 3-prong NEMA 5-15P to 3-slot NEMA 5-15R clearance (>= 1.5 mm clocked vs < -1.8 mm when 180 deg reversed)
- Wall-wart overhang collision: blocks Outlet 4 when in Outlet 3 (< -10 mm), frees Outlet 3 when moved to Outlet 4 (> +4 mm)
- Flush <= 1.5 mm zero-exposed-blade check (rejects >= 2.0 mm exposed-blade gaps; identical criterion in sim & real trials)
- All 5 live scoring tiers (0.00 / 0.25 / 0.50 / 0.75 / 1.00), existing-plug (> 2.5 mm) & strip-tip latches
- Printable STL fixture generation (fabricate.py) and 6-DoF IK dry_run (< 0.5 mm)
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import closedloop as cl  # noqa: E402
import fabricate  # noqa: E402
import mechkit as mk  # noqa: E402
import rearrange_wall_wart_to_end_of_crowded_power_strip as T  # noqa: E402
from mechkit import MechState  # noqa: E402


def _st(pos, tilt_deg: float = 0.0, joints: dict[str, float] | None = None) -> MechState:
    p = np.asarray(pos, dtype=float)
    ry = math.radians(tilt_deg) / 2.0
    q = np.array([0.0, math.sin(ry), 0.0, math.cos(ry)], dtype=float)
    z3 = np.zeros(3, dtype=float)
    return MechState(
        pos=p,
        origin=p,
        quat_xyzw=q,
        lin_vel=z3,
        ang_vel=z3,
        low=p - 0.05,
        high=p + 0.05,
        joints=dict(joints or {}),
    )


def test_nema_sockets_and_wall_wart_overhang_puzzle() -> None:
    strip_colliders = [s for s in T._power_strip_shapes() if s.collide]
    prong_colliders = list(T._nema_5_15p_prongs())

    # 1) Verify longitudinal 3-prong NEMA 5-15P clears all 4 outlets (>= +1.5 mm) at yaw = 0 deg,
    #    yet collides with the solid central dielectric bridge (< -1.8 mm) when 180 deg reversed!
    for yc in T.OUTLET_Y:
        pose_ok = (np.eye(3), np.array([0.0, yc, T.STRIP_H + 0.0002]))
        d_ok = min(mk.gap(ss, None, ps, pose_ok) for ss in strip_colliders for ps in prong_colliders)
        assert d_ok > 0.0015, f"Clocked 3-prong plug hits outlet {yc}: {d_ok:.4f}"

    pose_rev = (np.diag([-1.0, -1.0, 1.0]), np.array([0.0, T.OUTLET_Y[2], T.STRIP_H - 0.006]))
    d_rev = min(mk.gap(ss, None, ps, pose_rev) for ss in strip_colliders for ps in prong_colliders)
    assert d_rev < -0.0018, f"180-deg reversed plug failed to hit central bridge: {d_rev:.4f}"

    # 2) Verify WALL_WART in Outlet 3 physically blocks NEW_PLUG from entering Outlet 4 (< -10 mm),
    #    whereas moving WALL_WART to end Outlet 4 leaves > +4 mm clearance to NEW_PLUG in Outlet 3 AND PLUG_2 in Outlet 2!
    wart_shapes = [s for s in T._wall_wart_shapes() if s.collide]
    new_shapes = [s for s in T.NEW_PLUG.shapes if s.collide]
    p2_shapes = [s for s in T.PLUG_2.shapes if s.collide]

    pose_w3 = (np.eye(3), np.array([0.0, T.OUTLET_Y[2], T.STRIP_H + 0.0002]))
    pose_n4 = (np.eye(3), np.array([0.0, T.OUTLET_Y[3], T.STRIP_H + 0.0002]))
    d_blocked = min(mk.gap(ws, pose_w3, ns, pose_n4) for ws in wart_shapes for ns in new_shapes)
    assert d_blocked < -0.010, f"Wall-wart in Outlet 3 failed to block Outlet 4: {d_blocked:.4f}"

    pose_w4 = (np.eye(3), np.array([0.0, T.OUTLET_Y[3], T.STRIP_H + 0.0002]))
    pose_n3 = (np.eye(3), np.array([0.0, T.OUTLET_Y[2], T.STRIP_H + 0.0002]))
    pose_p2 = (np.eye(3), np.array([0.0, T.OUTLET_Y[1], T.STRIP_H + 0.0002]))
    d_w4_n3 = min(mk.gap(ws, pose_w4, ns, pose_n3) for ws in wart_shapes for ns in new_shapes)
    d_p2_n3 = min(mk.gap(ps, pose_p2, ns, pose_n3) for ps in p2_shapes for ns in new_shapes)
    assert d_w4_n3 > 0.0040, f"Wall-wart in Outlet 4 hits New Plug in Outlet 3: {d_w4_n3:.4f}"
    assert d_p2_n3 > 0.0060, f"Plug 2 hits New Plug in Outlet 3: {d_p2_n3:.4f}"


def test_live_scoring_ladder_and_zero_exposed_blade_rule() -> None:
    T.reset_episode_state()
    s0 = _st((T.STRIP_XY[0], T.STRIP_XY[1], T.MAT_TOP))
    p1 = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[0], T.MAT_TOP + T.STRIP_H + 0.0002))
    p2 = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[1], T.MAT_TOP + T.STRIP_H + 0.0002))
    w3 = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[2], T.MAT_TOP + T.STRIP_H + 0.0002))
    n_stow = _st((T.NEW_PLUG_STOW_XY[0], T.NEW_PLUG_STOW_XY[1], T.NEW_PLUG_SPAWN_Z))

    T.evaluate_step_state(s0, p1, p2, w3, n_stow, arms_released=True)
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w3, "new_plug": n_stow}) == 0.0

    # 1) Wall-wart lifted out of Outlet 3, Outlets 1 & 2 still seated -> 0.25
    w_lift = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[2], T.MAT_TOP + T.STRIP_H + 0.045))
    T.evaluate_step_state(s0, p1, p2, w_lift, n_stow, arms_released=False)
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w_lift, "new_plug": n_stow}) == 0.25

    # 2) Wall-wart reseated flush (<= 1.5 mm) in end Outlet 4, freeing Outlet 3 -> 0.50
    w4 = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[3], T.MAT_TOP + T.STRIP_H + 0.0002))
    T.evaluate_step_state(s0, p1, p2, w4, n_stow, arms_released=True)
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n_stow}) == 0.50

    # 3) Exposed-blade check: a plug hovering +2.5 mm above the strip face (exposed blades!) MUST NOT count as seated!
    n3_exposed = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[2], T.MAT_TOP + T.STRIP_H + 0.0025))
    assert T._plug_in_outlet(n3_exposed, s0, 2) is False
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n3_exposed}) == 0.50

    # 4) Black plug seated dead flush (<= 1.5 mm, zero exposed blades) in Outlet 3 -> 0.75 before dwell, 1.00 after 20 steps!
    n3 = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[2], T.MAT_TOP + T.STRIP_H + 0.0002))
    T.evaluate_step_state(s0, p1, p2, w4, n3, arms_released=False)
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n3}) == 0.75

    for _ in range(20):
        T.evaluate_step_state(s0, p1, p2, w4, n3, arms_released=True)
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n3}) == 1.00
    assert T.success({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n3}) is True

    # 5) Lifting an already-connected appliance (Plug 1 > 2.5 mm) latches existing_unplugged -> permanent 0.0!
    T.reset_episode_state()
    p1_out = _st((T.STRIP_XY[0], T.STRIP_XY[1] + T.OUTLET_Y[0], T.MAT_TOP + T.STRIP_H + 0.0040))
    T.evaluate_step_state(s0, p1_out, p2, w4, n3, arms_released=False)
    assert T._STATE["existing_unplugged"] is True
    assert T.score({"power_strip": s0, "plug_1": p1, "plug_2": p2, "wall_wart": w4, "new_plug": n3}) == 0.0


def test_fabricate_and_closedloop_dry_run() -> None:
    with tempfile.TemporaryDirectory() as td:
        written = fabricate.write_all(td)
        assert len(written) == 3
        for p in written.values():
            assert os.path.getsize(p) > 200

    res = cl.dry_run(os.path.join(_DIR, "rearrange_wall_wart_to_end_of_crowded_power_strip.py"))
    assert res["steps"] <= 560, f"Too many steps: {res['steps']}"
    assert res["worst_ik_mm"] < 0.5, f"IK error too large: {res['worst_ik_mm']}"
    assert res["worst_ik_deg"] < 0.5, f"IK deg error too large: {res['worst_ik_deg']}"
    assert res["state_reads"] == 0
    print(f"dry_run summary: steps={res['steps']}, worst_ik_mm={res['worst_ik_mm']:.3f}, worst_ik_deg={res['worst_ik_deg']:.3f}")


if __name__ == "__main__":
    test_nema_sockets_and_wall_wart_overhang_puzzle()
    test_live_scoring_ladder_and_zero_exposed_blade_rule()
    test_fabricate_and_closedloop_dry_run()
    print("ALL 3 CROWDED POWER STRIP WALL-WART REARRANGEMENT TEST SUITES PASSED CLEANLY!")
