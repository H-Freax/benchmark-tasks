"""A loose glass bottle with a narrow neck stands at (0.30, 0) m; a bottle brush lies on a tray at y = -0.18 m.

Orange film marks sit on the bottle's inner bottom and on the inside of its shoulder, behind the neck. Brush
them off with the brush head, take the brush out and put both items down. Any grasp, support or placement
is allowed; a mark only comes off where the brush head touches the glass and slides across it.

Run this file directly for the offline check (no simulator):  python task.py
"""
from pathlib import Path
import os
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "native"))
import bottle_scene as scene  # noqa: E402
from bottle_scene import CONTROL_HZ, OBJECTS, SCENE, SCORE_OVER, SUCCESS_HOLD_S, score, success  # noqa: E402,F401


def demo():
    """Scripted reference for ``--policy yam_scripted``. BOTTLE_DEMO=bottom_only or leave_brush_in plays a
    deliberately incomplete variant; the scorer does not read the reference's phases."""
    import reference

    return reference.install(scene, os.environ.get("BOTTLE_DEMO", "full"))


def _selftest():
    import math

    import numpy as np
    from scipy.spatial.transform import Rotation

    import reference
    from cleaning import Film
    from grader import final_state

    G = scene.G
    table = G["table_top_m"]
    bottle = [0.30, 0.0, table, 0.0, 0.0, 0.0, 1.0]
    cases = 0

    def check(name, got, want):
        nonlocal cases
        cases += 1
        if isinstance(want, float):
            assert abs(got - want) < 1e-9, f"{name}: got {got}, expected {want}"
        else:
            assert got == want, f"{name}: got {got}, expected {want}"

    # -- film model: marks come off only under head contact with relative sliding ---------------------------------
    def brush_at(x, y, z):
        return [0.30 + x, y, table + z, 0.0, 0.0, 0.0, 1.0]

    def run(path, force=1.0, contact_z=0.02):
        film = Film(G)
        for serial, (x, y, z) in enumerate(path):
            pose = brush_at(x, y, z)
            point = [pose[0], pose[1], pose[2] + contact_z]
            film.update(dict(serial=serial, bottle_pose=bottle, brush_pose=pose,
                             native_contacts=[dict(normal_force_N=force, world_point_m=point)] if force else []))
        return film.coverage()

    circle = [(0.007 * math.cos(a), 0.007 * math.sin(a), 0.0025) for a in np.linspace(0, 6 * math.pi, 151)]
    check("bottom circles clean the bottom", run(circle)["bottom"], 1.0)
    check("bottom circles leave the shoulder", run(circle)["shoulder"], 0.0)
    check("no contact removes nothing", run(circle, force=0.0)["all"], 0.0)
    check("contact under 0.1 N removes nothing", run(circle, force=0.05)["all"], 0.0)
    check("stem-only contact removes nothing", run(circle, contact_z=0.10)["all"], 0.0)
    check("resting on the bottom without sliding removes nothing", run([(0.0, 0.0, 0.0025)] * 200)["all"], 0.0)
    check("circling 20 mm above the bottom removes nothing", run([(x, y, 0.024) for x, y, _ in circle])["all"], 0.0)
    one_turn = [(0.007 * math.cos(a), 0.007 * math.sin(a), 0.0025) for a in np.linspace(0, 2 * math.pi, 51)]
    assert 0.0 < run(one_turn)["bottom"] < 1.0  # one circle only clears part of the bottom
    cases += 1

    # -- end state ------------------------------------------------------------------------------------------------
    far = np.array([[0.0, 0.3, 1.0]])
    brush_part = next(p for p in G["parts"] if p["name"] == "bottle_brush")
    tray = brush_part["pos"] + brush_part["quat_xyzw"]
    still = [0.0] * 6

    def state(bottle_pose=bottle, brush_pose=tray, hand=far, bottle_vel=still, brush_vel=still):
        return final_state(G, dict(feeding_bottle=dict(pose=bottle_pose, velocity=bottle_vel),
                                   bottle_brush=dict(pose=brush_pose, velocity=brush_vel)), hand)

    good = dict(withdrawn=True, at_rest=True, standing=True, hands_clear=True)
    check("brush on its tray, bottle standing", state(), good)
    check("brush head inside", state(brush_pose=brush_at(0, 0, 0.05)), dict(good, withdrawn=False))
    check("brush head still in the neck", state(brush_pose=brush_at(0, 0, 0.12)), dict(good, withdrawn=False))
    check("brush head just above the mouth", state(brush_pose=brush_at(0, 0, 0.145)), good)
    check("hand next to the bottle", state(hand=np.array([[0.30, 0.05, 0.80]])), dict(good, hands_clear=False))
    check("brush still moving", state(brush_vel=[0.03, 0, 0, 0, 0, 0]), dict(good, at_rest=False))
    tipped = bottle[:3] + Rotation.from_euler("x", 12, degrees=True).as_quat().tolist()
    check("bottle leaning 12 deg", state(bottle_pose=tipped)["standing"], False)
    lying = [0.30, 0.0, table + 0.0325] + Rotation.from_euler("x", 90, degrees=True).as_quat().tolist()
    check("bottle on its side", state(bottle_pose=lying)["standing"], False)

    # -- stage scores ---------------------------------------------------------------------------------------------
    def row(bottom, shoulder, withdrawn=False, released=False, stable=0.0):
        total = (bottom * 24 * 25 + shoulder * 24 * 16) / (24 * 25 + 24 * 16)
        return dict(coverage={"all": total, "bottom": bottom, "shoulder": shoulder},
                    state=dict(withdrawn=withdrawn, released=released), stable_s=stable)

    table_of_scores = [
        ("nothing brushed", row(0, 0), 0.0),
        ("nothing brushed, both items put down", row(0, 0, True, True, 5.0), 0.0),
        ("bottom 30 %", row(0.3, 0), 0.0),
        ("bottom half", row(0.5, 0), 0.15),
        ("bottom done", row(1.0, 0), 0.30),
        ("bottom done, shoulder skipped, all put down", row(1.0, 0, True, True, 5.0), 0.30),
        ("bottom done, shoulder half", row(1.0, 0.5), 0.45),
        ("both done, brush still inside", row(1.0, 1.0), 0.60),
        ("both done, brush left inside and let go", row(1.0, 1.0, False, True, 5.0), 0.60),
        ("both at 80 % but under 90 % overall, withdrawn", row(0.8, 0.8, True, True, 5.0), 0.60),
        ("both done, brush out, still held", row(1.0, 1.0, True), 0.80),
        ("both done, put down for 1 s", row(1.0, 1.0, True, True, 1.0), 0.80),
        ("both done, put down for 2 s", row(1.0, 1.0, True, True, 2.0), 1.0),
        ("bottom 100 %, shoulder 83 %, put down", row(1.0, 0.84, True, True, 3.0), 1.0),
    ]
    for name, r, want in table_of_scores:
        check(name, scene.stage_score(r), want)
    check("no observation yet", scene.stage_score(None), 0.0)
    print(f"scoring: {cases} cases pass")

    # -- the reference on the kinematics alone ----------------------------------------------------------------------
    held = [0.28, -0.001, table + 0.0012] + Rotation.from_euler("y", 2, degrees=True).as_quat().tolist()
    for variant, pose in (("full", bottle), ("full", held), ("bottom_only", bottle), ("leave_brush_in", bottle)):
        r = reference.dry_run(G, variant, pose)
        c = r["coverage"]
        where = "nominal bottle" if pose is bottle else "bottle shifted 20 mm and tilted 2 deg in the hand"
        print(f"{variant} ({where}): {r['steps']} steps = {r['seconds']:.1f} s, worst IK {r['worst_ik_mm']:.2f} mm / "
              f"{r['worst_ik_deg']:.2f} deg, path coverage bottom {c['bottom']:.2f} shoulder {c['shoulder']:.2f}")
        assert r["within_joint_limits"] and r["worst_ik_mm"] < 1.0 and r["worst_ik_deg"] < 0.5, r
        assert r["steps"] <= 2600, r
        if variant == "bottom_only":
            assert c["bottom"] >= 0.8 and c["shoulder"] == 0.0, c
        else:
            assert c["bottom"] >= 0.8 and c["shoulder"] >= 0.8 and c["all"] >= 0.9, c


if __name__ == "__main__":
    _selftest()
