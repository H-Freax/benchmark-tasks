"""Scene, per-step observation and scoring for the narrow-neck bottle brushing task.

Pure Python at import time; Isaac Sim is only touched from the per-step callbacks. Nothing here writes an
object pose or velocity, applies a force, attaches anything to the robot or changes a robot drive.
"""
from pathlib import Path
import json
import os
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import native_geometry as ng  # noqa: E402
from cleaning import Film, transform  # noqa: E402
from grader import final_state  # noqa: E402
from pad_contacts import PadObserver  # noqa: E402

G = json.loads((HERE.parent / "geometry" / "geometry.json").read_text())
OBJECTS = ng.objects(G)
SCENE = "station"
CONTROL_HZ = 20.0
SUCCESS_HOLD_S = 0.1
SCORE_OVER = "end"

REGION_FULL = 0.80      # removed share of a region's marked area for its full stage credit
REGION_HALF = 0.40      # ... and for half of it
TOTAL_FULL = 0.90       # removed share of all marked area needed, with both regions, for the last two stages
RELEASED_HOLD_S = 2.0   # both items let go and at rest this long for a full score

# Optional per-step trace (large): set BOTTLE_TRACE_DIR to a directory to write it.
TRACE = Path(os.environ["BOTTLE_TRACE_DIR"]) if os.environ.get("BOTTLE_TRACE_DIR") else None

FILM = Film(G)
PADS = PadObserver()
SERIAL = 0
VIEW = None
LATEST = None
STABLE_START = None
LAST_PHYSICS_TICK = None
ACTION_CONTROL = 0


def arr(v):
    v = getattr(v, "torch", v)
    if hasattr(v, "detach"):
        return v.detach().cpu().numpy().copy()
    if hasattr(v, "numpy"):
        return v.numpy().copy()
    return np.asarray(v)


def write(name, value):
    if TRACE is None:
        return
    TRACE.mkdir(parents=True, exist_ok=True)
    (TRACE / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def body(base, name):
    view = base.scene[name].root_view
    pose = arr(view.get_transforms()).reshape(-1, 7)[0].tolist()
    velocity = arr(view.get_velocities()).reshape(-1, 6)[0].tolist()
    transform(pose)
    if not np.isfinite(velocity).all():
        raise RuntimeError("non-finite body velocity")
    return dict(pose=pose, velocity=velocity)


def robot_state(base, side):
    robot = base.scene[side + "_robot"]
    view = robot.root_view
    q = arr(view.get_dof_positions()).reshape(-1, 8)[0]
    links = arr(view.get_link_transforms()).reshape(-1, 9, 7)[0]
    if not np.isfinite(np.r_[q, links.reshape(-1)]).all():
        raise RuntimeError("non-finite robot readback")
    return dict(joint_names=list(robot.backend_joint_names), body_names=list(robot.backend_body_names), q=q, links=links)


def gripper_points(env):
    """Six points on each gripper (palm, pad centre, both pad tips and pad middles) and its opening."""
    base = env.unwrapped
    out = {}
    for side in ("left", "right"):
        robot = robot_state(base, side)
        g = robot["links"][robot["body_names"].index("gripper")]
        R = transform(g)[:3, :3]
        p = np.array(g[:3])
        half = abs(float(robot["q"][robot["joint_names"].index("joint7")])) + 0.008
        local = [(0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147), (-half, 0, 0.147), (half, 0, 0.12), (-half, 0, 0.12)]
        out[side] = ([p + R @ np.array(v, float) for v in local], 2 * half - 0.016)
    return out


def partkit_pose(env, name):
    base = env.unwrapped
    v = base.scene[name].root_view
    pose = arr(v.get_transforms()).reshape(-1, 7)[0]
    velocity = arr(v.get_velocities()).reshape(-1, 6)[0]
    origin = arr(base.scene.env_origins).reshape(-1, 3)[0]
    return pose[:3] - origin, pose[3:7], velocity[:3]


ng.pk.gripper_points = gripper_points
ng.pk.pose = partkit_pose


def _mark_prim(base, patch):
    root = base.scene.env_prim_paths[0]
    return base.scene.stage.GetPrimAtPath(root + f"/feeding_bottle/body/geom_{patch['geom_index']}")


@ng.pk.on_reset
def reset(env):
    """Reset the film counters and show every mark again. No body pose or velocity is written here."""
    global FILM, SERIAL, VIEW, LATEST, STABLE_START, LAST_PHYSICS_TICK, PADS, ACTION_CONTROL
    FILM = Film(G)
    PADS = PadObserver()
    SERIAL = 0
    VIEW = None
    LATEST = None
    STABLE_START = None
    LAST_PHYSICS_TICK = None
    ACTION_CONTROL = 0
    from pxr import UsdGeom

    base = env.unwrapped
    for patch in G["patches"]:
        prim = _mark_prim(base, patch)
        if prim:
            UsdGeom.Imageable(prim).MakeVisible()


def bind(base):
    """Create the brush-bottle contact view and check the authored bodies, colliders and head compliance."""
    global VIEW
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import PhysxSchema, Usd, UsdPhysics

    root = base.scene.env_prim_paths[0]
    stage = base.scene.stage
    sensor = root + "/bottle_brush/body"
    target = root + "/feeding_bottle/body"
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        [sensor], filter_patterns=[[target]], max_contact_data_count=4096
    )
    if view is None:
        raise RuntimeError("brush-bottle contact view unavailable")
    inventory = {}
    for p in G["parts"]:
        v = base.scene[p["name"]].root_view
        expected = sum(s["collision_enabled"] for s in p["geoms"])
        actual = int(v.max_shapes)
        if int(v.count) != 1 or actual != expected:
            raise RuntimeError(f"collider count mismatch on {p['name']}: {actual}/{expected}")
        inventory[p["name"]] = dict(shape_count=actual, mass_kg=arr(v.get_masses()).reshape(-1).tolist())
    shapes = [
        str(prim.GetPath())
        for prim in stage.Traverse(Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.CollisionAPI)
        and any(str(prim.GetPath()).startswith(root + "/" + p["name"] + "/") for p in G["parts"])
    ]
    if len(shapes) != sum(sum(s["collision_enabled"] for s in p["geoms"]) for p in G["parts"]):
        raise RuntimeError("collider inventory differs from the geometry file")
    material = PhysxSchema.PhysxMaterialAPI(stage.GetPrimAtPath(root + "/bottle_brush/Looks/material_0"))
    compliance = dict(
        stiffness_N_m=material.GetCompliantContactStiffnessAttr().Get(),
        damping_Ns_m=material.GetCompliantContactDampingAttr().Get(),
    )
    if compliance["stiffness_N_m"] != G["brush"]["compliance"]["stiffness_N_m"]:
        raise RuntimeError("brush head compliance differs from the geometry file")
    write("scene-inventory.json", dict(bodies=inventory, shapes=shapes, head_compliance=compliance, task_joints=0))
    VIEW = view


def observe(env):
    """Once per control step: read poses and contacts, advance the film model, evaluate the end state."""
    global SERIAL, LATEST, STABLE_START, LAST_PHYSICS_TICK
    base = env.unwrapped
    tick = int(base._sim_step_counter)
    if LAST_PHYSICS_TICK == tick:
        return
    if LAST_PHYSICS_TICK is not None and tick < LAST_PHYSICS_TICK:
        raise RuntimeError("physics clock went backwards")
    LAST_PHYSICS_TICK = tick
    SERIAL += 1
    if VIEW is None:
        bind(base)
    bodies = {p["name"]: body(base, p["name"]) for p in G["parts"]}
    force, position, normal, separation, counts, starts = [arr(v) for v in VIEW.get_contact_data(dt=base.physics_dt)]
    if not np.isfinite(counts).all() or not np.isfinite(starts).all():
        raise RuntimeError("invalid contact extent")
    extent = int(np.max(counts + starts)) if counts.size else 0
    if extent > len(force):
        raise RuntimeError("truncated contact buffer")
    contacts = [
        dict(
            normal_force_N=float(force.reshape(-1)[i]),
            world_point_m=position.reshape(-1, 3)[i].tolist(),
            world_normal=normal.reshape(-1, 3)[i].tolist(),
            separation_m=float(separation.reshape(-1)[i]),
        )
        for i in range(extent)
    ]
    row = dict(
        serial=SERIAL,
        control=ACTION_CONTROL,
        time_s=tick * float(base.physics_dt),
        bottle_pose=bodies["feeding_bottle"]["pose"],
        brush_pose=bodies["bottle_brush"]["pose"],
        bodies=bodies,
        native_contacts=contacts,
    )
    result = FILM.update(row)
    coverage = FILM.coverage()
    row["material_update"] = result
    row["coverage"] = coverage
    from pxr import UsdGeom

    for i in result["removed"]:
        UsdGeom.Imageable(_mark_prim(base, G["patches"][i])).MakeInvisible()
    points = np.array([p for ps, _ in gripper_points(env).values() for p in ps])
    pads = PADS.observe(base, TRACE)
    # Every one of the 18 robot bodies must report zero contact with both items; a missing reading is not zero.
    untouched = all(v["count"] == 0 for pairs in pads["pairs"].values() for v in pairs.values()) and not np.asarray(
        pads["tangent_counts"]
    ).any()
    state = final_state(G, bodies, points)
    state["untouched"] = bool(untouched)
    state["released"] = bool(state["at_rest"] and state["standing"] and state["hands_clear"] and untouched)
    if is_cleaned(coverage) and state["withdrawn"] and state["released"]:
        if STABLE_START is None:
            STABLE_START = row["time_s"]
    else:
        STABLE_START = None
    row["state"] = state
    row["stable_s"] = 0.0 if STABLE_START is None else row["time_s"] - STABLE_START
    row["pad_contacts"] = pads
    row["hand_points"] = points.tolist()
    row["robots"] = {
        side: {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in robot_state(base, side).items()}
        for side in ("left", "right")
    }
    LATEST = row
    if TRACE is not None:
        TRACE.mkdir(parents=True, exist_ok=True)
        with (TRACE / "observations.jsonl").open("a") as f:
            f.write(json.dumps(row, allow_nan=False) + "\n")
    if SERIAL % 20 == 0:
        summary = dict(control=ACTION_CONTROL, coverage=coverage, state=state, score=stage_score(row))
        print("BOTTLE " + json.dumps(summary), flush=True)


def is_cleaned(coverage):
    return bool(
        coverage["bottom"] >= REGION_FULL and coverage["shoulder"] >= REGION_FULL and coverage["all"] >= TOTAL_FULL
    )


def stage_score(row):
    """Partial credit per finished stage, from the latest observation only.

    Each region (inner bottom, inner shoulder): 0.15 at 40 % of its marked area removed, 0.30 at 80 %.
    Both regions at 80 % and 90 % overall, with the brush head out of the bottle: 0.80.
    Also both items let go and at rest with the bottle upright for 2 s: 1.0.
    """
    if row is None:
        return 0.0
    c = row["coverage"]
    score = 0.0
    for kind in ("bottom", "shoulder"):
        score += 0.30 if c[kind] >= REGION_FULL else 0.15 if c[kind] >= REGION_HALF else 0.0
    if is_cleaned(c) and row["state"]["withdrawn"]:
        score = 0.80
        if row["state"]["released"] and row["stable_s"] >= RELEASED_HOLD_S:
            score = 1.0
    return score


def score(states):
    return stage_score(LATEST)


def success(states):
    return score(states) == 1.0


@ng.pk.on_step
def observe_fail_closed(env):
    """A failed readback stops the run instead of scoring on missing data."""
    try:
        observe(env)
    except Exception as error:
        import traceback

        write("first-observer-error.json", dict(exception=repr(error), traceback=traceback.format_exc()))
        raise


# Count issued actions separately from observations (the reset's settling steps are not actions).
_previous_inspect = ng.pk._Finder.targets["yam_arena.inspect_embodiment"]


def _patch_action_clock(mod):
    _previous_inspect(mod)
    cls = mod.YamArenaEmbodiment
    if getattr(cls, "_bottle_action_clock", False):
        return
    old_step = cls.step

    def step(self, action):
        global ACTION_CONTROL
        ACTION_CONTROL += 1
        return old_step(self, action)

    cls.step = step
    cls._bottle_action_clock = True


ng.pk._Finder.targets["yam_arena.inspect_embodiment"] = _patch_action_clock
if "yam_arena.inspect_embodiment" in sys.modules:
    _patch_action_clock(sys.modules["yam_arena.inspect_embodiment"])

# Some runtimes need the non-UJITSO collision cooking backend; BOTTLE_LEGACY_COOKING=1 selects it. Geometry,
# drives and physics are unchanged.
if os.environ.get("BOTTLE_LEGACY_COOKING") == "1":
    from cooking_local import configure

    _previous_scene = ng.pk._Finder.targets["yam_arena.scene"]

    def _patch_scene(mod):
        configure(TRACE)
        _previous_scene(mod)

    ng.pk._Finder.targets["yam_arena.scene"] = _patch_scene
