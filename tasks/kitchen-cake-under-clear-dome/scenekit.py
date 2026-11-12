"""Procedural multi-shape parts (glass dome shell, wedge layers, rings) and per-step/reset hooks for the task.

Patches yam_arena's scene loader in memory so a ``Part`` (a rigid body built from several components) can be
listed in ``OBJECTS``. All Isaac/Kit imports stay lazy.
"""

from __future__ import annotations

import hashlib
import importlib.abc
import importlib.util
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
GLASS_MDL_BINDING = {
    "schema_version": "isaaclab-glass-mdl-binding-1.0",
    "backend": "isaaclab.sim.GlassMdlCfg",
    "mdl_path": "OmniGlass.mdl",
    "glass_color": (0.95, 0.98, 1.0),
    "frosting_roughness": 0.0,
    "thin_walled": False,
    "glass_ior": 1.491,
}
DOME_PROFILE = (
    (0.00, 0.91),
    (0.08, 0.98),
    (0.35, 0.99),
    (0.62, 0.94),
    (0.82, 0.84),
    (0.95, 0.55),
    (1.00, 0.16),
)
DOME_WALL_THICKNESS_M = 0.003
_MODULE_PATH = Path(__file__).resolve()
MODULE_TOKEN = hashlib.sha256(str(_MODULE_PATH).encode() + b"\0" + _MODULE_PATH.read_bytes()).hexdigest()
_CALLBACKS: dict[str, dict[tuple[str, str], Callable]] = {"reset": {}, "step": {}}
_SPECS_BY_KEY: dict[str, Part] = {}
_ENV_SPECS: dict[int, dict[str, Part]] = {}


@dataclass(frozen=True)
class Component:
    """One semantic component of an original procedural asset."""

    name: str
    kind: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color
    opacity: float = 1.0
    collide: bool = True
    axis: str = "Z"
    start_angle_degrees: float = 0.0
    sweep_angle_degrees: float = 0.0

    def __post_init__(self) -> None:
        allowed = {
            "box",
            "cylinder",
            "capsule",
            "wedge",
            "cake_sector",
            "server_blade",
            "dome_shell",
            "ring",
        }
        if not self.name.isidentifier() or self.kind not in allowed:
            raise ValueError(f"invalid component {self.name!r}/{self.kind!r}")
        if len(self.center) != 3 or len(self.size) != 3 or any(not math.isfinite(v) or v <= 0 for v in self.size):
            raise ValueError(f"{self.name}: center/size must be finite positive 3-vectors")
        if self.axis not in {"X", "Y", "Z"} or not 0.0 < self.opacity <= 1.0:
            raise ValueError(f"{self.name}: invalid axis/opacity")
        angular = self.kind == "cake_sector"
        if angular:
            if (
                not math.isfinite(self.start_angle_degrees)
                or not 0.0 < self.sweep_angle_degrees < 360.0
            ):
                raise ValueError(
                    f"{self.name}: cake sectors need a finite start and 0 < sweep < 360 degrees"
                )
        elif self.start_angle_degrees != 0.0 or self.sweep_angle_degrees != 0.0:
            raise ValueError(f"{self.name}: angular fields are reserved for cake sectors")


@dataclass(frozen=True)
class Part(Box):
    """A fixed or loose coherent original asset with conservative bounds."""

    profile: str = ""
    components: tuple[Component, ...] = ()
    sliders: tuple = ()
    friction: float = 0.8
    linear_damping: float = 0.15
    angular_damping: float = 0.6

    def _check(self) -> None:
        super()._check()
        if not self.profile or not self.components or self.sliders:
            raise ValueError(f"{self.name}: profile/components required; sliders unsupported")
        if self.friction <= 0 or len({item.name for item in self.components}) != len(self.components):
            raise ValueError(f"{self.name}: invalid friction or duplicate component")


@dataclass(frozen=True)
class PartState(ObjectState):
    """Root pose, velocity, conservative bounds, and local/world transforms."""

    sliders: dict[str, float] | None = None

    def rotation(self) -> np.ndarray:
        x, y, z, w = (float(value) for value in self.quat_xyzw)
        norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    def to_world(self, local_point) -> np.ndarray:
        return np.asarray(self.pos, dtype=float) + self.rotation() @ np.asarray(local_point, dtype=float)

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, dtype=float) - np.asarray(self.pos, dtype=float))


def component(
    name,
    kind,
    center,
    size,
    color,
    *,
    opacity=1.0,
    collide=True,
    axis="Z",
    start_angle_degrees=0.0,
    sweep_angle_degrees=0.0,
) -> Component:
    return Component(
        name,
        kind,
        tuple(center),
        tuple(size),
        tuple(color),
        float(opacity),
        bool(collide),
        axis,
        float(start_angle_degrees),
        float(sweep_angle_degrees),
    )


def vertical_interval(item: Component) -> tuple[float, float]:
    """Return the exact local lower/upper surfaces of a component."""
    half_height = item.size[2] / 2.0
    return item.center[2] - half_height, item.center[2] + half_height


def wedge_profile_xy(item: Component) -> tuple[tuple[float, float], ...]:
    """Return one pointed triangular cake-wedge footprint."""
    if item.kind != "wedge":
        raise ValueError(f"expected wedge component, got {item.kind!r}")
    cx, cy, _ = item.center
    length, width, _ = item.size
    return (
        (cx - length / 2.0, cy),
        (cx + length / 2.0, cy + width / 2.0),
        (cx + length / 2.0, cy - width / 2.0),
    )


def sector_profile_xy(
    item: Component, *, arc_segments: int = 12
) -> tuple[tuple[float, float], ...]:
    """Return a centre-to-arc footprint for a true radial cake sector."""
    if item.kind != "cake_sector":
        raise ValueError(f"expected radial cake component, got {item.kind!r}")
    if isinstance(arc_segments, bool) or not isinstance(arc_segments, int) or arc_segments < 2:
        raise ValueError("cake-sector arc_segments must be an integer of at least two")
    cx, cy, _ = item.center
    radius = item.size[0] / 2.0
    angles = np.linspace(
        math.radians(item.start_angle_degrees),
        math.radians(item.start_angle_degrees + item.sweep_angle_degrees),
        arc_segments + 1,
    )
    return (
        (cx, cy),
        *((cx + radius * math.cos(angle), cy + radius * math.sin(angle)) for angle in angles),
    )


def _sector_prism_geometry(
    item: Component,
    *,
    start_angle_degrees: float,
    sweep_angle_degrees: float,
    arc_segments: int,
) -> dict[str, object]:
    cx, cy, _ = item.center
    radius = item.size[0] / 2.0
    low, high = vertical_interval(item)
    angles = np.linspace(
        math.radians(start_angle_degrees),
        math.radians(start_angle_degrees + sweep_angle_degrees),
        arc_segments + 1,
    )
    profile = [(cx, cy)] + [
        (cx + radius * math.cos(angle), cy + radius * math.sin(angle)) for angle in angles
    ]
    points = [(x, y, z) for z in (low, high) for x, y in profile]
    count = len(profile)
    bottom = list(range(count))[::-1]
    top = list(range(count, 2 * count))
    counts = [count, count]
    indices = [*bottom, *top]
    groups = ["bottom", "top"]
    for index in range(count):
        nxt = (index + 1) % count
        counts.append(4)
        indices.extend((index, nxt, count + nxt, count + index))
        groups.append("side")
    return {
        "points": points,
        "face_vertex_counts": counts,
        "face_vertex_indices": indices,
        "face_groups": groups,
        "bottom_face_indices": list(range(count)),
        "top_face_indices": top,
        "start_angle_degrees": float(start_angle_degrees),
        "sweep_angle_degrees": float(sweep_angle_degrees),
        "radius_m": radius,
        "height_interval_m": [low, high],
        "closed_prism": True,
        "connected_component_count": 1,
    }


def sector_prism_geometry(item: Component, *, arc_segments: int = 12) -> dict[str, object]:
    """Build the exact closed visible/collision prism for one removable sector."""
    if item.kind != "cake_sector":
        raise ValueError(f"expected cake_sector component, got {item.kind!r}")
    return _sector_prism_geometry(
        item,
        start_angle_degrees=item.start_angle_degrees,
        sweep_angle_degrees=item.sweep_angle_degrees,
        arc_segments=arc_segments,
    )


def validate_vertical_stack(
    items: tuple[Component, ...], *, minimum_clearance_m: float
) -> tuple[tuple[float, float], ...]:
    """Fail closed when authored wedge layers overlap or share coplanar faces."""
    if not items or minimum_clearance_m <= 0.0:
        raise ValueError("a nonempty stack and positive face clearance are required")
    if any(item.kind not in {"wedge", "cake_sector"} for item in items):
        raise ValueError("every stacked cake layer must be a radial or wedge prism")
    intervals = tuple(vertical_interval(item) for item in items)
    for lower, upper in zip(intervals[:-1], intervals[1:], strict=True):
        clearance = upper[0] - lower[1]
        if clearance < minimum_clearance_m:
            raise ValueError(
                f"stacked wedge faces need {minimum_clearance_m} m clearance, got {clearance}"
            )
    return intervals


def srgb_to_stage_rgb(color: Color) -> Color:
    """Use the renderer encoding selected by the exact rendered A/B diagnostic."""
    return tuple(
        channel / 12.92
        if channel <= 0.04045
        else ((channel + 0.055) / 1.055) ** 2.4
        for channel in color
    )


def preview_surface_properties(color: Color, opacity: float) -> dict[str, Color | float]:
    """Return the renderer-facing PreviewSurface values for one component."""
    translucent = opacity < 1.0
    return {
        "diffuse_color": color,
        "roughness": 0.04 if translucent else 0.52,
        "metallic": (
            0.0
            if translucent
            else 0.72
            if color[2] > 0.45 and color[0] < 0.7
            else 0.0
        ),
        "opacity": float(opacity),
    }


def visual_material_binding(item: Component) -> dict[str, object]:
    """Describe the explicit renderer binding for one visible component."""
    if item.kind == "dome_shell":
        if item.opacity != 1.0:
            raise ValueError("native dielectric dome glass forbids PreviewSurface opacity fallback")
        return dict(GLASS_MDL_BINDING)
    return {
        "schema_version": "usd-preview-surface-binding-1.0",
        "backend": "isaaclab.sim.PreviewSurfaceCfg",
        **preview_surface_properties(item.color, item.opacity),
    }


def visual_material_config(sim_utils, item: Component, *, linearize=None):
    """Create the selected Isaac Lab material config without a glass fallback."""
    binding = visual_material_binding(item)
    if binding["backend"] == "isaaclab.sim.GlassMdlCfg":
        return sim_utils.GlassMdlCfg(
            mdl_path=binding["mdl_path"],
            glass_color=binding["glass_color"],
            frosting_roughness=binding["frosting_roughness"],
            thin_walled=binding["thin_walled"],
            glass_ior=binding["glass_ior"],
        )
    encode = srgb_to_stage_rgb if linearize is None else linearize
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=encode(binding["diffuse_color"]),
        roughness=binding["roughness"],
        metallic=binding["metallic"],
        opacity=binding["opacity"],
    )


def dome_shell_geometry(item: Component, *, sides: int = 24) -> dict[str, object]:
    """Build one closed material volume around an open-bottom dome cavity."""
    if item.kind != "dome_shell":
        raise ValueError(f"expected dome_shell component, got {item.kind!r}")
    if isinstance(sides, bool) or not isinstance(sides, int) or sides < 3:
        raise ValueError("dome shell sides must be an integer of at least three")

    cx, cy, cz = item.center
    radius, height = item.size[0] / 2.0, item.size[2]
    bottom = cz - height / 2.0
    profile_z = tuple(z_value for z_value, _ in DOME_PROFILE)
    outer_radii = tuple(radius * radius_scale for _, radius_scale in DOME_PROFILE)
    inner_radii = tuple(value - DOME_WALL_THICKNESS_M for value in outer_radii)
    if min(inner_radii) <= 0.0:
        raise ValueError("dome profile is too small for the declared 3 mm wall")

    points: list[tuple[float, float, float]] = []
    for radial_profile in (outer_radii, inner_radii):
        for z_value, ring_radius in zip(profile_z, radial_profile, strict=True):
            for index in range(sides):
                angle = 2.0 * math.pi * index / sides
                points.append(
                    (
                        cx + ring_radius * math.cos(angle),
                        cy + ring_radius * math.sin(angle),
                        bottom + height * z_value,
                    )
                )

    rings = len(DOME_PROFILE)
    counts: list[int] = []
    indices: list[int] = []
    groups: list[str] = []

    def add_face(vertices: list[int], group: str) -> None:
        counts.append(len(vertices))
        indices.extend(vertices)
        groups.append(group)

    for layer, group in ((0, "outer"), (1, "inner")):
        base = layer * rings * sides
        for band in range(rings - 1):
            for index in range(sides):
                nxt = (index + 1) % sides
                face = [
                    base + band * sides + index,
                    base + band * sides + nxt,
                    base + (band + 1) * sides + nxt,
                    base + (band + 1) * sides + index,
                ]
                add_face(face if layer == 0 else face[::-1], group)

    outer_base, inner_base = 0, rings * sides
    for index in range(sides):
        nxt = (index + 1) % sides
        add_face(
            [
                outer_base + index,
                inner_base + index,
                inner_base + nxt,
                outer_base + nxt,
            ],
            "bottom_annulus",
        )
    top_ring = rings - 1
    for index in range(sides):
        nxt = (index + 1) % sides
        add_face(
            [
                outer_base + top_ring * sides + index,
                outer_base + top_ring * sides + nxt,
                inner_base + top_ring * sides + nxt,
                inner_base + top_ring * sides + index,
            ],
            "top_annulus",
        )

    return {
        "points": points,
        "face_vertex_counts": counts,
        "face_vertex_indices": indices,
        "face_groups": groups,
        "profile_z": profile_z,
        "outer_radii_m": outer_radii,
        "inner_radii_m": inner_radii,
        "wall_thickness_m": DOME_WALL_THICKNESS_M,
        "sides": sides,
    }


def part(name, *, pos, size, profile, components, mass=None, yaw=0.0, friction=0.8) -> Part:
    return Part(
        name=name,
        pos=tuple(pos),
        size=tuple(size),
        mass=None if mass is None else float(mass),
        color=(0.5, 0.5, 0.5),
        yaw=float(yaw),
        profile=str(profile),
        components=tuple(components),
        friction=float(friction),
    )


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec = _SPECS_BY_KEY[cfg.spec_key]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0, 0, 0, 1)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    physics_path = f"{prim_path}/Looks/physics"
    physics = sim_utils.RigidBodyMaterialCfg(
        static_friction=spec.friction,
        dynamic_friction=max(0.05, spec.friction * 0.85),
        friction_combine_mode="max",
        restitution=0.02,
    )
    physics.func(physics_path, physics)
    looks: dict[tuple[tuple[str, object], ...], str] = {}

    def look(item: Component) -> str:
        binding = visual_material_binding(item)
        key = tuple(binding.items())
        if key not in looks:
            suffix = "glass_mdl_v1" if item.kind == "dome_shell" else f"material_{len(looks)}"
            path = f"{prim_path}/Looks/{suffix}"
            material = visual_material_config(sim_utils, item)
            material.func(path, material)
            looks[key] = path
        return looks[key]

    body_path = f"{prim_path}/body"
    body = UsdGeom.Xform.Define(stage, body_path).GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(spec.mass)
        props = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        props.CreateMaxDepenetrationVelocityAttr(1.0)
        props.CreateSolverPositionIterationCountAttr(32)
        props.CreateSolverVelocityIterationCountAttr(4)
        props.CreateLinearDampingAttr(spec.linear_damping)
        props.CreateAngularDampingAttr(spec.angular_damping)

    def bind(prim, item: Component, *, collide: bool | None = None, hidden=False) -> None:
        active = item.collide if collide is None else collide
        if active:
            UsdPhysics.CollisionAPI.Apply(prim)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.0015)
            collision.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), physics_path, stage=stage)
        if hidden:
            UsdGeom.Imageable(prim).CreatePurposeAttr("guide")
        else:
            bind_visual_material(str(prim.GetPath()), look(item), stage=stage)

    def mesh(
        path: str,
        points,
        counts,
        indices,
        item: Component,
        *,
        hidden=False,
        collide=None,
        double_sided=True,
    ):
        geom = UsdGeom.Mesh.Define(stage, path)
        geom.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        geom.CreateFaceVertexCountsAttr(counts)
        geom.CreateFaceVertexIndicesAttr(indices)
        geom.CreateSubdivisionSchemeAttr("none")
        geom.CreateDoubleSidedAttr(bool(double_sided))
        if hidden:
            UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
        bind(geom.GetPrim(), item, hidden=hidden, collide=collide)
        return geom

    def prism(item: Component, blade=False):
        cx, cy, cz = item.center
        length, width, height = item.size
        z0, z1 = cz - height / 2, cz + height / 2
        if blade:
            xy = [(-length / 2, -0.22 * width), (-length / 2, 0.22 * width),
                  (0.28 * length, width / 2), (length / 2, 0.38 * width),
                  (length / 2, -0.38 * width), (0.28 * length, -width / 2)]
        else:
            xy = wedge_profile_xy(item)
        points = [
            (px if not blade else cx + px, py if not blade else cy + py, z)
            for z in (z0, z1)
            for px, py in xy
        ]
        n = len(xy)
        counts = [n, n] + [4] * n
        indices = list(range(n)) + list(range(n, 2 * n))[::-1]
        for i in range(n):
            j = (i + 1) % n
            indices += [i, j, n + j, n + i]
        geom = mesh(f"{body_path}/{item.name}", points, counts, indices, item)
        UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")

    def cake_sector(item: Component):
        geometry = sector_prism_geometry(item, arc_segments=12)
        geom = mesh(
            f"{body_path}/{item.name}",
            geometry["points"],
            geometry["face_vertex_counts"],
            geometry["face_vertex_indices"],
            item,
        )
        UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")

    def dome(item: Component):
        cx, cy, cz = item.center
        radius, height = item.size[0] / 2, item.size[2]
        bottom = cz - height / 2
        profile = DOME_PROFILE
        collision_inner = [
            (z_value, max(0.02, radius_scale * radius - 0.003) / radius)
            for z_value, radius_scale in profile
        ]
        geometry = dome_shell_geometry(item, sides=24)
        mesh(
            f"{body_path}/{item.name}",
            geometry["points"],
            geometry["face_vertex_counts"],
            geometry["face_vertex_indices"],
            item,
            hidden=False,
            collide=False,
            double_sided=False,
        )
        # Invisible convex shell sectors provide real open-bottom collision.
        collision_sides = 16
        for band in range(len(profile) - 1):
            z0, ro0 = profile[band]
            z1, ro1 = profile[band + 1]
            _, ri0 = collision_inner[band]
            _, ri1 = collision_inner[band + 1]
            for index in range(collision_sides):
                a0 = 2 * math.pi * index / collision_sides
                a1 = 2 * math.pi * (index + 1) / collision_sides
                pts = []
                for zv, rv in ((z0, ro0), (z1, ro1), (z0, ri0), (z1, ri1)):
                    for angle in (a0, a1):
                        pts.append((cx + radius * rv * math.cos(angle), cy + radius * rv * math.sin(angle), bottom + height * zv))
                faces = [4, 4, 4, 4, 4, 4]
                face_indices = [0, 1, 3, 2, 4, 6, 7, 5, 0, 2, 6, 4, 1, 5, 7, 3, 0, 4, 5, 1, 2, 3, 7, 6]
                mesh(f"{body_path}/{item.name}_collision_{band}_{index}", pts, faces, face_indices, item, hidden=True)

    def ring(item: Component):
        cx, cy, cz = item.center
        outer = item.size[0] / 2
        fallback_inner = outer - min(0.008, outer * 0.14)
        inner_radius = min(outer - 0.001, item.size[1] / 2) if item.size[1] < item.size[0] else fallback_inner
        half_h = item.size[2] / 2
        sides = 24
        points = []
        for zvalue in (cz - half_h, cz + half_h):
            for radius in (outer, inner_radius):
                for index in range(sides):
                    angle = 2 * math.pi * index / sides
                    points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle), zvalue))
        counts, indices = [], []
        for surface in range(4):
            for index in range(sides):
                nxt = (index + 1) % sides
                if surface == 0: face = [index, nxt, 2*sides+nxt, 2*sides+index]
                elif surface == 1: face = [sides+index, 3*sides+index, 3*sides+nxt, sides+nxt]
                elif surface == 2: face = [2*sides+index, 2*sides+nxt, 3*sides+nxt, 3*sides+index]
                else: face = [index, sides+index, sides+nxt, nxt]
                counts.append(4); indices += face
        geom = mesh(f"{body_path}/{item.name}", points, counts, indices, item)
        UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexDecomposition")

    for item in spec.components:
        if item.kind == "dome_shell":
            dome(item)
        elif item.kind == "ring":
            ring(item)
        elif item.kind == "cake_sector":
            cake_sector(item)
        elif item.kind in {"wedge", "server_blade"}:
            prism(item, blade=item.kind == "server_blade")
        elif item.kind == "box":
            geom = UsdGeom.Cube.Define(stage, f"{body_path}/{item.name}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*item.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*item.size))
            bind(geom.GetPrim(), item)
        elif item.kind == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, f"{body_path}/{item.name}")
            geom.CreateAxisAttr(item.axis)
            long_axis = {"X": 0, "Y": 1, "Z": 2}[item.axis]
            radial = [i for i in range(3) if i != long_axis]
            geom.CreateHeightAttr(item.size[long_axis])
            geom.CreateRadiusAttr(min(item.size[i] for i in radial) / 2)
            geom.AddTranslateOp().Set(Gf.Vec3d(*item.center))
            bind(geom.GetPrim(), item)
        elif item.kind == "capsule":
            geom = UsdGeom.Capsule.Define(stage, f"{body_path}/{item.name}")
            geom.CreateAxisAttr(item.axis)
            long_axis = {"X": 0, "Y": 1, "Z": 2}[item.axis]
            radial = [i for i in range(3) if i != long_axis]
            radius = min(item.size[i] for i in radial) / 2
            geom.CreateRadiusAttr(radius)
            geom.CreateHeightAttr(max(item.size[long_axis] - 2 * radius, 1e-4))
            geom.AddTranslateOp().Set(Gf.Vec3d(*item.center))
            bind(geom.GetPrim(), item)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    spec_key = f"{id(spec):x}"
    _SPECS_BY_KEY[spec_key] = spec

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        spec_key: str = ""

    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=PartSpawnerCfg(spec_key=spec_key),
        initial_pose=Pose(
            position_xyz=tuple(spec.pos),
            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)),
        ),
    )


def _numpy(value) -> np.ndarray:
    value = getattr(value, "torch", value)
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(np.float64)
    return np.asarray(value, dtype=np.float64)


def part_state(environment, spec: Part) -> PartState:
    if not spec.free:
        pos = np.asarray(spec.pos, dtype=float)
        quat = np.array([0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)])
        velocity = np.zeros(6)
    else:
        base = environment.unwrapped
        asset = base.scene[spec.name]
        origin = _numpy(base.scene.env_origins)[0]
        pose = _numpy(asset.data.root_link_pose_w)[0]
        velocity = _numpy(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
        pos, quat = pose[:3] - origin, pose[3:7]
    probe = PartState(pos=pos, quat_xyzw=quat, lin_vel=velocity[:3], ang_vel=velocity[3:6],
                      low=np.zeros(3), high=np.zeros(3), sliders={})
    corners = np.array([[sx*spec.size[0]/2, sy*spec.size[1]/2, sz*spec.size[2]/2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    points = np.array([probe.to_world(point) for point in corners])
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=points.min(axis=0), high=points.max(axis=0), sliders={})


def restore_part_root(environment, spec: Part) -> None:
    """Restore one free task-owned root exactly after the official reset settles."""
    if not spec.free:
        return
    base = environment.unwrapped
    asset = base.scene[spec.name]
    pose = asset.data.default_root_pose.torch.clone()
    pose[:, :3] = base.scene.env_origins + pose[:, :3]
    velocity = asset.data.default_root_vel.torch.clone()
    velocity.zero_()
    asset.write_root_pose_to_sim_index(root_pose=pose)
    asset.write_root_velocity_to_sim_index(root_velocity=velocity)


def _patch_scene(scene) -> None:
    installed = set(getattr(scene, "_cake_dome_part_modules", ()))
    if MODULE_TOKEN in installed:
        return
    base_task_object, base_object_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, Part) else base_task_object(spec)

    def object_states(environment, names):
        specs = _ENV_SPECS.get(id(environment), {})
        plain = [name for name in names if name not in specs]
        out = base_object_states(environment, plain) if plain else {}
        out.update({name: part_state(environment, specs[name]) for name in names if name in specs})
        return {name: out[name] for name in names}

    scene.task_object, scene.object_states = task_object, object_states
    installed.add(MODULE_TOKEN)
    scene._cake_dome_part_modules = frozenset(installed)


class _SceneFinder(importlib.abc.MetaPathFinder):
    target = "yam_arena.scene"

    def find_spec(self, fullname, path, target=None):
        if fullname != self.target:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        original_exec = spec.loader.exec_module
        def execute(module):
            original_exec(module); _patch_scene(module)
        spec.loader.exec_module = execute
        return spec


def _resolved(path) -> str:
    return str(Path(path).expanduser().resolve())


def _register(kind: str, task_path, key: str):
    identity = (_resolved(task_path), str(key))
    def decorate(function):
        _CALLBACKS[kind][identity] = function
        install()
        return function
    return decorate


def on_reset(task_path, key: str): return _register("reset", task_path, key)
def on_step(task_path, key: str): return _register("step", task_path, key)


def callback_counts(task_path) -> dict[str, int]:
    path = _resolved(task_path)
    return {kind: sum(callback_path == path for callback_path, _ in callbacks) for kind, callbacks in _CALLBACKS.items()}


def _dispatch(kind: str, embodiment) -> None:
    task = getattr(embodiment, "task", None)
    if task is None:
        return
    owned = {spec.name: spec for spec in task.objects if isinstance(spec, Part)}
    if not owned:
        return
    environment = embodiment._env
    _ENV_SPECS[id(environment)] = owned
    path = _resolved(task.path)
    for (callback_path, _), function in tuple(_CALLBACKS[kind].items()):
        if callback_path == path:
            function(environment)


def _install_embodiment_hook() -> None:
    from yam_arena import inspect_embodiment
    cls = inspect_embodiment.YamArenaEmbodiment
    installed = set(getattr(cls, "_cake_dome_callback_modules", ()))
    if MODULE_TOKEN in installed:
        return
    base_reset, base_task_step = cls.reset, cls._task_step
    def reset(self, *args, **kwargs):
        observation = base_reset(self, *args, **kwargs); _dispatch("reset", self); return observation
    def task_step(self, *args, **kwargs):
        _dispatch("step", self); return base_task_step(self, *args, **kwargs)
    cls.reset, cls._task_step = reset, task_step
    installed.add(MODULE_TOKEN)
    cls._cake_dome_callback_modules = frozenset(installed)


def install() -> None:
    if _SceneFinder.target in sys.modules:
        _patch_scene(sys.modules[_SceneFinder.target])
    elif not any(isinstance(finder, _SceneFinder) for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneFinder())
    _install_embodiment_hook()


install()
