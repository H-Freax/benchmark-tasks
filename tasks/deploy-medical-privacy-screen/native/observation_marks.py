"""Visible doorway viewing marks; fixed context only, no colliders or supports.

A square amber outline is centred exactly on each existing scored sight point.
The 60 mm aperture admits the physical judging camera's optical centre. Marks
represent tape/paint on an independent camera-positioning jig, not screen parts.
"""
import copy


def with_observation_marks(geometry):
    """Return an independent geometry with three registered context outlines."""
    out = copy.deepcopy(geometry)
    colour = [1., .65, .05]
    # Box sizes are half-extents, matching native_geometry's shape convention.
    bars = [([0, -.034, 0], [.002, .004, .038]),
            ([0,  .034, 0], [.002, .004, .038]),
            ([0, 0, -.034], [.002, .030, .004]),
            ([0, 0,  .034], [.002, .030, .004])]
    for index, eye in enumerate(out['doorway_sight_points_m'], 1):
        name = f'doorway_view_{index}'
        out['parts'].append(dict(
            name=name, pos=list(eye), quat_xyzw=[0.,0.,0.,1.], fixed=True,
            mass_kg=.001, context_only=True,
            geoms=[dict(name=f'view_outline_{i}', type='box', pos=pos,
                        quat_xyzw=[0.,0.,0.,1.], size=size, color=colour,
                        collision_enabled=False)
                   for i,(pos,size) in enumerate(bars)]))
    return out
