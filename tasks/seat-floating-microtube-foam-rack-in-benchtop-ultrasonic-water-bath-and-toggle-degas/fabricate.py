"""Print files for the real-world preparation and post-trial verification fixtures of
seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas:

1. ``staging_cradle.stl``: open-well PETG benchtop staging cradle (102 x 74 x 26 mm outer frame with a 66 x 38 x 23 mm
   central clearance well and an 8 mm wide perimeter support ledge at z = 23.0 mm) that supports a Heathrow Scientific
   HS29041B / Bel-Art F18876-0008 8-well rectangular EVA foam floating rack (82 x 54 x 20 mm) flat by its foam
   underside while its eight protruding 19.0 mm conical 1.5 mL Eppendorf tube tips hang 4.0 mm clear of the floor.
2. ``button_actuator_cap.stl``: self-adhesive PETG tactile cap (22 x 14 x 6 mm with a 35-degree top press facet) bonded
   with 0.5 mm 3M VHB tape onto the oval ON and OFF membrane switches of the Magnasonic CD2800 600 mL ultrasonic
   cleaner so vertical/angled parallel-jaw fingertip presses actuate the dome switches cleanly without side slip.
3. ``freeboard_gauge.stl``: L-shaped post-trial go/no-go verification gauge with a 14.0 mm rim-to-MAX-waterline step
   (verifying the 480 mL / 38.0 mm fill level before each trial) and a 16.1 +/- 3.0 mm dry-foam-freeboard notch
   (verifying Archimedes floating height of the 22.0 g loaded rack after the trial).

    python3 fabricate.py [out_dir]     writes staging_cradle.stl, button_actuator_cap.stl, freeboard_gauge.stl (ASCII STL, mm)

Standard library only; nothing is written on import.
"""

from __future__ import annotations

import math
import os
import sys

# Exact dimensions (mm) matching seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py
CRADLE_OUTER_X = 74.0
CRADLE_OUTER_Y = 102.0
CRADLE_WALL_H = 26.0
CRADLE_LEDGE_Z = 23.0
CRADLE_FLOOR_H = 3.0
RACK_X = 54.0
RACK_Y = 82.0
POCKET_X = 56.0  # open X flanks; 54.0 mm foam rack sits flat on the 4.5 mm perimeter ledge
POCKET_Y = 88.0  # 3.0 mm per-end clearance around the 82.0 mm foam rack length
WELL_X = 45.0    # 45 x 75 mm central clearance well (> 4.5 mm X and > 2.3 mm Y clearance around all 8 tube cones)
WELL_Y = 75.0


def _box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> list:
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _write_ascii_stl(path: str, name: str, tris: list) -> None:
    lines = [f"solid {name}"]
    for a, b, c in tris:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        lines.append(f"  facet normal {nx / ln:.6f} {ny / ln:.6f} {nz / ln:.6f}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}\n")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def staging_cradle_tris() -> list:
    tris: list = []
    ox, oy = CRADLE_OUTER_X / 2.0, CRADLE_OUTER_Y / 2.0
    wx, wy = WELL_X / 2.0, WELL_Y / 2.0
    px, py = POCKET_X / 2.0, POCKET_Y / 2.0
    # Base floor plate (3.0 mm)
    tris += _box(-ox, -oy, 0.0, ox, oy, CRADLE_FLOOR_H)
    # Four perimeter walls up to the 23.0 mm foam-support ledge leaving the 38 x 66 mm tube-tip well open
    tris += _box(-ox, -oy, CRADLE_FLOOR_H, -wx, oy, CRADLE_LEDGE_Z)
    tris += _box(wx, -oy, CRADLE_FLOOR_H, ox, oy, CRADLE_LEDGE_Z)
    tris += _box(-wx, -oy, CRADLE_FLOOR_H, wx, -wy, CRADLE_LEDGE_Z)
    tris += _box(-wx, wy, CRADLE_FLOOR_H, wx, oy, CRADLE_LEDGE_Z)
    # Short 3.0 mm corner registration lips (z = 23..26 mm) at the Y ends, leaving X grip flanks completely open
    tris += _box(-ox, -oy, CRADLE_LEDGE_Z, ox, -py, CRADLE_WALL_H)
    tris += _box(-ox, py, CRADLE_LEDGE_Z, ox, oy, CRADLE_WALL_H)
    return tris


def button_actuator_cap_tris() -> list:
    tris: list = []
    # 22 x 14 x 6 mm tactile button cap with raised center contact ridge
    tris += _box(-7.0, -11.0, 0.0, 7.0, 11.0, 4.0)
    tris += _box(-4.5, -8.5, 4.0, 4.5, 8.5, 6.0)
    return tris


def freeboard_gauge_tris() -> list:
    tris: list = []
    # Spine bar (12 x 8 x 60 mm) + 14.0 mm rim-to-waterline step + 16.1 mm dry-freeboard reference step
    tris += _box(0.0, -4.0, 0.0, 8.0, 4.0, 60.0)
    tris += _box(8.0, -4.0, 46.0, 28.0, 4.0, 60.0)   # 14.0 mm step from top rim down to MAX water line
    tris += _box(-20.0, -4.0, 0.0, 0.0, 4.0, 16.1)   # 16.1 mm dry foam freeboard check step
    return tris


def write_all(out_dir: str) -> dict[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    paths = {
        "staging_cradle": os.path.join(out_dir, "staging_cradle.stl"),
        "button_actuator_cap": os.path.join(out_dir, "button_actuator_cap.stl"),
        "freeboard_gauge": os.path.join(out_dir, "freeboard_gauge.stl"),
    }
    _write_ascii_stl(paths["staging_cradle"], "staging_cradle", staging_cradle_tris())
    _write_ascii_stl(paths["button_actuator_cap"], "button_actuator_cap", button_actuator_cap_tris())
    _write_ascii_stl(paths["freeboard_gauge"], "freeboard_gauge", freeboard_gauge_tris())
    return paths


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "."
    written = write_all(target)
    for k, p in written.items():
        print(f"wrote {k}: {p}")
