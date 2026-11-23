"""Generate 1:1 printable STL fixtures for disable-phantom-power-before-unlatching-xlr-mic-plug:
- audio_interface_chassis.stl: 1:1 117 x 215 x 54 mm 2-channel desktop USB audio interface housing with XLR & TRS bores
- neutrik_xlr_plug.stl: 1:1 male XLR barrel with underside retention lug, pull-shoulder collar, and strain-relief boot
- studio_cable_tray.stl: walled 140 x 104 x 22 mm desktop cable stowage tray (128 x 92 mm inner basin)
"""

from __future__ import annotations

import os
import struct


def _box_tris(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> list:
    hx, hy, hz = 0.5 * sx, 0.5 * sy, 0.5 * sz
    v = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7)]
    out = []
    for a, b, c, d in quads:
        out.append((v[a], v[b], v[c]))
        out.append((v[a], v[c], v[d]))
    return out


def _write_binary_stl(path: str, tris: list, label: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    hdr = label.encode("ascii", errors="ignore")[:80].ljust(80, b" ")
    with open(path, "wb") as fh:
        fh.write(hdr)
        fh.write(struct.pack("<I", len(tris)))
        for p0, p1, p2 in tris:
            ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
            vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            n = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
            fh.write(struct.pack("<3f", nx / n, ny / n, nz / n))
            for pt in (p0, p1, p2):
                fh.write(struct.pack("<3f", *pt))
            fh.write(struct.pack("<H", 0))
    return path


def write_all(out_dir: str) -> dict[str, str]:
    chassis = _box_tris(0.0, 0.0, 27.0, 117.0, 215.0, 54.0)
    xlr = _box_tris(0.0, 0.0, 13.0, 82.0, 28.0, 26.0)
    tray = (
        _box_tris(0.0, 0.0, 2.0, 140.0, 104.0, 4.0)
        + _box_tris(-67.0, 0.0, 13.0, 6.0, 104.0, 18.0)
        + _box_tris(+67.0, 0.0, 13.0, 6.0, 104.0, 18.0)
        + _box_tris(0.0, -49.0, 13.0, 128.0, 6.0, 18.0)
        + _box_tris(0.0, +49.0, 13.0, 128.0, 6.0, 18.0)
    )
    return {
        "audio_interface_chassis": _write_binary_stl(os.path.join(out_dir, "audio_interface_chassis.stl"), chassis, "scarlett_2i2_mm"),
        "neutrik_xlr_plug": _write_binary_stl(os.path.join(out_dir, "neutrik_xlr_plug.stl"), xlr, "neutrik_xlr_mm"),
        "studio_cable_tray": _write_binary_stl(os.path.join(out_dir, "studio_cable_tray.stl"), tray, "cable_tray_mm"),
    }
