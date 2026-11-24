"""Generate printable STL fixtures and the 1.5 mm no-go feeler gauge for rearrange-wall-wart-to-end-of-crowded-power-strip:
1. strip_cleat_frame.stl   — Low-profile 4.5 mm desk-mat alignment frame keeping the 214 x 50 mm CyberPower CSB404 on mark
2. plug_cord_cradle.stl    — Weighted desktop cord cradle staging the black NEMA 5-15P plug prongs-down 2.5 mm above the mat
3. feeler_gauge_1p5mm.stl  — 1.5 mm no-go blade-exposure feeler gauge for flush-seating verification
"""

from __future__ import annotations

import os
import struct
import sys


def _box_tris(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> list[tuple]:
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = cz - sz / 2.0, cz + sz / 2.0
    v = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    quads = [
        (0, 3, 2, 1, (0.0, 0.0, -1.0)),
        (4, 5, 6, 7, (0.0, 0.0, 1.0)),
        (0, 1, 5, 4, (0.0, -1.0, 0.0)),
        (2, 3, 7, 6, (0.0, 1.0, 0.0)),
        (0, 4, 7, 3, (-1.0, 0.0, 0.0)),
        (1, 2, 6, 5, (1.0, 0.0, 0.0)),
    ]
    tris = []
    for a, b, c, d, n in quads:
        tris.append((n, v[a], v[b], v[c]))
        tris.append((n, v[a], v[c], v[d]))
    return tris


def _write_binary_stl(path: str, tris: list[tuple], header: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    hdr = header.encode("ascii", errors="ignore")[:80].ljust(80, b"\x00")
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(struct.pack("<I", len(tris)))
        for n, p0, p1, p2 in tris:
            f.write(struct.pack("<12fH", *n, *p0, *p1, *p2, 0))
    return path


def write_all(out_dir: str) -> dict[str, str]:
    # Units in mm for standard slicer import
    cleat_tris = (
        _box_tris(0.0, 21.0, 1.0, 66.0, 232.0, 2.0)
        + _box_tris(-29.5, -62.0, 4.25, 5.0, 36.0, 4.5)
        + _box_tris(+29.5, -62.0, 4.25, 5.0, 36.0, 4.5)
        + _box_tris(-29.5, +96.0, 4.25, 5.0, 36.0, 4.5)
        + _box_tris(+29.5, +96.0, 4.25, 5.0, 36.0, 4.5)
        + _box_tris(0.0, -90.5, 4.25, 50.0, 5.0, 4.5)
        + _box_tris(0.0, +132.5, 4.25, 50.0, 5.0, 4.5)
    )
    cradle_tris = (
        _box_tris(0.0, 0.0, 0.75, 48.0, 46.0, 1.5)
        + _box_tris(-15.5, 0.0, 9.25, 5.0, 28.0, 18.5)
        + _box_tris(+15.5, 0.0, 9.25, 5.0, 28.0, 18.5)
    )
    feeler_tris = (
        _box_tris(0.0, 0.0, 2.5, 80.0, 16.0, 5.0)       # Handle body
        + _box_tris(+57.5, 0.0, 0.75, 35.0, 12.0, 1.5)  # 1.5 mm no-go zero-exposed-blade feeler tip
    )
    return {
        "strip_cleat_frame": _write_binary_stl(
            os.path.join(out_dir, "strip_cleat_frame.stl"), cleat_tris, "strip_cleat_frame_mm"
        ),
        "plug_cord_cradle": _write_binary_stl(
            os.path.join(out_dir, "plug_cord_cradle.stl"), cradle_tris, "plug_cord_cradle_mm"
        ),
        "feeler_gauge_1p5mm": _write_binary_stl(
            os.path.join(out_dir, "feeler_gauge_1p5mm.stl"), feeler_tris, "feeler_gauge_1p5mm"
        ),
    }


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "fixtures")
    written = write_all(target)
    for k, v in written.items():
        print(f"Wrote {k}: {v}")
