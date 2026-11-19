"""Collision-shape vertices of a part in its own frame."""
import numpy as np
from scipy.spatial.transform import Rotation


def vertices(part):
    out = []
    for s in part["geoms"]:
        if s["type"] == "convex":
            v = np.array(s["vertices"])
        elif s["type"] == "box":
            h = np.array(s["size"])
            v = np.array([[x, y, z] for x in [-h[0], h[0]] for y in [-h[1], h[1]] for z in [-h[2], h[2]]])
        elif s["type"] == "cylinder":
            r, h = s["size"][:2]
            v = np.array([[r * np.cos(a), r * np.sin(a), z] for z in [-h, h] for a in np.arange(32) * np.pi / 16])
        else:
            raise ValueError(s["type"])
        out.append(v @ Rotation.from_quat(s["quat_xyzw"]).as_matrix().T + s["pos"])
    return np.vstack(out)
