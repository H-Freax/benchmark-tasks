"""End-state checks from the two items' poses and velocities and the gripper points.

Independent of the film counters and of any scripted phase.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from vertex_helper import vertices

MAX_SPEED = 0.02          # m/s
MAX_SPIN = 0.1            # rad/s
HAND_CLEARANCE = 0.03     # m, gripper points to each item's bounding box
MAX_BOTTLE_TILT = math.radians(10.0)


def final_state(G, bodies, hand):
    """withdrawn: every point of the brush head is above the bottle mouth, or the head is beside the bottle.
    at_rest: both items are on or above the tabletop and slower than 20 mm/s and 0.1 rad/s.
    standing: the bottle axis is within 10 deg of vertical.
    hands_clear: every gripper point is at least 30 mm from each item's bounding box."""
    out = dict(withdrawn=False, at_rest=False, standing=False, hands_clear=False)
    points = np.asarray(hand, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        return out
    parts = {p["name"]: p for p in G["parts"]}
    at_rest = hands_clear = True
    for name in ("feeding_bottle", "bottle_brush"):
        row = bodies.get(name, {})
        pose = np.asarray(row.get("pose"), float)
        vel = np.asarray(row.get("velocity"), float)
        if pose.shape != (7,) or vel.shape != (6,) or not np.isfinite(np.r_[pose, vel]).all():
            return out
        v = vertices(parts[name]) @ Rotation.from_quat(pose[3:]).as_matrix().T + pose[:3]
        slow = np.linalg.norm(vel[:3]) <= MAX_SPEED and np.linalg.norm(vel[3:]) <= MAX_SPIN
        if v[:, 2].min() < G["table_top_m"] - 0.002 or not slow:
            at_rest = False
        d = np.linalg.norm(np.maximum(np.maximum(v.min(0) - points, points - v.max(0)), 0), axis=1)
        if d.min() < HAND_CLEARANCE:
            hands_clear = False
    b = np.asarray(bodies["feeding_bottle"]["pose"], float)
    h = np.asarray(bodies["bottle_brush"]["pose"], float)
    Rb = Rotation.from_quat(b[3:]).as_matrix()
    head = vertices(dict(geoms=[parts["bottle_brush"]["geoms"][0]])) @ Rotation.from_quat(h[3:]).as_matrix().T + h[:3]
    local = (head - b[:3]) @ Rb
    outer_radius = parts["feeding_bottle"]["geoms"][0]["size"][0]
    beside = np.linalg.norm(local[:, :2], axis=1).min() > outer_radius + 0.005
    out["withdrawn"] = bool(local[:, 2].min() > G["bottle"]["top_m"] + 0.005 or beside)
    out["at_rest"] = bool(at_rest)
    out["standing"] = bool(math.acos(min(1.0, max(-1.0, Rb[2, 2]))) <= MAX_BOTTLE_TILT)
    out["hands_clear"] = bool(hands_clear)
    return out
