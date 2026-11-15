"""Full-size folding-rule collision geometry and orientation-independent compactness progress."""
import math
import numpy as np

N = 10
PITCH, LEAF_LENGTH, WIDTH, THICKNESS, LAYER = 0.200, 0.204, 0.016, 0.003, 0.0034
POCKET_INNER = np.array([0.224, 0.036, 0.050])
SHAPE_HALF = np.array([LEAF_LENGTH, WIDTH, THICKNESS]) / 2
_SIGNS = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def collision_corners(centers, rotations):
    centers, rotations = np.asarray(centers), np.asarray(rotations)
    if centers.shape != (N, 3) or rotations.shape != (N, 3, 3):
        raise ValueError("Need live transforms for all10 leaves")
    return centers[:, None, :] + np.einsum("nij,kj->nki", rotations, _SIGNS * SHAPE_HALF)


def compact_progress(corners):
    """Orientation-independent compactness progress, not a claim of pocket fit.

    Every pair of actual collision corners must fit within the target diagonal.
    A rigid partly extended rule cannot gain this tier by standing vertically.
    Full credit separately checks all corners against the actual pocket volume.
    """
    points = np.asarray(corners, dtype=float)
    if points.shape != (N, 8, 3) or not np.all(np.isfinite(points)):
        return False
    points = points.reshape(-1, 3)
    delta = points[:, None, :] - points[None, :, :]
    diameter_squared = float(np.max(np.einsum("ijk,ijk->ij", delta, delta)))
    diagonal = np.array([0.222, 0.034, 0.050])
    return diameter_squared <= float(diagonal @ diagonal) + 1e-12
