"""CPU distances between complete, finite, world-space convex vertex sets.

Lower bounds certify clearance only. Coal queries the solid convex hulls (not
triangle soups); its witness direction supplies an independent dual lower
bound. Numerically unresolved threshold crossings remain indeterminate.
"""
import itertools
import math
from fractions import Fraction

import numpy as np
from scipy.spatial import ConvexHull


def _result(status, lower=None, upper=None, distance=None, method="validation", detail=""):
    return dict(status=status, lower_m=lower, upper_m=upper,
                distance_m=distance, method=method, detail=detail)


def _axis_box(v):
    lo, hi = v.min(0), v.max(0)
    if np.any(hi <= lo):
        return False
    # Exact corner presence proves conv(vertices) is precisely its AABB.
    points = set(map(tuple, v))
    return all(c in points for c in itertools.product(*zip(lo, hi)))


def _box_distance(a, b, threshold):
    # Rational arithmetic avoids a tolerance-based pass at the inclusive edge.
    gaps = []
    for amin, amax, bmin, bmax in zip(a.min(0), a.max(0), b.min(0), b.max(0)):
        gaps.append(max(Fraction(0), Fraction(float(amin))-Fraction(float(bmax)),
                        Fraction(float(bmin))-Fraction(float(amax))))
    square = sum(x*x for x in gaps)
    limit = Fraction(threshold)**2
    distance = math.hypot(*(float(x) for x in gaps))
    if not math.isfinite(distance):
        raise ValueError("Analytical distance overflow")
    if square == limit:
        lower = upper = threshold
    elif square == 0:
        lower = upper = 0.0
    else:
        # Certify the rounded square-root bracket with the same exact values.
        lower = upper = distance
        while Fraction(lower)**2 > square:
            lower = float(np.nextafter(lower, -np.inf))
        while Fraction(upper)**2 < square:
            upper = float(np.nextafter(upper, np.inf))
    return _result("clear" if square >= limit else "violation", lower, upper,
                   distance, "analytic_axis_boxes_exact_rational_comparison",
                   "Full hulls equal their AABBs; squared distance compared exactly to the binary-float threshold.")


def _inner_witness(p, vertices, hull, guard):
    """Move a numerical witness inside all full-hull supporting planes."""
    center = vertices.mean(0)
    normals, offsets = hull.equations[:, :3], hull.equations[:, 3]
    cv = normals @ center + offsets
    pv = normals @ p + offsets
    if np.any(cv >= -guard):
        raise ValueError("Hull too thin for guarded interior witness")
    denom = pv-cv
    needed = np.divide(pv+4*guard, denom, out=np.zeros_like(pv), where=denom > 0)
    t = min(1.0, max(0.0, float(needed.max())))
    q = (1-t)*p+t*center
    if np.any(normals @ q + offsets > guard):
        raise ValueError("Could not validate convex witness membership")
    return q


def distance_bounds(a, b, threshold=.05):
    """Return clear, violation, or indeterminate with a guarded distance bracket.

    Inputs are Nx3/Mx3 world vertices. No simplification, pose cache, simulator
    import, or surface-mesh approximation is used. Unsupported/degenerate hulls,
    solver errors, nonfinite data and unresolved boundaries are indeterminate.
    """
    try:
        threshold = float(threshold)
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        if not math.isfinite(threshold) or threshold < 0:
            raise ValueError("Threshold must be finite and nonnegative")
        for v in (a, b):
            if v.ndim != 2 or v.shape[1] != 3 or len(v) < 4 or not np.isfinite(v).all():
                raise ValueError("Expected at least four finite three-dimensional vertices")
        # Preserve the original binary coordinates for the exact comparison;
        # recentering can otherwise round an exact boundary to either side.
        if _axis_box(a) and _axis_box(b):
            return _box_distance(a, b, threshold)
        # Recenter, without dropping any vertex, for stable support projections.
        origin = a[0].copy()
        a, b = a-origin, b-origin
        if not np.isfinite(a).all() or not np.isfinite(b).all():
            raise ValueError("Coordinate subtraction overflow")
        ha, hb = ConvexHull(a), ConvexHull(b)
        scale = max(1.0, float(np.abs(a).max()), float(np.abs(b).max()))
        guard = max(1e-9, 128*np.finfo(float).eps*scale)
        separation = np.maximum(np.maximum(a.min(0)-b.max(0), b.min(0)-a.max(0)), 0)
        lower = max(0.0, float(np.linalg.norm(separation))-guard)
        axes = np.vstack((ha.equations[:, :3], hb.equations[:, :3]))
        axes /= np.linalg.norm(axes, axis=1)[:, None]
        av, bv = a @ axes.T, b @ axes.T
        face = float(np.maximum(av.min(0)-bv.max(0), bv.min(0)-av.max(0)).max())
        lower = max(lower, face-guard, 0.0)
        # A vertex pair is always a valid primal upper bound.
        upper = float(np.linalg.norm(a[0]-b[0]))+guard
        if lower >= threshold:
            return _result("clear", lower, upper, None, "aabb_face_support_lower_bound",
                           "Clearance proved by a separating support bound; exact distance not queried.")
        import coal
        def solid(v, hull):
            # Reuse the complete SciPy hull; do not enter Coal's Qhull builder.
            points, faces = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
            for p in v:
                points.append(p)
            for face, plane in zip(hull.simplices, hull.equations, strict=True):
                i, j, k = map(int, face)
                if np.dot(np.cross(v[j]-v[i], v[k]-v[i]), plane[:3]) < 0:
                    j, k = k, j
                faces.append(coal.Triangle(i, j, k))
            return coal.Convex(points, faces)
        ca, cb = solid(a, ha), solid(b, hb)
        request, result = coal.DistanceRequest(), coal.DistanceResult()
        request.gjk_tolerance = 1e-12
        request.epa_tolerance = 1e-12
        request.gjk_max_iterations = 512
        request.epa_max_iterations = 512
        request.abs_err = request.rel_err = 0.0
        raw = float(coal.distance(ca, coal.Transform3s(), cb, coal.Transform3s(), request, result))
        p, q = np.asarray(result.getNearestPoint1()), np.asarray(result.getNearestPoint2())
        if not math.isfinite(raw) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError("Coal returned nonfinite distance or witnesses")
        if raw < 0:
            # Signed penetration witnesses need not coincide. Find a common
            # solid point, then validate/repair it against both complete hulls.
            from scipy.optimize import linprog
            planes = np.vstack((ha.equations, hb.equations))
            fit = linprog(np.zeros(3), A_ub=planes[:, :3], b_ub=-planes[:, 3],
                          bounds=[(None, None)]*3, method="highs")
            if not fit.success or not np.isfinite(fit.x).all():
                raise ValueError("Could not certify Coal overlap")
            p = q = fit.x
        else:
            witness = q-p
            norm = float(np.linalg.norm(witness))
            if abs(norm-raw) > max(32*guard, 1e-7*max(norm, abs(raw))):
                raise ValueError("Coal distance and nearest witnesses disagree")
            if norm > guard:
                direction = witness/norm
                gap = float((b @ direction).min()-(a @ direction).max())
                lower = max(lower, gap-guard, 0.0)
        p = _inner_witness(p, a, ha, guard)
        q = _inner_witness(q, b, hb, guard)
        upper = min(upper, float(np.linalg.norm(q-p))+guard)
        if lower > upper:
            raise ValueError("Primal and dual distance certificates disagree")
        status = "clear" if lower >= threshold else "violation" if upper < threshold else "indeterminate"
        return _result(status, lower, upper, max(0.0, raw), "coal_convex_primal_dual",
                       "Full convex solid query; support-axis lower and interior witness upper; guard="+repr(guard)+
                       ("; threshold lies inside bracket" if status == "indeterminate" else ""))
    except Exception as error:
        return _result("indeterminate", detail=type(error).__name__+": "+str(error))
