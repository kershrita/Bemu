"""Static stability margin: how far the centre of mass is inside the support polygon."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike


def _hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convex hull, counter-clockwise, without collinear points (Andrew's monotone chain)."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-12:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-12:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def stability_margin(com: ArrayLike, feet: ArrayLike) -> float:
    """Shortest distance (m) from the centre of mass to the edge of the feet's convex hull.

    Positive inside, 0 on an edge, negative outside (minus the distance to the polygon).
    `com` is (2,) or (3,), `feet` is (n, 2) or (n, 3); only x and y are used. With fewer than
    three distinct feet, or feet all on one line, there is no support polygon: -inf.
    """
    point = np.asarray(com, dtype=float).reshape(-1)[:2]
    array = np.asarray(feet, dtype=float)
    array = array.reshape(-1, array.shape[-1] if array.ndim > 1 else 2)[:, :2]
    hull = _hull([(float(x), float(y)) for x, y in array])
    if len(hull) < 3:
        return -math.inf

    px, py = float(point[0]), float(point[1])
    inside_margin = math.inf
    outside_distance = math.inf
    inside = True
    for (ax, ay), (bx, by) in zip(hull, hull[1:] + hull[:1], strict=True):
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        signed = (ex * (py - ay) - ey * (px - ax)) / length  # > 0 on the inner side (CCW hull)
        inside_margin = min(inside_margin, signed)
        if signed < 0:
            inside = False
        t = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / (length * length)))
        outside_distance = min(outside_distance, math.hypot(px - ax - t * ex, py - ay - t * ey))
    return inside_margin if inside else -outside_distance
