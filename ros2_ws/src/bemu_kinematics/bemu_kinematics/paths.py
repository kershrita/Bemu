"""Closed test paths for the foot (leg frame, metres).

Points are evenly spaced along the perimeter and the first point is not repeated at the end.
"""

import numpy as np
from numpy.typing import ArrayLike

_PLANES = {"xz": (0, 2), "xy": (0, 1)}


def _axes(plane: str) -> tuple[int, int]:
    try:
        return _PLANES[plane]
    except KeyError:
        raise ValueError(f"plane must be 'xz' or 'xy', got {plane!r}") from None


def _embed(center: ArrayLike, plane: str, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ia, ib = _axes(plane)
    points = np.tile(np.asarray(center, dtype=float), (len(a), 1))
    points[:, ia] += a
    points[:, ib] += b
    return points


def _check(center: ArrayLike, size: float, n: int, minimum: int, name: str) -> None:
    if n < minimum:
        raise ValueError(f"n must be at least {minimum}, got {n}")
    if not (isinstance(size, int | float) and np.isfinite(size) and size > 0):
        raise ValueError(f"{name} must be finite and positive, got {size!r}")
    try:
        c = np.asarray(center, dtype=float)
    except (TypeError, ValueError):
        raise ValueError(f"center must be three numbers, got {center!r}") from None
    if c.shape != (3,) or not np.all(np.isfinite(c)):
        raise ValueError(f"center must be three finite numbers, got {center!r}")


def square_path(center: ArrayLike, side: float, n: int, *, plane: str = "xz") -> np.ndarray:
    """(n, 3) points around a square of the given side centred on `center`, counter-clockwise."""
    _axes(plane)
    _check(center, side, n, 4, "side")
    half = side / 2.0
    s = np.arange(n) * (4.0 * side / n)  # distance travelled along the perimeter
    edge = np.minimum((s // side).astype(int), 3)
    t = s - edge * side
    a = np.choose(edge, [-half + t, np.full(n, half), half - t, np.full(n, -half)])
    b = np.choose(edge, [np.full(n, -half), -half + t, np.full(n, half), half - t])
    return _embed(center, plane, a, b)


def circle_path(center: ArrayLike, radius: float, n: int, *, plane: str = "xy") -> np.ndarray:
    """(n, 3) points around a circle of the given radius centred on `center`."""
    _axes(plane)
    _check(center, radius, n, 3, "radius")
    theta = np.arange(n) * (2.0 * np.pi / n)
    return _embed(center, plane, radius * np.cos(theta), radius * np.sin(theta))
