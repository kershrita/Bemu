"""Foot paths for one leg (docs/07-locomotion.md, section 4), as offsets from the neutral point."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike

from bemu_kinematics import ramp


def stride_vector(
    vx: float, vy: float, omega: float, neutral_xy: ArrayLike, stance_time: float
) -> np.ndarray:
    """How far the foot moves, in the body frame, during one stance: (v + ω × r) · stance_time."""
    rx, ry = (float(c) for c in np.asarray(neutral_xy, dtype=float)[:2])
    return np.array([vx - omega * ry, vy + omega * rx]) * stance_time


def foot_offset(phase: float, duty: float, stride: ArrayLike, lift: float) -> np.ndarray:
    """Offset (3,) from the neutral point at a leg phase in [0, 1).

    Stance: a straight line from +stride/2 to -stride/2 on the ground. Swing: back to +stride/2
    along a smooth blend, lifted by lift·sin(π·u). Continuous in phase, including the wrap.
    """
    s = np.asarray(stride, dtype=float)[:2]
    if phase < duty:
        u = phase / duty
        return np.array([*(s * (0.5 - u)), 0.0])
    u = (phase - duty) / (1.0 - duty)
    return np.array([*(s * (-0.5 + ramp(u))), lift * math.sin(math.pi * u)])
