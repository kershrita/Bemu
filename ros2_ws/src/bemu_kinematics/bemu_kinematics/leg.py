"""Forward and inverse kinematics for one leg (see docs/07-locomotion.md, section 1).

Leg frame: origin on the hip swing axis, x out along the resting direction, y left, z up.
All lengths in metres, all angles in radians.
"""

import math
from typing import NamedTuple

import numpy as np
from numpy.typing import ArrayLike

from bemu_params import LegParams

REACH_TOLERANCE = 1e-12  # m
LIMIT_TOLERANCE = 1e-9  # rad


class JointAngles(NamedTuple):
    coxa: float  # rad
    femur: float  # rad
    tibia: float  # rad


class KinematicsError(ValueError):
    """Base class for kinematics failures."""


class UnreachableError(KinematicsError):
    def __init__(self, message: str, reason: str) -> None:
        super().__init__(message)
        self.reason = reason  # "too_far" | "too_close" | "behind_hip"


class JointLimitError(KinematicsError):
    def __init__(self, message: str, joint: str) -> None:
        super().__init__(message)
        self.joint = joint  # "coxa" | "femur" | "tibia"


def forward(leg: LegParams, angles: JointAngles) -> np.ndarray:
    """Foot position (shape (3,), m) in the leg frame for the given joint angles."""
    coxa, femur, tibia = angles
    reach = leg.femur_length * math.cos(femur) + leg.tibia_length * math.cos(femur + tibia)
    z = leg.femur_length * math.sin(femur) + leg.tibia_length * math.sin(femur + tibia)
    radial = leg.coxa_length + reach
    return np.array([radial * math.cos(coxa), radial * math.sin(coxa), z])


def _clamped_acos(value: float) -> float:
    return math.acos(min(1.0, max(-1.0, value)))


def inverse(leg: LegParams, foot: ArrayLike, *, check_limits: bool = True) -> JointAngles:
    """Joint angles (knee-up solution) that put the foot at `foot` (m, leg frame).

    Raises ValueError for a malformed target, UnreachableError if the target cannot be
    reached, and JointLimitError if an angle is outside its limits (unless check_limits
    is False).
    """
    try:
        target = np.asarray(foot, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"foot target must be three numbers, got {foot!r}") from exc
    if target.shape != (3,):
        raise ValueError(f"foot target must have shape (3,), got shape {target.shape}")
    if not np.all(np.isfinite(target)):
        raise ValueError(f"foot target must be finite, got {target.tolist()}")

    x, y, z = (float(v) for v in target)
    lf, lt = leg.femur_length, leg.tibia_length
    text = f"({x:.4f}, {y:.4f}, {z:.4f}) m"

    r = math.hypot(x, y) - leg.coxa_length
    if r <= 0.0:
        raise UnreachableError(
            f"foot target {text} is behind the hip lift axis (reach {r * 1000:.1f} mm)",
            "behind_hip",
        )
    d = math.hypot(r, z)
    if d > lf + lt + REACH_TOLERANCE:
        raise UnreachableError(
            f"foot target {text} is too far: {d * 1000:.1f} mm from the hip lift axis, "
            f"maximum {(lf + lt) * 1000:.1f} mm",
            "too_far",
        )
    if d < abs(lf - lt) - REACH_TOLERANCE:
        raise UnreachableError(
            f"foot target {text} is too close: {d * 1000:.1f} mm from the hip lift axis, "
            f"minimum {abs(lf - lt) * 1000:.1f} mm",
            "too_close",
        )

    alpha = math.atan2(z, r)
    beta = _clamped_acos((lf**2 + d**2 - lt**2) / (2.0 * lf * d))
    gamma = _clamped_acos((lf**2 + lt**2 - d**2) / (2.0 * lf * lt))
    angles = JointAngles(math.atan2(y, x), alpha + beta, gamma - math.pi)

    if check_limits:
        limits = (
            ("coxa", angles.coxa, leg.coxa_limits),
            ("femur", angles.femur, leg.femur_limits),
            ("tibia", angles.tibia, leg.tibia_limits),
        )
        for joint, angle, (lower, upper) in limits:
            if angle < lower - LIMIT_TOLERANCE or angle > upper + LIMIT_TOLERANCE:
                raise JointLimitError(
                    f"foot target {text} needs the {joint} at {math.degrees(angle):.1f} deg, "
                    f"outside its limits {math.degrees(lower):.1f} to "
                    f"{math.degrees(upper):.1f} deg",
                    joint,
                )
    return angles
