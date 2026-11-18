"""Body-pose kinematics: where the joints go so that planted feet stay put as the body moves.

Frames: the ground frame is fixed to the floor (z up). The body frame has its origin at the body
centre on the hip plane (x forward, y left, z up). Each leg frame is as in `leg.py`.
A pose is a body position and an orientation R = Rz(yaw) @ Ry(pitch) @ Rx(roll). A planted foot
is the foot sphere's centre, `leg.foot_radius` above the floor.

    P = Rᵀ (F − t)          planted foot F, seen from the body
    p = Rz(−ψ) (P − m)      the same point in the leg frame (mount m, resting direction ψ)
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from bemu_kinematics.leg import JointAngles, KinematicsError, forward, inverse
from bemu_params import LEG_CODES, RobotParams


@dataclass(frozen=True)
class BodyPose:
    height: float  # m, hip plane above the floor
    x: float = 0.0  # m, ground frame
    y: float = 0.0
    roll: float = 0.0  # rad
    pitch: float = 0.0
    yaw: float = 0.0


class BodyPoseError(KinematicsError):
    """A leg cannot reach its planted foot in the requested pose."""

    def __init__(self, leg: str, cause: KinematicsError) -> None:
        super().__init__(f"leg {leg}: {cause}")
        self.leg = leg
        self.cause = cause


def stand_height(params: RobotParams) -> float:
    """Hip-plane height (m) when standing: the stance height above the foot centres, plus them."""
    return params.stance.body_height + params.leg.foot_radius


def neutral_feet(params: RobotParams) -> dict[str, np.ndarray]:
    """Planted foot centres (ground frame) for the body at x = y = yaw = 0."""
    feet = {}
    for leg, mount in params.mounts.items():
        reach = params.stance.foot_radius
        feet[leg] = np.array(
            [
                mount.x + reach * math.cos(mount.yaw),
                mount.y + reach * math.sin(mount.yaw),
                params.leg.foot_radius,
            ]
        )
    return feet


def _rotation(pose: BodyPose) -> np.ndarray:
    cr, sr = math.cos(pose.roll), math.sin(pose.roll)
    cp, sp = math.cos(pose.pitch), math.sin(pose.pitch)
    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _check_pose(pose: BodyPose) -> None:
    values = (pose.height, pose.x, pose.y, pose.roll, pose.pitch, pose.yaw)
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"body pose must be finite, got {pose}")
    if abs(pose.roll) >= math.pi / 2 or abs(pose.pitch) >= math.pi / 2:
        raise ValueError(f"roll and pitch must be within ±90°, got {pose}")


def _check_feet(feet: Mapping[str, ArrayLike]) -> dict[str, np.ndarray]:
    missing = [leg for leg in LEG_CODES if leg not in feet]
    if missing:
        raise ValueError(f"feet is missing the legs: {', '.join(missing)}")
    extra = [leg for leg in feet if leg not in LEG_CODES]
    if extra:
        raise ValueError(f"feet has unknown legs: {', '.join(extra)}")
    checked = {}
    for leg in LEG_CODES:
        point = np.asarray(feet[leg], dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError(f"foot {leg} must be three finite numbers, got {feet[leg]!r}")
        checked[leg] = point
    return checked


def pose_angles(
    params: RobotParams, pose: BodyPose, feet: Mapping[str, ArrayLike] | None = None
) -> dict[str, JointAngles]:
    """Joint angles for every leg so the planted feet stay at `feet` (default: neutral feet)."""
    _check_pose(pose)
    planted = neutral_feet(params) if feet is None else _check_feet(feet)
    rotation = _rotation(pose)
    position = np.array([pose.x, pose.y, pose.height])
    angles = {}
    for leg in LEG_CODES:
        mount = params.mounts[leg]
        in_body = rotation.T @ (planted[leg] - position)
        in_leg = _rz(-mount.yaw) @ (in_body - np.array([mount.x, mount.y, 0.0]))
        try:
            angles[leg] = inverse(params.leg, in_leg)
        except KinematicsError as cause:
            raise BodyPoseError(leg, cause) from cause
    return angles


def foot_ground_position(
    params: RobotParams, leg_code: str, pose: BodyPose, angles: JointAngles
) -> np.ndarray:
    """Where a leg's foot centre is, in the ground frame, for a body pose and joint angles."""
    mount = params.mounts[leg_code]
    in_leg = forward(params.leg, angles)
    in_body = np.array([mount.x, mount.y, 0.0]) + _rz(mount.yaw) @ in_leg
    return np.array([pose.x, pose.y, pose.height]) + _rotation(pose) @ in_body
