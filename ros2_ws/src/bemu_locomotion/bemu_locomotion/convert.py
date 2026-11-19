"""Message-to-core conversions that need no ROS."""

from __future__ import annotations

import math
from collections.abc import Sequence

from bemu_kinematics import BodyPose


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Roll, pitch and yaw (rad) of the rotation R = Rz(yaw)·Ry(pitch)·Rx(roll), the convention
    of BodyPose. The quaternion need not be normalised."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError(f"not a usable quaternion: {(x, y, z, w)}")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def pose_from_msg_values(position: Sequence[float], quaternion: Sequence[float]) -> BodyPose:
    """A BodyPose from a pose's position (x, y, z = hip height) and quaternion (x, y, z, w)."""
    x, y, height = position
    roll, pitch, yaw = quaternion_to_rpy(*quaternion)
    return BodyPose(height=height, x=x, y=y, roll=roll, pitch=pitch, yaw=yaw)
