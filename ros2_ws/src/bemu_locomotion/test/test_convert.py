import math

import numpy as np
import pytest

from bemu_kinematics import BodyPose
from bemu_locomotion.convert import pose_from_msg_values, quaternion_to_rpy


def quaternion(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll) as a quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


@pytest.mark.parametrize(
    "rpy", [(0, 0, 0), (math.radians(10), 0, 0), (0, math.radians(-8), 0), (0.1, -0.2, 0.7)]
)
def test_round_trip(rpy):
    assert quaternion_to_rpy(*quaternion(*rpy)) == pytest.approx(rpy, abs=1e-9)


def test_a_non_unit_quaternion_is_normalised():
    x, y, z, w = quaternion(0.2, 0.0, 0.0)
    assert quaternion_to_rpy(3 * x, 3 * y, 3 * z, 3 * w) == pytest.approx((0.2, 0, 0), abs=1e-9)


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (math.nan, 0, 0, 1), (0, 0, math.inf, 1)])
def test_a_degenerate_quaternion_is_rejected(q):
    with pytest.raises(ValueError):
        quaternion_to_rpy(*q)


def test_pose_from_message_values():
    pose = pose_from_msg_values((0.01, -0.02, 0.09), quaternion(0.1, 0.05, 0.0))
    assert isinstance(pose, BodyPose)
    assert np.allclose(
        [pose.x, pose.y, pose.height, pose.roll, pose.pitch, pose.yaw],
        [0.01, -0.02, 0.09, 0.1, 0.05, 0.0],
    )
