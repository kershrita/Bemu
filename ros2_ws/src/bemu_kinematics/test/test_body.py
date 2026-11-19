import math

import numpy as np
import pytest

from bemu_kinematics import (
    BodyPose,
    BodyPoseError,
    KinematicsError,
    foot_ground_position,
    neutral_feet,
    pose_angles,
    stand_height,
)
from bemu_params import LEG_CODES

NEUTRAL_ANGLES = (0.0, 0.300494, -1.753631)
DEG = math.pi / 180.0


def test_neutral_pose_gives_every_leg_the_version_0_neutral_angles(params):
    angles = pose_angles(params, BodyPose(stand_height(params)))
    assert tuple(angles) == LEG_CODES
    for leg in LEG_CODES:
        assert angles[leg] == pytest.approx(NEUTRAL_ANGLES, abs=1e-6), leg


def test_stand_height_is_the_stance_height_plus_the_foot_radius(params):
    assert stand_height(params) == pytest.approx(0.098)


def test_neutral_feet_are_the_mount_plus_the_reach_on_the_floor(params):
    feet = neutral_feet(params)
    assert tuple(feet) == LEG_CODES
    assert feet["lf"] == pytest.approx(
        (0.100 + 0.120 * math.cos(math.pi / 4), 0.060 + 0.120 * math.sin(math.pi / 4), 0.008)
    )
    assert feet["rm"] == pytest.approx((0.0, -0.200, 0.008))


def poses(params):
    h = stand_height(params)
    return [
        BodyPose(h + 0.015),
        BodyPose(h - 0.015),
        BodyPose(h, x=0.020),
        BodyPose(h, x=-0.020),
        BodyPose(h, y=0.020),
        BodyPose(h, y=-0.020),
        BodyPose(h, roll=10 * DEG),
        BodyPose(h, roll=-10 * DEG),
        BodyPose(h, pitch=10 * DEG),
        BodyPose(h, pitch=-10 * DEG),
        BodyPose(h, yaw=15 * DEG),
        BodyPose(h, yaw=-15 * DEG),
        BodyPose(h + 0.01, x=0.01, y=-0.01, roll=5 * DEG, pitch=-4 * DEG, yaw=8 * DEG),
    ]


def test_planted_feet_stay_planted_for_poses(params):
    planted = neutral_feet(params)
    for pose in poses(params):
        angles = pose_angles(params, pose)
        for leg in LEG_CODES:
            there = foot_ground_position(params, leg, pose, angles[leg])
            assert there == pytest.approx(planted[leg], abs=1e-6), (leg, pose)


def test_feet_override_is_used(params):
    pose = BodyPose(stand_height(params))
    feet = {leg: p.copy() for leg, p in neutral_feet(params).items()}
    feet["lm"] = feet["lm"] + np.array([0.0, 0.010, 0.0])
    base = pose_angles(params, pose)
    moved = pose_angles(params, pose, feet)
    for leg in LEG_CODES:
        same = moved[leg] == pytest.approx(tuple(base[leg]), abs=1e-12)
        assert same == (leg != "lm"), leg
    there = foot_ground_position(params, "lm", pose, moved["lm"])
    assert there == pytest.approx(feet["lm"], abs=1e-6)


def test_unreachable_pose_names_the_leg(params):
    with pytest.raises(BodyPoseError) as info:
        pose_angles(params, BodyPose(0.5))
    assert info.value.leg in LEG_CODES
    assert info.value.leg in str(info.value)
    assert isinstance(info.value.cause, KinematicsError)


@pytest.mark.parametrize(
    "pose",
    [
        BodyPose(float("nan")),
        BodyPose(0.098, x=float("inf")),
        BodyPose(0.098, yaw=float("nan")),
        BodyPose(0.098, roll=math.pi / 2),
        BodyPose(0.098, pitch=-math.pi / 2),
    ],
)
def test_bad_poses_raise_value_error(params, pose):
    with pytest.raises(ValueError):
        pose_angles(params, pose)


def test_feet_missing_a_leg_is_an_error_naming_it(params):
    feet = neutral_feet(params)
    del feet["rr"]
    with pytest.raises(ValueError, match="rr"):
        pose_angles(params, BodyPose(stand_height(params)), feet)


@pytest.mark.parametrize("bad", [[float("nan"), 0.0, 0.008], [0.1, 0.0], [0.1, 0.0, float("inf")]])
def test_a_bad_foot_is_an_error_naming_its_leg(params, bad):
    feet = neutral_feet(params)
    feet["lr"] = bad
    with pytest.raises(ValueError, match="lr"):
        pose_angles(params, BodyPose(stand_height(params)), feet)
