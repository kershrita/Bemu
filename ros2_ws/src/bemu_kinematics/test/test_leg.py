import math

import numpy as np
import pytest

from bemu_kinematics.leg import (
    JointAngles,
    JointLimitError,
    KinematicsError,
    UnreachableError,
    forward,
    inverse,
)

NEUTRAL_FOOT = (0.120, 0.0, -0.090)
NEUTRAL_ANGLES = (0.0, 0.300494, -1.753631)


def test_inverse_neutral_stance(params):
    assert inverse(params.leg, NEUTRAL_FOOT) == pytest.approx(NEUTRAL_ANGLES, abs=1e-6)


def test_inverse_femur_horizontal_tibia_down(params):
    assert inverse(params.leg, (0.110, 0.0, -0.110)) == pytest.approx(
        (0.0, 0.0, -math.pi / 2), abs=1e-9
    )


def test_forward_neutral_stance(params):
    assert forward(params.leg, JointAngles(*NEUTRAL_ANGLES)) == pytest.approx(
        NEUTRAL_FOOT, abs=1e-6
    )


def test_coxa_rotates_the_foot_about_z(params):
    foot = forward(params.leg, JointAngles(math.radians(30), 0.300494, -1.753631))
    assert math.hypot(foot[0], foot[1]) == pytest.approx(0.120, abs=1e-6)
    assert math.atan2(foot[1], foot[0]) == pytest.approx(math.radians(30), abs=1e-9)
    assert foot[2] == pytest.approx(-0.090, abs=1e-6)


def test_round_trip_10000_random_poses(params):
    leg = params.leg
    rng = np.random.default_rng(0)
    lo = np.array([leg.coxa_limits[0], leg.femur_limits[0], leg.tibia_limits[0]])
    hi = np.array([leg.coxa_limits[1], leg.femur_limits[1], leg.tibia_limits[1]])
    kept = 0
    while kept < 10_000:
        angles = JointAngles(*rng.uniform(lo, hi))
        foot = forward(leg, angles)
        reach = math.hypot(foot[0], foot[1]) - leg.coxa_length
        if reach < 0.005:
            continue
        kept += 1
        recovered = inverse(leg, foot)
        assert recovered == pytest.approx(tuple(angles), abs=1e-9)
        assert forward(leg, recovered) == pytest.approx(foot, abs=1e-8)
    assert kept == 10_000


@pytest.mark.parametrize(
    "foot, reason",
    [
        ((0.300, 0.0, 0.0), "too_far"),
        ((0.055, 0.0, -0.010), "too_close"),
        ((0.030, 0.0, -0.090), "behind_hip"),
    ],
)
def test_unreachable_targets_are_reported(params, foot, reason):
    with pytest.raises(UnreachableError) as e:
        inverse(params.leg, foot)
    assert e.value.reason == reason
    assert isinstance(e.value, KinematicsError)
    assert str(foot[0]) in str(e.value) or f"{foot[0]:.3f}" in str(e.value)


def test_joint_limit_is_reported_with_the_joint_name(params):
    with pytest.raises(JointLimitError) as e:
        inverse(params.leg, (0.100, 0.150, -0.090))  # coxa would need 56.3 degrees
    assert e.value.joint == "coxa"
    assert "56.3" in str(e.value)
    assert "45.0" in str(e.value)


def test_limits_can_be_skipped(params):
    a = inverse(params.leg, (0.100, 0.150, -0.090), check_limits=False)
    assert a.coxa == pytest.approx(0.982794, abs=1e-6)


def test_full_extension_does_not_fail_on_rounding(params):
    a = inverse(params.leg, (0.220, 0.0, 0.0), check_limits=False)  # d == Lf + Lt exactly
    assert a == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_exception_constructors_take_message_then_attribute():
    assert UnreachableError("m", "too_far").reason == "too_far"
    assert JointLimitError("m", "tibia").joint == "tibia"
    assert str(JointLimitError("m", "tibia")) == "m"


@pytest.mark.parametrize(
    "foot",
    [
        (float("nan"), 0.0, -0.09),
        (0.12, float("inf"), -0.09),
        (0.12, 0.0),
        (0.12, 0.0, -0.09, 1.0),
    ],
)
@pytest.mark.parametrize("check_limits", [True, False])
def test_malformed_targets_raise_plain_value_error(params, foot, check_limits):
    # Plain ValueError from the input guard, not an UnreachableError/JointLimitError.
    with pytest.raises(ValueError, match="finite|shape|three") as info:
        inverse(params.leg, foot, check_limits=check_limits)
    assert not isinstance(info.value, KinematicsError)


def test_unreachable_target_still_raises_without_limit_checks(params):
    with pytest.raises(UnreachableError):
        inverse(params.leg, (0.5, 0.0, 0.0), check_limits=False)
