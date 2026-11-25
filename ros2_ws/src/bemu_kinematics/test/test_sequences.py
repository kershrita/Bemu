import math

import pytest

from bemu_kinematics import (
    BodyPose,
    pose_angles,
    pose_transition,
    ramp,
    sit_down_targets,
    sit_height,
    stand_height,
    stand_up_targets,
)
from bemu_params import LEG_CODES


def same_angles(a, b, tol=1e-9):
    assert tuple(a) == tuple(b) == LEG_CODES
    for leg in LEG_CODES:
        assert a[leg] == pytest.approx(tuple(b[leg]), abs=tol), leg


def test_sit_height_is_half_the_body_thickness(params):
    assert sit_height(params) == pytest.approx(0.02904)  # half the CAD body thickness (1.5b)


def test_ramp_properties():
    assert ramp(0.0) == 0.0
    assert ramp(1.0) == 1.0
    assert ramp(0.5) == pytest.approx(0.5)
    grid = [i / 100 for i in range(101)]
    values = [ramp(s) for s in grid]
    assert all(b >= a for a, b in zip(values, values[1:], strict=False))
    assert values[1] - values[0] < 1e-3
    assert values[-1] - values[-2] < 1e-3
    assert ramp(-1.0) == 0.0
    assert ramp(2.0) == 1.0


def test_stand_up_starts_sitting_and_ends_standing(params):
    targets = stand_up_targets(params)
    assert len(targets) == 101
    same_angles(targets[0], pose_angles(params, BodyPose(sit_height(params))))
    same_angles(targets[-1], pose_angles(params, BodyPose(stand_height(params))))


def test_midpoint_matches_the_pose_at_the_midpoint_height(params):
    targets = stand_up_targets(params)
    middle = (sit_height(params) + stand_height(params)) / 2
    same_angles(targets[50], pose_angles(params, BodyPose(middle)))


def test_sit_down_is_stand_up_reversed(params):
    up = stand_up_targets(params, duration=1.0, rate=20.0)
    down = sit_down_targets(params, duration=1.0, rate=20.0)
    assert len(up) == len(down) == 21
    for a, b in zip(up, reversed(down), strict=True):
        same_angles(a, b)


def test_pose_transition_endpoints_and_length(params):
    start = BodyPose(stand_height(params))
    end = BodyPose(stand_height(params), roll=math.radians(10))
    targets = pose_transition(params, start, end)
    assert len(targets) == 51
    same_angles(targets[0], pose_angles(params, start))
    same_angles(targets[-1], pose_angles(params, end))


@pytest.mark.parametrize(("duration", "rate"), [(0.0, 50.0), (-1.0, 50.0), (1.0, 0.0), (1.0, -5.0)])
def test_invalid_duration_or_rate_is_rejected(params, duration, rate):
    pose = BodyPose(stand_height(params))
    with pytest.raises(ValueError):
        stand_up_targets(params, duration=duration, rate=rate)
    with pytest.raises(ValueError):
        sit_down_targets(params, duration=duration, rate=rate)
    with pytest.raises(ValueError):
        pose_transition(params, pose, pose, duration=duration, rate=rate)
