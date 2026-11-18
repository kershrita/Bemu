import math

import numpy as np
import pytest

from bemu_gait import foot_offset, stride_vector

STRIDE = np.array([0.06, -0.02])
LIFT = 0.03


@pytest.mark.parametrize("duty", [1 / 2, 2 / 3, 5 / 6])
def test_foot_path_is_continuous_everywhere(duty):
    n = 4000
    phases = np.arange(n + 1) / n  # includes 1.0, the wrap back to 0
    path = np.array([foot_offset(p % 1.0, duty, STRIDE, LIFT) for p in phases])
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    bound = 4 * (np.linalg.norm(STRIDE) + LIFT) / (n * min(duty, 1 - duty))
    assert steps.max() < bound
    assert path[-1] == pytest.approx(path[0], abs=1e-12)


def test_stance_is_a_straight_line_at_constant_speed():
    duty = 0.5
    for u in (0.0, 0.25, 0.5, 0.9):
        off = foot_offset(u * duty, duty, STRIDE, LIFT)
        assert off[:2] == pytest.approx(STRIDE * (0.5 - u), abs=1e-12)
        assert off[2] == 0.0


def test_swing_lifts_and_returns_to_the_front():
    duty = 0.5
    assert foot_offset(duty, duty, STRIDE, LIFT)[:2] == pytest.approx(-STRIDE / 2, abs=1e-12)
    assert foot_offset(0.75, duty, STRIDE, LIFT)[2] == pytest.approx(LIFT)
    assert foot_offset(0.999999, duty, STRIDE, LIFT)[:2] == pytest.approx(STRIDE / 2, abs=1e-4)
    assert foot_offset(0.999999, duty, STRIDE, LIFT)[2] == pytest.approx(0.0, abs=1e-4)


def test_zero_stride_and_lift_means_the_foot_stays_put():
    for phase in (0.0, 0.3, 0.7):
        assert foot_offset(phase, 0.5, [0.0, 0.0], 0.0) == pytest.approx((0, 0, 0))


@pytest.mark.parametrize("neutral", [(0.185, 0.145), (0.0, 0.2), (-0.185, -0.145)])
def test_forward_stride_is_the_same_for_every_leg(neutral):
    assert stride_vector(0.1, 0.0, 0.0, neutral, 0.5) == pytest.approx((0.05, 0.0))


def test_sideways_and_turning_strides():
    assert stride_vector(0.0, 0.08, 0.0, (0.1, 0.1), 0.5) == pytest.approx((0.0, 0.04))
    # turning counter-clockwise: the stride is tangent to the circle round the body centre
    assert stride_vector(0.0, 0.0, 1.0, (0.2, 0.0), 0.5) == pytest.approx((0.0, 0.1))
    assert stride_vector(0.0, 0.0, 1.0, (0.0, 0.2), 0.5) == pytest.approx((-0.1, 0.0))


def test_mixed_stride_is_the_sum():
    mixed = stride_vector(0.1, 0.05, 0.4, (0.15, -0.1), 0.5)
    assert mixed == pytest.approx(
        np.array(stride_vector(0.1, 0.0, 0.0, (0.15, -0.1), 0.5))
        + np.array(stride_vector(0.0, 0.05, 0.0, (0.15, -0.1), 0.5))
        + np.array(stride_vector(0.0, 0.0, 0.4, (0.15, -0.1), 0.5))
    )
    assert math.isfinite(mixed[0])
