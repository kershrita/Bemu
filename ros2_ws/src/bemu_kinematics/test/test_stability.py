import math

import numpy as np
import pytest

from bemu_kinematics import stability_margin

SQUARE = [(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)]


def test_centre_of_a_square_is_half_a_side_from_every_edge():
    assert stability_margin((0.0, 0.0), SQUARE) == pytest.approx(0.1)


def test_point_on_an_edge_has_zero_margin():
    assert stability_margin((0.1, 0.03), SQUARE) == pytest.approx(0.0, abs=1e-12)


def test_point_outside_has_negative_margin_equal_to_the_distance():
    assert stability_margin((0.15, 0.0), SQUARE) == pytest.approx(-0.05)
    # outside past a corner: distance to the corner
    assert stability_margin((0.2, 0.2), SQUARE) == pytest.approx(-math.hypot(0.1, 0.1))


def test_interior_feet_change_nothing():
    assert stability_margin((0.02, 0.0), [*SQUARE, (0.0, 0.0), (0.05, 0.02)]) == pytest.approx(
        stability_margin((0.02, 0.0), SQUARE)
    )


def test_order_of_the_feet_does_not_matter():
    shuffled = [SQUARE[2], SQUARE[0], SQUARE[3], SQUARE[1]]
    assert stability_margin((0.03, -0.02), shuffled) == pytest.approx(
        stability_margin((0.03, -0.02), SQUARE)
    )


def test_three_dimensional_inputs_use_x_and_y_only():
    feet = [(x, y, 0.008) for x, y in SQUARE]
    assert stability_margin((0.0, 0.0, 0.2), feet) == pytest.approx(0.1)


def test_a_triangle():
    tri = [(0.0, 0.0), (0.3, 0.0), (0.0, 0.3)]
    inradius = (0.3 + 0.3 - math.hypot(0.3, 0.3)) / 2
    assert stability_margin((inradius, inradius), tri) == pytest.approx(inradius)


@pytest.mark.parametrize(
    "feet",
    [
        [],
        [(0.0, 0.0)],
        [(0.0, 0.0), (0.1, 0.0)],
        [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)],
        [(0.0, 0.0), (0.0, 0.0), (0.1, 0.1)],
        [(0.0, 0.0), (0.1, 0.1), (0.0, 0.0), (0.1, 0.1)],
    ],
)
def test_too_few_or_collinear_feet_give_minus_infinity(feet):
    result = stability_margin((0.0, 0.0), np.array(feet).reshape(-1, 2))
    assert result == -math.inf


@pytest.mark.parametrize(
    ("com", "feet"),
    [
        ((float("nan"), 0.0), SQUARE),
        ((0.0, float("inf")), SQUARE),
        ((0.1, 0.1), [*SQUARE[:3], (float("nan"), 0.5)]),
        ((0.0, 0.0), [*SQUARE[:3], (float("inf"), 0.0)]),
    ],
)
def test_non_finite_inputs_give_minus_infinity(com, feet):
    assert stability_margin(com, feet) == -math.inf
