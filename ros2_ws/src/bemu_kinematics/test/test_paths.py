import numpy as np
import pytest

from bemu_kinematics import inverse
from bemu_kinematics.paths import circle_path, square_path

CENTER = (0.120, 0.0, -0.090)


def test_square_has_corners_and_stays_in_plane():
    p = square_path(CENTER, 0.060, 400)
    assert p.shape == (400, 3) and np.all(p[:, 1] == 0.0)
    assert p[:, 0].min() == pytest.approx(0.090) and p[:, 0].max() == pytest.approx(0.150)
    assert p[:, 2].min() == pytest.approx(-0.120) and p[:, 2].max() == pytest.approx(-0.060)
    steps = np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1)
    assert steps == pytest.approx(0.240 / 400, abs=1e-9)
    # the four corners are all points of the path
    for cx in (0.090, 0.150):
        for cz in (-0.120, -0.060):
            assert np.min(np.hypot(p[:, 0] - cx, p[:, 2] - cz)) < 1e-9


def test_circle_has_constant_radius_in_xy():
    p = circle_path(CENTER, 0.030, 300)
    assert p.shape == (300, 3)
    assert np.hypot(p[:, 0] - 0.120, p[:, 1]) == pytest.approx(0.030)
    assert np.all(p[:, 2] == -0.090)
    assert not np.allclose(p[0], p[-1])  # first point is not repeated at the end


@pytest.mark.parametrize("fn,size", [(square_path, 0.060), (circle_path, 0.030)])
def test_unknown_plane_is_rejected(fn, size):
    with pytest.raises(ValueError, match="yz"):
        fn(CENTER, size, 100, plane="yz")


def test_every_point_of_both_test_paths_is_reachable(params):
    for path in (square_path(CENTER, 0.060, 400), circle_path(CENTER, 0.030, 300)):
        for point in path:
            inverse(params.leg, point)  # raises if unreachable or outside the limits
