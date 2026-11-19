import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.servo import servo_mass_properties, servo_pocket, servo_solid  # noqa: E402

MM = 1e-3


@pytest.fixture(scope="module")
def spec():
    return load_components().servo


def box_mm(shape):
    b = shape.bounding_box()
    return (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)


def test_the_solid_matches_the_step_envelope(spec):
    x0, x1, y0, y1, z0, z1 = box_mm(servo_solid(spec))
    # STEP (both manufacturer files): case x -10.11..35.11, y +-12.36, z -28.2..7.5; horn to 9.2
    assert (x0, x1) == pytest.approx((-10.11, 35.11), abs=0.05)
    assert (y0, y1) == pytest.approx((-12.36, 12.36), abs=0.05)
    assert (z0, z1) == pytest.approx((-28.2, 9.2), abs=0.05)
    assert x1 - x0 == pytest.approx(45.22, abs=0.05)  # the drawing
    assert z1 - z0 == pytest.approx(37.4, abs=0.2)  # the drawing's 37.25 overall height


def test_the_pocket_clears_the_solid_by_the_allowance_on_every_side(spec):
    allowance = 0.2
    solid, pocket = servo_solid(spec), servo_pocket(spec, allowance * MM)
    s, p = box_mm(solid), box_mm(pocket)
    for lower, upper in ((0, 1), (2, 3), (4, 5)):
        assert s[lower] - p[lower] == pytest.approx(allowance, abs=0.01)
        assert p[upper] - s[upper] == pytest.approx(allowance, abs=0.01)
    assert (solid - pocket).volume == pytest.approx(0.0, abs=1e-6)  # nothing sticks out
    assert pocket.volume > solid.volume


@pytest.mark.parametrize("allowance", [0.0, -0.1 * MM, float("nan")])
def test_a_pocket_without_clearance_is_rejected(spec, allowance):
    with pytest.raises(ValueError, match="allowance"):
        servo_pocket(spec, allowance)


def test_mass_properties(spec):
    props = servo_mass_properties(spec)
    assert props.mass == pytest.approx(0.055)
    x, y, z = props.center  # metres, servo frame
    assert -0.0101 < x < 0.0352 and abs(y) < 1e-3 and -0.0282 < z < 0.0092
    for i in range(3):
        assert props.inertia[i][i] > 0
        for j in range(3):
            assert props.inertia[i][j] == pytest.approx(props.inertia[j][i], abs=1e-12)
    # a 55 g body of this size has inertias of order m (a^2 + b^2) / 12 ~ 1e-5 kg m^2
    assert 1e-6 < props.inertia[2][2] < 1e-4
