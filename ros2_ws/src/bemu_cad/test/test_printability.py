import pytest

pytest.importorskip("build123d")

from bemu_cad.leg import Part  # noqa: E402
from bemu_cad.printability import analyze  # noqa: E402


def _part(solid, axes="z"):
    return Part("probe", solid, 1, "PETG", (0, 0, 0, 0), (), axes, "fixed")


def _make():
    from build123d import Box, Pos

    return Box, Pos


def test_a_plain_block_needs_no_supports_and_reports_its_size():
    Box, _ = _make()
    a = analyze(_part(Box(40, 20, 10)))
    assert a.size_mm == pytest.approx((40.0, 20.0, 10.0), abs=0.01)
    assert not a.supports and a.overhang_mm2 < 1.0


def test_an_overhanging_shelf_needs_supports_in_the_worse_direction_only():
    Box, Pos = _make()
    stem = Pos(0, 0, 10) * Box(10, 10, 20)
    shelf = Pos(0, 0, 25) * Box(40, 40, 4)  # wide plate on top of a stem: overhangs both ways up
    a = analyze(_part(Box(10, 10, 1) + stem + shelf))
    # printed upright the shelf underside overhangs; flipped, the base plate does not.
    assert a.up_sign in (1, -1)
    assert a.overhang_mm2 <= a.overhang_other_mm2
    assert a.overhang_other_mm2 > 25.0
