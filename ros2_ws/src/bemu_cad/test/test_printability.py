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


def test_an_overhanging_shelf_is_printed_the_way_up_that_avoids_supports():
    Box, Pos = _make()
    # a stem on a wide shelf: printed shelf-down (-z up) it needs nothing; the other way the shelf
    # underside (40 x 40 minus the stem footprint) overhangs
    part = Pos(0, 0, 10) * Box(10, 10, 20) + Pos(0, 0, 21) * Box(40, 40, 2)
    a = analyze(_part(part))
    assert a.up_sign == -1
    assert a.overhang_mm2 < 1.0 and not a.supports
    assert a.overhang_other_mm2 == pytest.approx(40 * 40 - 10 * 10, rel=0.02)


def test_the_support_label_states_the_area_and_the_threshold():
    from bemu_cad.printability import SUPPORT_AREA, support_label

    Box, Pos = _make()
    flat = analyze(_part(Box(40, 20, 10)))
    assert support_label(flat) == "no"
    shelf = analyze(_part(Pos(0, 0, 10) * Box(10, 10, 20) + Pos(0, 0, 21) * Box(40, 40, 2), "z"))
    assert SUPPORT_AREA == 25.0
    assert support_label(_with(shelf, overhang_mm2=198.4, supports=True)) == "yes (198 mm²)"


def _with(analysis, **changes):
    import dataclasses

    return dataclasses.replace(analysis, **changes)
