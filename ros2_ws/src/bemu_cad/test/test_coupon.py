import pytest

pytest.importorskip("build123d")

from build123d import Vector  # noqa: E402

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.coupon import build_coupon  # noqa: E402
from bemu_cad.fit import bed_overshoot  # noqa: E402


@pytest.fixture(scope="module")
def coupon():
    return build_coupon(load_components())


def inside(coupon, x, y, z):
    return coupon.part.solid.is_inside(Vector(x, y, z))


def test_the_coupon_fits_the_bed(coupon):
    assert bed_overshoot(coupon.part, (200.0, 200.0, 200.0)) == []


def test_there_are_servo_pockets_at_three_allowances(coupon):
    pockets = [c for c in coupon.cells if c.kind == "servo_pocket"]
    assert [round(c.allowance, 2) for c in pockets] == [0.1, 0.2, 0.3]
    servo = load_components().servo
    for cell in pockets:
        # inside the pocket is empty, just outside the pocket wall is material (at mid depth)
        half_len = servo.size[0] * 1000 / 2 + cell.allowance
        half_wid = servo.size[1] * 1000 / 2 + cell.allowance
        z = cell.z_mid
        assert not inside(coupon, cell.x, cell.y, z)
        assert not inside(coupon, cell.x + half_len - 0.02, cell.y, z)
        assert inside(coupon, cell.x + half_len + 0.02, cell.y, z)
        assert not inside(coupon, cell.x, cell.y + half_wid - 0.02, z)
        assert inside(coupon, cell.x, cell.y + half_wid + 0.02, z)


def test_every_cell_is_labelled_with_notches_equal_to_its_index(coupon):
    by_kind = {}
    for cell in coupon.cells:
        by_kind.setdefault(cell.kind, []).append(cell)
    for cells in by_kind.values():
        assert [c.notches for c in cells] == list(range(1, len(cells) + 1))


def test_hole_rows_have_the_stated_diameters(coupon):
    kinds = {c.kind for c in coupon.cells}
    assert {"clearance_m2", "clearance_m3", "insert_m3", "pilot"} <= kinds
    for cell in coupon.cells:
        if cell.kind == "servo_pocket":
            continue
        r = cell.diameter / 2
        assert not inside(coupon, cell.x + r - 0.02, cell.y, cell.z_mid)
        assert inside(coupon, cell.x + r + 0.02, cell.y, cell.z_mid)


def test_the_insert_row_spans_the_unverified_default(coupon):
    diameters = [c.diameter for c in coupon.cells if c.kind == "insert_m3"]
    assert diameters == pytest.approx([4.0, 4.1, 4.2])
