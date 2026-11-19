from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.step_probe import servo_features  # noqa: E402

REF = Path(__file__).resolve().parents[4] / "docs" / "reference" / "sts3215"
WAVESHARE = REF / "waveshare-ST3215.step"
FEETECH = REF / "feetech-ST3215-20200328.stp"
MM = 1e-3


@pytest.fixture(scope="module")
def waveshare():
    return servo_features(WAVESHARE)


@pytest.fixture(scope="module")
def feetech():
    return servo_features(FEETECH)


def test_body_envelope_matches_the_drawing(waveshare):
    length, width = waveshare.body_length, waveshare.body_width
    assert length == pytest.approx(45.22, abs=0.05)
    assert width == pytest.approx(24.72, abs=0.05)
    assert waveshare.axis_from_end == pytest.approx(10.11, abs=0.05)


def test_horn_face_holes_are_on_a_14_mm_circle(waveshare):
    holes = sorted(waveshare.horn_holes)
    assert len(holes) == 4
    for x, y, diameter in holes:
        assert (x**2 + y**2) ** 0.5 == pytest.approx(7.0, abs=0.02)
        assert diameter == pytest.approx(2.5, abs=0.05)


def test_bracket_holes_per_face(waveshare):
    horn = {(round(h.x, 1), round(h.y, 2)) for h in waveshare.bracket_holes if h.side == "horn"}
    back = {(round(h.x, 1), round(h.y, 2)) for h in waveshare.bracket_holes if h.side == "back"}
    assert horn == {(8.3, 10.25), (8.3, -10.25), (29.0, 10.25), (29.0, -10.25)}
    assert back == {(8.3, 10.25), (8.3, -10.25), (32.8, 10.25), (32.8, -10.25)}
    for hole in waveshare.bracket_holes:
        assert hole.pilot_diameter == pytest.approx(1.6, abs=0.05)
        assert hole.counterbore_diameter == pytest.approx(3.5, abs=0.05)


def test_hole_depths_from_the_manufacturer_model(feetech):
    depths = {h.side: h.depth for h in feetech.bracket_holes}
    assert depths == {"horn": pytest.approx(3.9, abs=0.1), "back": pytest.approx(4.8, abs=0.1)}
    # the dimensioned patterns of the 2D drawing: 20.7 between 8.3 and 29.0, 24.45 to 32.75
    assert 29.0 - 8.3 == pytest.approx(20.7, abs=0.05)
    assert 32.75 - 8.3 == pytest.approx(24.45, abs=0.05)
    assert 10.25 * 2 == pytest.approx(20.5, abs=0.05)


def test_the_two_manufacturer_models_agree(waveshare, feetech):
    def key(features):
        return sorted((h.side, round(h.x, 1), round(h.y, 1)) for h in features.bracket_holes)

    assert key(waveshare) == key(feetech)
    for a, b in zip(sorted(waveshare.horn_holes), sorted(feetech.horn_holes), strict=True):
        assert a[0] == pytest.approx(b[0], abs=0.1) and a[1] == pytest.approx(b[1], abs=0.1)


def test_the_library_matches_the_probe(waveshare):
    """Review Focus 5: components.yaml and the STEP must not drift apart."""
    servo = load_components().servo
    probed = {(h.side, round(h.x, 1), round(h.y, 2)): h for h in waveshare.bracket_holes}
    assert len(servo.case_holes) == len(probed) == 8
    for hole in servo.case_holes:
        match = probed[(hole.side, round(hole.x / MM, 1), round(hole.y / MM, 2))]
        assert hole.x / MM == pytest.approx(match.x, abs=0.06)
        assert hole.y / MM == pytest.approx(match.y, abs=0.06)
        assert hole.diameter / MM == pytest.approx(match.pilot_diameter, abs=0.06)
        assert hole.counterbore / MM == pytest.approx(match.counterbore_diameter, abs=0.06)
        assert hole.depth / MM in (pytest.approx(3.9, abs=0.1), pytest.approx(4.8, abs=0.1))
    for hole in servo.horn_holes:
        assert any(
            abs(hole.x / MM - x) < 0.06 and abs(hole.y / MM - y) < 0.06
            for x, y, _ in waveshare.horn_holes
        )


def test_a_missing_file_is_one_clear_error(tmp_path):
    with pytest.raises(ValueError, match="cannot read"):
        servo_features(tmp_path / "nope.step")
