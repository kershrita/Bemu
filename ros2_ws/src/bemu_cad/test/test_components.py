from pathlib import Path

import pytest
import yaml

from bemu_cad.components import ComponentsError, load_components, unverified_fields

REPO = Path(__file__).resolve().parents[4]
COMPONENTS = REPO / "config" / "components.yaml"
ALLOWED = {"datasheet", "cad", "standard", "unverified", "measured"}


def test_servo_values_from_the_datasheet():
    servo = load_components(COMPONENTS).servo
    assert servo.size == pytest.approx((0.04522, 0.02472, 0.035))
    assert servo.mass == pytest.approx(0.055)
    assert servo.horn_screw == "M3x6"
    assert servo.horn_circle == pytest.approx(0.0192)
    assert servo.output_axis == pytest.approx((0.01011, 0.0))
    assert len(servo.horn_holes) == 4
    for hole in servo.horn_holes:
        assert (hole.x**2 + hole.y**2) ** 0.5 == pytest.approx(
            0.007, abs=1e-6
        )  # on a dia 14 circle
    assert "37.25" in str(servo.drawing_heights_mm) or 37.25 in servo.drawing_heights_mm
    assert "12 V" in servo.variant


def test_every_field_has_a_source_file_that_exists_and_a_valid_status():
    raw = yaml.safe_load(COMPONENTS.read_text())
    count = 0

    def walk(node, path):
        nonlocal count
        if isinstance(node, dict) and "value" in node:
            count += 1
            assert node["status"] in ALLOWED, path
            source = node["source"]
            assert source, path
            if node["status"] in ("datasheet", "cad", "measured"):
                relative = source.split("#")[0].split(" ")[0]
                assert (REPO / relative).exists(), (path, source)
        elif isinstance(node, dict):
            for key, child in node.items():
                walk(child, f"{path}.{key}")

    walk({k: v for k, v in raw.items() if k != "units"}, "")
    assert count > 20


def test_unverified_fields_are_reported():
    fields = unverified_fields(load_components(COMPONENTS))
    assert "insert_m3.hole_diameter" in fields
    assert "servo.size" not in fields


def write(tmp_path, text):
    path = tmp_path / "components.yaml"
    path.write_text(text)
    return path


GOOD = """
units: {length: mm, mass: g}
servo:
  model: {value: X, source: docs/reference/sts3215/README.md, status: datasheet}
  variant: {value: "12 V", source: docs/reference/sts3215/README.md, status: datasheet}
  size: {value: [45.22, 24.72, 35], source: docs/reference/sts3215/README.md, status: datasheet}
"""


def test_a_missing_servo_field_is_named(tmp_path):
    with pytest.raises(ComponentsError, match=r"servo\.output_axis"):
        load_components(write(tmp_path, GOOD))


def broken(replacement_from, replacement_to):
    text = COMPONENTS.read_text()
    assert replacement_from in text
    return text.replace(replacement_from, replacement_to, 1)


def test_a_non_positive_size_is_rejected(tmp_path):
    text = broken("value: [45.22, 24.72, 35]", "value: [45.22, -24.72, 35]")
    with pytest.raises(ComponentsError, match=r"servo\.size"):
        load_components(write(tmp_path, text))


def test_a_missing_source_is_rejected(tmp_path):
    raw = yaml.safe_load(COMPONENTS.read_text())
    raw["servo"]["mass"]["source"] = ""
    with pytest.raises(ComponentsError, match=r"servo\.mass.*source"):
        load_components(write(tmp_path, yaml.safe_dump(raw)))


def test_an_unknown_status_is_rejected(tmp_path):
    raw = yaml.safe_load(COMPONENTS.read_text())
    raw["servo"]["mass"]["status"] = "probably"
    with pytest.raises(ComponentsError, match=r"servo\.mass.*status"):
        load_components(write(tmp_path, yaml.safe_dump(raw)))


def test_a_file_that_does_not_exist_is_one_clear_error(tmp_path):
    with pytest.raises(ComponentsError, match="cannot read"):
        load_components(tmp_path / "nope.yaml")
