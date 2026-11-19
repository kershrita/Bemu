import json
import sys
from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.export import export_all  # noqa: E402

REPO = Path(__file__).resolve().parents[4]


@pytest.fixture(scope="module")
def exported(tmp_path_factory):
    from bemu_params import load_params

    out = tmp_path_factory.mktemp("cad")
    params = load_params(REPO / "config" / "robot_params.yaml")
    export_all(params, load_components(), out)
    return out


def test_every_part_has_step_and_stl(exported):
    for name in (
        "coxa_bracket",
        "femur_plate_horn",
        "femur_plate_back",
        "knee_spacer_horn",
        "knee_spacer_back",
        "tibia_plate_horn",
        "tibia_plate_back",
        "foot_bridge",
        "foot",
        "tolerance_coupon",
    ):
        assert (exported / f"{name}.step").stat().st_size > 1000, name
        assert (exported / f"{name}.stl").stat().st_size > 500, name  # a box is 684 bytes


def test_mass_properties_round_trip_and_make_sense(exported):
    data = json.loads((exported / "mass_properties.json").read_text())
    assert json.loads(json.dumps(data)) == data
    parts = data["parts"]
    assert set(parts) >= {"coxa_bracket", "tibia_plate_horn", "foot"}
    for name, entry in parts.items():
        assert entry["mass_kg"] > 0.0005, name
        assert len(entry["center_m"]) == 3 and len(entry["inertia_kg_m2"]) == 3
    leg = data["leg"]
    assert leg["servo_mass_kg"] == pytest.approx(3 * 0.055)
    assert 0.18 < leg["total_kg"] < 0.40  # the V1 estimate was 0.185 kg per leg
    assert leg["total_kg"] == pytest.approx(
        leg["printed_kg"] + leg["servo_mass_kg"] + leg["fasteners_kg"]
    )
    assert data["assumptions"]["fill_factor"] > 0 and data["assumptions"]["status"] == "estimate"


def test_the_print_order_lists_every_file_with_quantities(exported):
    text = (exported / "print_order.md").read_text()
    for name in (
        "coxa_bracket",
        "femur_plate_horn",
        "tibia_plate_back",
        "foot",
        "tolerance_coupon",
    ):
        assert f"{name}.stl" in text
    assert "NOT READY TO ORDER" in text  # until the coupon and the servo measurement are done
    rows = {line.split("|")[1].strip(): line for line in text.splitlines() if line.startswith("| ")}
    assert "| 7 |" in rows["coxa_bracket.stl"]  # 6 legs + 1 spare
    assert "| 6 |" in rows["tibia_plate_horn.stl"]
    assert "TPU" in rows["foot.stl"]


def test_the_tool_error_path_is_one_line(monkeypatch, tmp_path, capsys):
    import importlib.util

    spec = importlib.util.spec_from_file_location("tool", REPO / "tools" / "generate_cad.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    blocker = tmp_path / "file"
    blocker.write_text("x")
    monkeypatch.setattr(sys, "argv", ["generate_cad", "--out", str(blocker / "sub")])
    assert module.main() == 1
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and err[0].startswith("generate_cad: ")


def test_the_tool_coupon_only_run(monkeypatch, tmp_path):
    import importlib.util

    spec = importlib.util.spec_from_file_location("tool2", REPO / "tools" / "generate_cad.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, "argv", ["generate_cad", "--out", str(tmp_path), "--coupon-only"])
    assert module.main() == 0
    assert (tmp_path / "tolerance_coupon.stl").exists()
    assert not (tmp_path / "coxa_bracket.stl").exists()


def test_the_print_order_gates_on_measured_values_and_lists_hardware(exported):
    text = (exported / "print_order.md").read_text()
    assert "`servo.case_hole_thread`" in text.split("NOT READY TO ORDER")[1].split("Two steps")[0]
    assert "## Hardware" in text and "| clevis walls | M2 |" in text
    foot_row = next(line for line in text.splitlines() if line.startswith("| foot.stl"))
    assert "turn the part over" in foot_row


def test_the_order_becomes_ready_only_when_the_fit_values_are_measured():
    import dataclasses

    from bemu_cad.components import Field
    from bemu_cad.export import RELEASE_FIELDS, release_blockers

    components = load_components()
    assert release_blockers(components) == list(RELEASE_FIELDS)
    fields = {c: dict(body) for c, body in components.fields.items()}
    for dotted in RELEASE_FIELDS:
        component, name = dotted.split(".", 1)
        old = fields[component][name]
        fields[component][name] = Field(old.value, old.source, "measured")
    measured = dataclasses.replace(components, fields=fields)
    assert release_blockers(measured) == []
