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
    assert 0.18 < leg["total_kg"] < 0.40  # the V1 estimate was 0.185 kg without the coxa servo
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
    assert "dome down, stem up" in foot_row


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


def test_the_body_is_exported_with_counts_and_a_mass_budget(exported):
    data = json.loads((exported / "mass_properties.json").read_text())
    body = data["body"]
    assert body["total_kg"] == pytest.approx(
        body["printed_kg"] + body["coxa_servos_kg"] + body["electronics_placeholders_kg"]
    )
    assert body["margin_kg"] == pytest.approx(body["chassis_budget_kg"] - body["total_kg"])
    assert body["margin_kg"] > 0.0  # the body fits the 2.5 kg design mass budget
    counts = {name: entry["count"] for name, entry in body["parts"].items()}
    assert counts["hip_plate"] == 6 and counts["post_hip"] == 12 and counts["post_main"] == 8
    assert counts["splice_top"] == 4 and counts["splice_bottom"] == 4
    for name in ("top_plate_centre", "bottom_plate_front", "hip_plate", "post_main", "splice_top"):
        assert (exported / f"{name}.stl").stat().st_size > 500, name
    text = (exported / "print_order.md").read_text()
    assert "| top_plate_rear.stl |" in text and "| post_hip.stl | PETG | 12 |" in text


def test_the_printability_report_is_exported_and_fits_the_bed(exported):
    text = (exported / "printability.md").read_text()
    assert "Printability review" in text
    assert "Fits a 200 × 200 mm bed: yes" in text
    order = (exported / "print_order.md").read_text()
    assert "Supports" in order and "up)" in order
    assert "(no supports)" not in order
    assert "flat face down, stem up" not in order and "turn it over" not in text


def test_the_printability_text_agrees_with_its_numbers(exported):
    import re

    from bemu_cad.export_printability import SHRINK_ALLOWANCE

    text = (exported / "printability.md").read_text()
    total = re.search(r"PETG (\d+) g, TPU (\d+) g; with (\d+)% .*?PETG (\d+) g", text)
    petg, _, percent, with_extra = map(int, total.groups())
    assert percent == round((SHRINK_ALLOWANCE - 1) * 100) == 15
    assert with_extra == pytest.approx(petg * SHRINK_ALLOWANCE, abs=1)
    # every part marked as needing supports is named in the list to ask about
    table = text.split("| Part |")[1].split("\n\n")[0]
    needing = {row.split("|")[1].strip() for row in table.splitlines() if "| yes (" in row}
    asked = text.split("## Parts to ask about")[1]
    assert needing and all(f"`{name}`" in asked for name in needing)
    assert "brim" not in text.split("## What a print service must offer")[1].split("##")[0]


def test_the_print_order_and_the_printability_report_quote_the_same_overhangs(exported):
    import re

    order = (exported / "print_order.md").read_text()
    report = (exported / "printability.md").read_text()
    for name in ("coxa_bracket", "foot_bridge", "foot"):
        in_order = re.search(rf"\| {name}\.stl .*?yes \((\d+) mm²\)", order)
        in_report = re.search(rf"\| {name} \|.*?yes \((\d+) mm²\)", report)
        assert in_order and in_report and in_order.group(1) == in_report.group(1), name
