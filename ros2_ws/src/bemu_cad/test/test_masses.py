import dataclasses
import re
import shutil
from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.export import _mass_json  # noqa: E402
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_cad.masses import LinkMasses, link_masses, sync_robot_params  # noqa: E402
from bemu_params import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
PARAMS = REPO / "config" / "robot_params.yaml"


@pytest.fixture(scope="module")
def parts():
    params = load_params(PARAMS)
    components = load_components()
    leg = build_leg(params, components)
    return params, components, leg, link_masses(leg, components)


def test_the_three_link_masses_are_plausible(parts):
    _, _, _, masses = parts
    assert 0.060 < masses.coxa < 0.085  # bracket + femur servo + screws
    assert 0.070 < masses.femur < 0.100  # plates + spacers + knee servo + screws
    assert 0.025 < masses.tibia < 0.050  # plates + bridge + foot + screws


def test_masses_add_up_to_the_exported_leg_total(parts):
    _, components, leg, masses = parts
    total = _mass_json(leg, components)["leg"]["total_kg"]
    coxa_servo = components.servo.mass
    assert masses.coxa + masses.femur + masses.tibia + coxa_servo == pytest.approx(total, abs=1e-6)


def test_a_longer_femur_changes_the_femur_and_tibia_links(params, parts):
    _, components, _, base = parts
    longer = dataclasses.replace(params.leg, femur_length=params.leg.femur_length + 0.010)
    leg = build_leg(dataclasses.replace(params, leg=longer), components)
    masses = link_masses(leg, components)
    assert masses.femur > base.femur + 0.0005  # the plates are longer
    assert masses.coxa == pytest.approx(base.coxa, abs=0.001)


def test_sync_changes_only_the_three_numbers(tmp_path):
    copy = tmp_path / "robot_params.yaml"
    shutil.copy(PARAMS, copy)
    before = copy.read_text().splitlines()
    sync_robot_params(copy, LinkMasses(coxa=0.0712, femur=0.0803, tibia=0.0351))
    after = copy.read_text().splitlines()
    assert len(before) == len(after)
    changed = [(b, a) for b, a in zip(before, after, strict=True) if b != a]
    assert 1 <= len(changed) <= 4  # three links and the chassis budget
    text = "\n".join(after)
    assert re.search(r"coxa:\s*71\.2\b", text)
    assert re.search(r"femur:\s*80\.3\b", text)
    assert re.search(r"tibia:\s*35\.1\b", text)
    for line in before:
        if line.strip().startswith("#"):
            assert line in after  # every comment line survives
    assert load_params(copy).leg.femur_mass == pytest.approx(0.0803)
    assert load_params(copy).body.mass == pytest.approx(
        2.5 - 6 * (0.0712 + 0.0803 + 0.0351), abs=1e-4
    )


def test_the_parameter_file_matches_the_cad(parts):
    params, _, _, masses = parts
    leg = params.leg
    assert leg.coxa_mass == pytest.approx(masses.coxa, abs=0.0005)
    assert leg.femur_mass == pytest.approx(masses.femur, abs=0.0005)
    assert leg.tibia_mass == pytest.approx(masses.tibia, abs=0.0005)
    assert params.body.mass == pytest.approx(
        masses.chassis_budget(), abs=0.001
    )  # 0.1 g rounding x 6
    total = params.body.mass + 6 * (leg.coxa_mass + leg.femur_mass + leg.tibia_mass)
    assert total == pytest.approx(2.5, abs=0.002)  # the design mass


def test_the_body_thickness_in_the_parameter_file_matches_the_cad(parts):
    from bemu_cad.body import build_body

    params, components, leg, _ = parts
    body = build_body(params, components, leg)
    assert params.body.size[2] == pytest.approx(2 * body.layout_mm["half_height"] * 1e-3, abs=1e-4)
