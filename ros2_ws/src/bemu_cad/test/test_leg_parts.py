import dataclasses
from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.fit import (  # noqa: E402
    bed_overshoot,
    edge_distance_problems,
    interference_volumes,
    sweep_collisions,
)
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_params import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]

MM = 1e-3


@pytest.fixture(scope="module")
def components():
    return load_components()


@pytest.fixture(scope="module")
def leg(components):
    return build_leg(load_params(REPO / "config" / "robot_params.yaml"), components)


def test_axis_distances_reproduce_the_parameter_file(leg, params):
    axes = leg.axes_mm
    assert axes["femur"][0] - axes["coxa"][0] == pytest.approx(
        params.leg.coxa_length / MM, abs=0.05
    )
    assert axes["knee"][0] - axes["femur"][0] == pytest.approx(
        params.leg.femur_length / MM, abs=0.05
    )
    assert axes["foot"][0] - axes["knee"][0] == pytest.approx(
        params.leg.tibia_length / MM, abs=0.05
    )
    # every servo's output axis, measured on its placed solid, sits where the layout says
    for name, solid in leg.servo_solids.items():
        measured = leg.measured_axis_x(name, solid)
        assert measured == pytest.approx(
            axes[{"coxa_servo": "coxa", "femur_servo": "femur", "knee_servo": "knee"}[name]][0],
            abs=0.05,
        )


def test_every_part_is_named_and_counted(leg):
    names = {p.name: p.count for p in leg.parts}
    assert names == {
        "coxa_bracket": 1,
        "femur_plate_horn": 1,
        "femur_plate_back": 1,
        "knee_spacer_horn": 1,
        "knee_spacer_back": 1,
        "tibia_plate_horn": 1,
        "tibia_plate_back": 1,
        "foot_bridge": 1,
        "foot": 1,
    }
    for part in leg.parts:
        assert part.solid.volume > 100.0 and part.solid.is_valid


def test_no_printed_part_penetrates_a_servo(leg):
    # includes parts fixed to a servo's own group (walls, spacers): only touching is allowed
    problems = {k: v for k, v in interference_volumes(leg, 0.0, 0.0).items() if v > 1e-3}
    assert problems == {}


def test_screw_holes_keep_their_edge_distance(leg):
    assert edge_distance_problems(leg) == []


def test_every_part_fits_the_bed_in_its_print_orientation(leg):
    for part in leg.parts:
        assert bed_overshoot(part, (200.0, 200.0, 200.0)) == [], part.name


def test_a_small_bed_is_reported_with_the_part_and_the_overshoot(leg):
    problems = [p for part in leg.parts for p in bed_overshoot(part, (100.0, 100.0, 100.0))]
    assert any("tibia_plate_horn" in p and "mm" in p for p in problems)


def test_the_joints_sweep_their_full_range_without_collisions(leg, params):
    limits = {
        "femur": tuple(a for a in params.leg.femur_limits),
        "tibia": tuple(a for a in params.leg.tibia_limits),
    }
    assert sweep_collisions(leg, limits, steps=7) == []


def test_the_sweep_does_detect_the_fold_under_pose(leg, params):
    # femur -60 deg with the knee at -150 deg tucks the foot under the hip: a real collision
    limits = {
        "femur": (params.leg.femur_limits[0],) * 2,
        "tibia": (params.leg.tibia_limits[0],) * 2,
    }
    found = sweep_collisions(leg, limits, steps=2, min_foot_radius=-1000.0)
    assert any("foot_bridge|coxa_servo" in line for line in found)
    assert sweep_collisions(leg, limits, steps=2) == []  # skipped: the gaits never go there


def test_a_longer_femur_regenerates_with_the_checks_passing(params, components):
    longer = dataclasses.replace(params.leg, femur_length=params.leg.femur_length + 0.010)
    leg = build_leg(dataclasses.replace(params, leg=longer), components)
    axes = leg.axes_mm
    assert axes["knee"][0] - axes["femur"][0] == pytest.approx(75.0, abs=0.05)
    assert edge_distance_problems(leg) == []
    assert {k for k, v in interference_volumes(leg, 0.0, 0.0).items() if v > 1e-3} == set()
    assert all(bed_overshoot(p, (200.0, 200.0, 200.0)) == [] for p in leg.parts)


def test_a_pocket_without_clearance_is_rejected(params, components):
    with pytest.raises(ValueError, match="allowance"):
        build_leg(params, components, allowance=0.0)
