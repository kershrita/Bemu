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


# ---- review fixes: heads, bridge, hub, windows, library-driven holes, gating -----------------


def test_screw_heads_are_modelled_and_every_sliding_gap_holds_one(leg):
    assert len(leg.hardware) == 8 + 8 + 4  # clevis walls, knee servo, foot bridge
    layout = leg.layout_mm
    head = layout["screw_head_height"]
    need = head + layout["running_clearance"] - 1e-9
    assert layout["femur_plate_inner_y"] - layout["wall_outer_y"] >= need
    femur_outer = layout["femur_plate_inner_y"] + 3.0
    assert layout["tibia_plate_inner_y"] - femur_outer >= need


def test_the_foot_bridge_has_screw_pilots_and_a_pocket_for_the_foot_stem(leg):
    from build123d import Vector

    bridge = next(p for p in leg.parts if p.name == "foot_bridge")
    foot = next(p for p in leg.parts if p.name == "foot")
    assert len(bridge.holes) == 4 and all(abs(h.diameter - 1.6) < 1e-9 for h in bridge.holes)
    y_end = leg.layout_mm["tibia_plate_inner_y"]
    for hole in bridge.holes:
        sign = 1 if hole.face == "horn" else -1
        inside_pilot = Vector(hole.a, sign * (y_end - 4.0), hole.b)
        assert not bridge.solid.is_inside(inside_pilot)
        assert bridge.solid.is_inside(Vector(hole.a, sign * (y_end - 4.0), hole.b + 1.5))
    assert float((foot.solid & bridge.solid).volume) < 1e-3  # the stem sits in its pocket
    assert foot.solid.distance_to(bridge.solid) <= 0.11


def test_thin_walls_and_servo_pairs_are_checked(leg):
    from bemu_cad.fit import thickness_problems

    assert thickness_problems(leg) == []
    assert "coxa_servo|femur_servo" in interference_volumes(leg, 0.0, 0.0)


def test_a_hole_too_close_to_a_window_or_a_sloped_edge_is_caught(leg):
    from bemu_cad.leg import HoleSpec

    plate = next(p for p in leg.parts if p.name == "femur_plate_horn")
    window = plate.windows[0]
    near = HoleSpec(window.a + window.diameter / 2 + 1.2, window.b, 2.4)  # 1.2 mm from centre edge
    broken = dataclasses.replace(leg, parts=(dataclasses.replace(plate, holes=(near,)),))
    assert any("window" in p for p in edge_distance_problems(broken))

    tibia = next(p for p in leg.parts if p.name == "tibia_plate_horn")
    slope = HoleSpec(185.0, -3.0, 2.4)  # on the sloped lower edge
    broken = dataclasses.replace(leg, parts=(dataclasses.replace(tibia, holes=(slope,)),))
    assert any("edge" in p for p in edge_distance_problems(broken))


def test_the_horn_hole_pattern_comes_from_the_library(params, components):
    servo = components.servo
    wider = tuple(dataclasses.replace(h, x=h.x * 8 / 7, y=h.y * 8 / 7) for h in servo.horn_holes)
    changed = dataclasses.replace(components, servo=dataclasses.replace(servo, horn_holes=wider))
    leg = build_leg(params, changed)
    plate = next(p for p in leg.parts if p.name == "tibia_plate_horn")
    x_k = leg.axes_mm["knee"][0]
    assert any(abs(h.a - (x_k + 8.0)) < 1e-6 and abs(h.b) < 1e-6 for h in plate.holes)


def test_the_pocket_allowance_comes_from_the_library(params, components):
    from bemu_cad.components import Field

    printing = dict(components.fields["printing"])
    printing["pocket_allowance"] = Field(0.3, "test", "measured")
    changed = dataclasses.replace(components, fields={**components.fields, "printing": printing})
    assert build_leg(params, changed).allowance_mm == pytest.approx(0.3)


def test_hardware_lengths_are_standard_and_listed(leg):
    assert leg.screws
    for joint, thread, count, length in leg.screws:
        assert thread in ("M2", "M3") and count > 0
        assert length in (4, 5, 6, 8, 10, 12, 14, 16, 20), (joint, length)


# ---- version 1.5b: the coxa servo is held by the body at both faces --------------------------


def test_the_coxa_shelves_are_32_mm_apart_and_the_support_plate_clears_the_base_plate(leg):
    layout = leg.layout_mm
    assert layout["coxa_back_shelf_z"] - layout["coxa_support_top_z"] == pytest.approx(
        32.0, abs=0.01
    )
    # the support plate sits on the horn-side shelf; the rotating base plate stays below it
    assert layout["coxa_support_bottom_z"] - layout["base_plate_top_z"] >= 0.5
    # the stock horn screw head fits in the boss relief; the boss reaches the horn tip
    assert layout["coxa_horn_tip_z"] - layout["base_plate_top_z"] >= 2.0


def test_horn_screws_cross_their_stack_and_engage_the_horn(leg):
    """C1 of the 1.5b review: the coxa horn screws were left at 6 mm after the boss was added."""
    layout = leg.layout_mm
    screws = {joint: length for joint, _, _, length in leg.screws}
    stack = {
        "coxa horn": 3.0 + (layout["coxa_horn_tip_z"] - layout["base_plate_top_z"]),
        "femur horn side": 3.0 + (layout["femur_plate_inner_y"] - layout["horn_tip_y"]),
        "femur back side": 3.0 + (layout["femur_plate_inner_y"] + layout["rear_idler_y"]),
        "knee horn side": 3.0 + (layout["tibia_plate_inner_y"] - layout["horn_tip_y"]),
        "knee back side": 3.0 + (layout["tibia_plate_inner_y"] + layout["rear_idler_y"]),
    }
    for joint, thickness in stack.items():
        assert screws[joint] >= thickness + 3.0 - 1e-9, (joint, screws[joint], thickness)
