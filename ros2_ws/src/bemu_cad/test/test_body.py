import dataclasses
from pathlib import Path

import pytest

pytest.importorskip("build123d")

from build123d import Pos, Rot, Vector  # noqa: E402

from bemu_cad.body import build_body  # noqa: E402
from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.fit import (  # noqa: E402
    CONTACT_VOLUME,
    _posed,
    bed_overshoot,
    edge_distance_problems,
    thickness_problems,
)
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_cad.servo import servo_step_solid  # noqa: E402
from bemu_cad.step_probe import servo_features  # noqa: E402
from bemu_params import LEG_CODES, load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
FEETECH = REPO / "docs" / "reference" / "sts3215" / "feetech-ST3215-20200328.stp"
MM = 1e-3


@pytest.fixture(scope="module")
def world():
    params = load_params(REPO / "config" / "robot_params.yaml")
    components = load_components()
    leg = build_leg(params, components)
    return params, components, leg, build_body(params, components, leg)


def parts_named(body, prefix):
    return [p for p in body.parts if p.name.startswith(prefix)]


def test_the_plates_cover_every_hip_and_every_coxa_servo_footprint(world):
    params, _, leg, body = world
    top = parts_named(body, "top_plate")
    assert len(top) == 3
    z = body.layout_mm["top_plate_z"][1] - 0.5  # above the hub recess floor
    for code in LEG_CODES:
        t = body.hip_transforms[code]
        for x_leg, y_leg in ((0.0, 0.0), (-35.0, 11.0), (-35.0, -11.0), (9.0, 11.0), (9.0, -11.0)):
            point = (t * Pos(x_leg, y_leg, 0)).position
            assert any(p.solid.is_inside(Vector(point.X, point.Y, z)) for p in top), (code, x_leg)
    sizes = [p.solid.bounding_box() for p in top]
    xs = (min(b.min.X for b in sizes), max(b.max.X for b in sizes))
    # the 240 mm decagon plus the padded coxa-servo footprints at the front and rear hips
    assert 240.0 <= xs[1] - xs[0] <= 250.0


def test_every_piece_fits_the_bed_and_a_small_bed_is_reported(world):
    _, _, _, body = world
    for part in body.parts:
        assert bed_overshoot(part, (200.0, 200.0, 200.0)) == [], part.name
    assert any(bed_overshoot(p, (100.0, 100.0, 100.0)) for p in parts_named(body, "top_plate"))


def test_the_coxa_servo_is_seated_at_both_faces_on_the_real_pilot_holes(world):
    _, _, leg, body = world
    features = servo_features(FEETECH)
    tolerance = 0.06
    for code in LEG_CODES:
        t_hip = body.hip_transforms[code]
        t_servo = leg.servo_transforms["coxa_servo"]
        for plate_prefix, side in (("top_plate", "back"), ("hip_plate_" + code, "horn")):
            holes = []
            for part in parts_named(body, plate_prefix):
                holes += [h for h in part.holes if abs(h.diameter - 2.4) < 1e-6]
            assert holes, (code, plate_prefix)
            for pilot in features.bracket_holes:
                if pilot.side != side:
                    continue
                point = (t_hip * t_servo * Pos(pilot.x, pilot.y, 0.0)).position
                if plate_prefix == "top_plate":
                    assert any(
                        abs(h.a - point.X) < tolerance and abs(h.b - point.Y) < tolerance
                        for h in holes
                    ), (code, pilot.x, pilot.y)
                else:  # hip plate holes are in the leg frame
                    local = (t_servo * Pos(pilot.x, pilot.y, 0.0)).position
                    assert any(
                        abs(h.a - local.X) < tolerance and abs(h.b - local.Y) < tolerance
                        for h in holes
                    ), (code, pilot.x, pilot.y)


def test_no_body_part_touches_a_real_coxa_servo(world):
    _, _, leg, body = world
    real = servo_step_solid(FEETECH)
    for code in LEG_CODES:
        servo = body.hip_transforms[code] * (leg.servo_transforms["coxa_servo"] * real)
        for part in body.parts:
            overlap = part.solid & servo
            volume = float(overlap.volume) if overlap else 0.0
            assert volume <= CONTACT_VOLUME, (code, part.name, volume)


@pytest.mark.parametrize("coxa_deg", [-45.0, 0.0, 45.0])
@pytest.mark.parametrize("femur_knee", [(0.0, -1.5), (1.57, -0.35), (-1.0, -2.0)])
def test_a_swinging_leg_never_touches_the_body(world, coxa_deg, femur_knee):
    """The left-front leg through the coxa range with the femur and knee in three poses."""
    _, _, leg, body = world
    posed = _posed(leg, *femur_knee)
    t_hip = body.hip_transforms["lf"]
    swing = Rot(Z=coxa_deg)
    problems = {}
    for name, (_, solid) in posed.items():
        if name == "coxa_servo":
            continue  # the coxa servo is part of the body mount, not the swinging leg
        moved = t_hip * (swing * solid)
        for part in body.parts:
            overlap = part.solid & moved
            volume = float(overlap.volume) if overlap else 0.0
            if volume > CONTACT_VOLUME:
                problems[f"{name}|{part.name}"] = round(volume, 2)
    assert problems == {}


def test_plate_joints_are_bridged_by_splice_plates(world):
    _, _, _, body = world
    for joint_x in body.layout_mm["joint_x"]:
        for level in ("top", "bottom"):
            splices = [
                p
                for p in parts_named(body, f"splice_{level}")
                if p.solid.bounding_box().min.X < joint_x - 8
                and p.solid.bounding_box().max.X > joint_x + 8
            ]
            assert len(splices) >= 2, (joint_x, level)


def test_holes_keep_their_edge_distance_and_walls_are_thick_enough(world):
    _, _, _, body = world
    assert edge_distance_problems(dataclasses.replace(world[2], parts=body.parts)) == []
    assert thickness_problems(dataclasses.replace(world[2], parts=body.parts)) == []


def test_the_top_plate_carries_the_mounting_grid(world):
    _, _, _, body = world
    grid = {
        (round(h.a), round(h.b))
        for p in parts_named(body, "top_plate")
        for h in p.holes
        if abs(h.diameter - 4.1) < 1e-6
    }
    xs = {a for a, _ in grid}
    ys = {b for _, b in grid}
    assert len(xs) >= 7 and len(ys) >= 5
    assert all((a - min(xs)) % 20 == 0 for a, _ in grid)  # a 20 mm pitch


def test_the_body_mass_is_an_estimate_with_a_budget_check(world):
    _, _, _, body = world
    assert 0.2 < body.printed_mass_kg < 0.9
    assert body.mass_kg == pytest.approx(
        body.printed_mass_kg + 6 * 0.055 + body.electronics_mass_kg, abs=1e-9
    )
