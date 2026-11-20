import dataclasses
import math
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
        for x_leg, y_leg in ((6.0, 0.0), (-35.0, 11.0), (-35.0, -11.0), (9.0, 11.0), (9.0, -11.0)):
            point = (t * Pos(x_leg, y_leg, 0)).position
            assert any(p.solid.is_inside(Vector(point.X, point.Y, z)) for p in top), (code, x_leg)
    bottom = parts_named(body, "bottom_plate")
    zb = sum(body.layout_mm["bottom_plate_z"]) / 2
    for code in LEG_CODES:
        point = (body.hip_transforms[code] * Pos(-20.0, 0, 0)).position
        assert any(p.solid.is_inside(Vector(point.X, point.Y, zb)) for p in bottom), code
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
    """Probe the solids: each real pilot position is an empty 2.4 mm hole in the plate."""
    _, _, leg, body = world
    features = servo_features(FEETECH)
    t_servo = leg.servo_transforms["coxa_servo"]
    top = parts_named(body, "top_plate")
    z_top = sum(body.layout_mm["top_plate_z"]) / 2
    z_hip = sum(body.layout_mm["hip_plate_z"]) / 2
    for code in LEG_CODES:
        t_hip = body.hip_transforms[code]
        hip = next(p for p in body.parts if p.name == f"hip_plate_{code}")
        for pilot in features.bracket_holes:
            if pilot.side == "back":
                centre = (t_hip * t_servo * Pos(pilot.x, pilot.y, 0)).position
                plates, z = top, z_top
            else:
                centre = (t_hip * t_servo * Pos(pilot.x, pilot.y, 0)).position
                plates, z = [hip], z_hip
            empty = Vector(centre.X, centre.Y, z)
            solid_near = Vector(centre.X + 1.2 + 0.5, centre.Y, z)  # beyond the 2.4 mm hole
            assert not any(p.solid.is_inside(empty) for p in plates), (code, pilot.side, pilot.x)
            assert any(p.solid.is_inside(solid_near) for p in plates), (code, pilot.side, pilot.x)


def test_no_body_part_touches_a_real_coxa_servo(world):
    _, _, leg, body = world
    real = servo_step_solid(FEETECH)
    for code in LEG_CODES:
        servo = body.hip_transforms[code] * (leg.servo_transforms["coxa_servo"] * real)
        for part in body.parts:
            overlap = part.solid & servo
            volume = float(overlap.volume) if overlap else 0.0
            assert volume <= CONTACT_VOLUME, (code, part.name, volume)


SIT = (math.radians(84.6), math.radians(-135.8))
STAND = (math.radians(17.2), math.radians(-100.4))


@pytest.mark.parametrize("coxa_deg", [-45.0, 0.0, 45.0])
@pytest.mark.parametrize(
    "femur_knee",
    [(0.0, -1.5), (1.57, -0.35), (-1.0, -2.0), SIT, STAND],
    ids=lambda p: f"{p[0]:.2f}",
)
@pytest.mark.parametrize("code", ["lf", "lm"])
def test_a_swinging_leg_never_touches_the_body(world, code, coxa_deg, femur_knee):
    """A front and a middle leg through the coxa range with the femur and knee in five poses; the
    body's screw heads under the hip plates are included."""
    _, _, leg, body = world
    posed = _posed(leg, *femur_knee)
    t_hip = body.hip_transforms[code]
    swing = Rot(Z=coxa_deg)
    problems = {}
    for name, (_, solid) in posed.items():
        if name == "coxa_servo":
            continue  # the coxa servo is part of the body mount, not the swinging leg
        moved = t_hip * (swing * solid)
        for part in (*body.parts, *body.hardware):
            solid = part.solid
            overlap = solid & moved
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
    assert len(grid) >= 30 and len(xs) >= 7 and len(ys) >= 5
    assert all((a - min(xs)) % 20 == 0 for a, _ in grid)  # a 20 mm pitch


def test_the_body_mass_is_an_estimate_with_a_budget_check(world):
    _, _, _, body = world
    assert 0.2 < body.printed_mass_kg < 0.9
    assert body.mass_kg == pytest.approx(
        body.printed_mass_kg + 6 * 0.055 + body.electronics_mass_kg, abs=1e-9
    )


def test_the_body_hardware_is_listed_and_the_hub_passes_through_the_top_plate(world):
    _, _, leg, body = world
    by_where = {row[0]: row for row in body.screws}
    assert by_where["top plate to coxa servo (M2)"][2] == 24
    assert by_where["hip plate to coxa servo (M2)"][2] == 24
    assert by_where["splice plates (M3, with nuts)"][4] is True
    assert (
        by_where["insert grid: heat-set inserts M3 (not screws)"][2] >= 30
    )  # the docs promise at least 5 x 7 positions
    # the hub tip is above the recess floor, so the plate has a through hole there
    top = parts_named(body, "top_plate")
    z_mid = sum(body.layout_mm["top_plate_z"]) / 2
    for code in LEG_CODES:
        hub = (body.hip_transforms[code] * Pos(0, 0, 0)).position
        assert not any(p.solid.is_inside(Vector(hub.X, hub.Y, z_mid)) for p in top)
        assert not any(
            p.solid.is_inside(Vector(hub.X, hub.Y, body.layout_mm["top_plate_z"][1] - 0.3))
            for p in top
        )


def _gap(a, b):
    """The smallest distance (mm) between two solids; 0 when they touch or overlap."""
    return float(a.distance_to(b))


def test_the_battery_pack_clears_every_post_plate_and_coxa_servo(world):
    _, _, leg, body = world
    pack = next(h for h in body.hardware if h.name == "battery_pack")
    obstacles = {p.name: p.solid for p in body.parts if p.name != "battery_tray"}
    obstacles |= {
        h.name: h.solid for h in body.hardware if h.name not in ("battery_pack", "pi_board")
    }
    servo = leg.servo_solids["coxa_servo"]
    obstacles |= {f"coxa_servo_{c}": t * servo for c, t in body.hip_transforms.items()}
    # the pack stands on its tray, 3 mm above the bottom plate: that plate is not an obstacle
    near = {
        name: round(_gap(pack.solid, solid), 2)
        for name, solid in obstacles.items()
        if not name.startswith("bottom_plate") and _gap(pack.solid, solid) < 1.0
    }
    assert near == {}


def test_the_battery_tray_clears_the_posts_and_the_pi_clears_everything(world):
    _, _, leg, body = world
    tray = next(p for p in body.parts if p.name == "battery_tray")
    posts = [p for p in body.parts if p.name.startswith("post")]
    assert {
        p.name: round(_gap(tray.solid, p.solid), 2)
        for p in posts
        if _gap(tray.solid, p.solid) < 1.0
    } == {}
    pi = next(h for h in body.hardware if h.name == "pi_board")
    for code, t in body.hip_transforms.items():
        assert _gap(pi.solid, t * leg.servo_solids["coxa_servo"]) > 1.0, code
    assert all(_gap(pi.solid, p.solid) > 5.0 for p in body.parts if p.name.startswith("top_plate"))
    hips = [p for p in body.parts if p.name.startswith("hip_plate")]
    assert [p.name for p in hips if _gap(tray.solid, p.solid) < 1.0] == []


def test_the_tray_screws_land_on_inserts_in_the_plates(world):
    _, _, _, body = world
    tray = next(p for p in body.parts if p.name == "battery_tray")
    plate = next(p for p in body.parts if p.name == "pi_plate")
    bottom = [h for p in parts_named(body, "bottom_plate") for h in p.holes]
    top = [h for p in parts_named(body, "top_plate") for h in p.holes]
    for h in tray.holes:
        assert any(
            abs(i.a - h.a) < 1e-6 and abs(i.b - h.b) < 1e-6 and abs(i.diameter - 4.1) < 1e-6
            for i in bottom
        ), h
    for h in plate.holes:
        assert any(
            abs(i.a - h.a) < 1e-6 and abs(i.b - h.b) < 1e-6 and abs(i.diameter - 4.1) < 1e-6
            for i in top
        ), h


def test_every_plate_piece_has_at_least_two_posts_under_it(world):
    _, _, _, body = world
    for family in ("top_plate", "bottom_plate"):
        for piece in parts_named(body, family):
            box = piece.solid.bounding_box()
            posts = [
                p
                for p in body.parts
                if p.name.startswith("post_main")
                and box.min.X <= p.solid.bounding_box().min.X
                and p.solid.bounding_box().max.X <= box.max.X
            ]
            assert len(posts) >= 2, piece.name


def test_the_pi_standoffs_sit_at_the_boards_hole_pattern(world):
    _, components, _, body = world
    plate = next(p for p in body.parts if p.name == "pi_plate")
    pitch_x, pitch_y = components.fields["raspberry_pi_5"]["hole_pattern"].value
    z = plate.solid.bounding_box().max.Z - 1.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            assert plate.solid.is_inside(Vector(sx * (pitch_x / 2 + 2.2), sy * pitch_y / 2, z))
            # the pilot hole is empty at the very centre
            assert not plate.solid.is_inside(Vector(sx * pitch_x / 2, sy * pitch_y / 2, z))


POSES = [STAND, SIT, (0.0, -1.5), (1.57, -0.35), (0.5, -1.0)]
TOWARD = (-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0)


def test_neighbouring_legs_do_not_touch_up_to_a_combined_45_degrees_toward_each_other(world):
    from bemu_cad.fit import FREE_TOWARD_SUM, neighbour_collisions

    _, _, leg, body = world
    free = [(a, b) for a in TOWARD for b in TOWARD if a + b <= FREE_TOWARD_SUM]
    assert neighbour_collisions(leg, body.hip_transforms, free, POSES) == []


def test_neighbouring_legs_do_touch_at_a_combined_60_degrees(world):
    """The limit is real: 15 degrees more than the free sum collides in this pose set."""
    from bemu_cad.fit import neighbour_collisions

    _, _, leg, body = world
    assert neighbour_collisions(leg, body.hip_transforms, [(30.0, 30.0), (45.0, 15.0)], POSES)


def test_the_strap_slots_connect_through_a_channel_under_the_tray(world):
    from bemu_cad.trays import SLOT_GROOVE, SLOT_X, SLOT_Y

    _, _, _, body = world
    tray = next(p for p in body.parts if p.name == "battery_tray")
    z0 = tray.solid.bounding_box().min.Z
    for sx in (-1, 1):
        assert not tray.solid.is_inside(Vector(sx * SLOT_X, SLOT_Y, z0 + 1.5))  # the slot
        assert not tray.solid.is_inside(
            Vector(sx * SLOT_X, 0.0, z0 + SLOT_GROOVE / 2)
        )  # the channel
        assert tray.solid.is_inside(Vector(sx * SLOT_X, 0.0, z0 + SLOT_GROOVE + 0.5))  # base above


def test_the_pi_screws_fit_the_standoffs_and_the_board_hangs_past_the_pattern(world):
    from bemu_cad.trays import PI_STANDOFF

    _, components, _, body = world
    pi = components.fields["raspberry_pi_5"]
    rows = {row[0]: row for row in body.screws}
    length = rows["Raspberry Pi 5 to the plate (M2.5, self-tapping)"][3]
    board_thickness = 1.6  # a Pi's circuit board
    assert length - board_thickness <= 5.0 <= PI_STANDOFF[1]  # the 5 mm pilot takes the screw
    for name in (
        "battery tray to the bottom plate (M3)",
        "Pi plate to the top deck (M3, into the grid inserts)",
    ):
        assert rows[name][3] <= 3.0 + 4.0 - 1.0  # 3 mm part + 4 mm plate: nothing sticks through
    box = next(h for h in body.hardware if h.name == "pi_board").solid.bounding_box()
    centre = (box.min.X + box.max.X) / 2
    expected = 85 / 2 - pi["hole_inset"].value - pi["hole_pattern"].value[0] / 2
    assert centre == pytest.approx(expected, abs=0.01)
