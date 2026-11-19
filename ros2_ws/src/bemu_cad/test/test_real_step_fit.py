"""Judge the printed leg against the manufacturer's own 3D model, not our envelope."""

from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_cad.servo import servo_step_solid  # noqa: E402
from bemu_cad.step_probe import servo_features  # noqa: E402
from bemu_params import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
REF = REPO / "docs" / "reference" / "sts3215"
FEETECH = REF / "feetech-ST3215-20200328.stp"
WAVESHARE = REF / "waveshare-ST3215.step"
CONTACT = 1e-3  # mm^3


@pytest.fixture(scope="module")
def leg():
    return build_leg(load_params(REPO / "config" / "robot_params.yaml"), load_components())


@pytest.fixture(scope="module", params=[FEETECH, WAVESHARE], ids=["feetech", "waveshare"])
def real_servos(request, leg):
    solid = servo_step_solid(request.param)
    return {name: t * solid for name, t in leg.servo_transforms.items()}


def test_no_printed_part_or_screw_head_touches_the_real_servo(leg, real_servos):
    problems = {}
    for servo_name, real in real_servos.items():
        group = leg.servo_groups[servo_name]
        for part in leg.parts:
            overlap = part.solid & real
            volume = float(overlap.volume) if overlap else 0.0
            if volume > CONTACT:
                problems[f"{part.name}|{servo_name}"] = round(volume, 2)
        for hardware in leg.hardware:
            overlap = hardware.solid & real
            volume = float(overlap.volume) if overlap else 0.0
            if volume > CONTACT and hardware.group != group:
                problems[f"{hardware.name}|{servo_name}"] = round(volume, 2)
    assert problems == {}


def test_every_servo_screw_hole_lines_up_with_a_real_pilot_hole(leg):
    """Each part hole that takes a servo case screw sits on a pilot hole of the real model."""
    features = servo_features(FEETECH)
    tolerance = 0.06
    placements = {
        "coxa_bracket": ("femur_servo", {"horn", "back"}),
        "femur_plate_horn": ("knee_servo", {"horn"}),
        "femur_plate_back": ("knee_servo", {"back"}),
        "knee_spacer_horn": ("knee_servo", {"horn"}),
        "knee_spacer_back": ("knee_servo", {"back"}),
    }
    from build123d import Pos

    checked = 0
    for part in leg.parts:
        if part.name not in placements:
            continue
        servo_name, sides = placements[part.name]
        transform = leg.servo_transforms[servo_name]
        pilots = []
        for pilot in features.bracket_holes:
            if pilot.side in sides:
                point = (transform * Pos(pilot.x, pilot.y, 0.0)).position
                pilots.append((pilot.side, point.X, point.Z))
        case_holes = [h for h in part.holes if abs(h.diameter - 2.4) < 1e-6]
        for hole in case_holes:
            matches = [
                p
                for p in pilots
                if abs(p[1] - hole.a) < tolerance
                and abs(p[2] - hole.b) < tolerance
                and (hole.face in ("", p[0]))
            ]
            assert matches, (part.name, hole.a, hole.b)
            checked += 1
    assert checked == 4 + 4 + 4 + 4 + 4 + 4  # both clevis walls, both plates, both spacers


@pytest.mark.parametrize(
    ("part_name", "servo_name"),
    [
        ("femur_plate_horn", "femur_servo"),
        ("femur_plate_back", "femur_servo"),
        ("tibia_plate_horn", "knee_servo"),
        ("tibia_plate_back", "knee_servo"),
        ("knee_spacer_horn", "knee_servo"),
        ("knee_spacer_back", "knee_servo"),
    ],
)
def test_bosses_and_spacers_reach_the_real_servo_within_half_a_millimetre(
    leg, real_servos, part_name, servo_name
):
    """Contact, not a gap: the boss end must sit within 0.5 mm of the horn (it is designed to the
    larger of the two manufacturer models), so the horn screws clamp it. Zero overlap is the other
    test; together they pin the boss lengths."""
    part = next(p for p in leg.parts if p.name == part_name)
    distance = part.solid.distance_to(real_servos[servo_name])
    assert distance <= 0.5 + 1e-6, distance
