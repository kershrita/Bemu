"""The body's outputs: one STEP/STL per part family, quantities, and the mass budget."""

from __future__ import annotations

from build123d import Pos

from bemu_cad.body import BodyParts
from bemu_cad.export import part_mass_properties
from bemu_cad.leg import LegParts, Part
from bemu_cad.masses import DESIGN_MASS, link_masses


def families(body: BodyParts) -> dict[str, list[Part]]:
    grouped: dict[str, list[Part]] = {}
    for part in body.parts:
        grouped.setdefault(part.family or part.name, []).append(part)
    return grouped


def canonical_solid(body: BodyParts, part: Part):
    """The part in its own frame for printing: hip plates in the leg frame, posts at the origin."""
    if part.family == "hip_plate":
        code = part.name.removeprefix("hip_plate_")
        return body.hip_transforms[code].inverse() * part.solid
    if part.family.startswith("post"):
        box = part.solid.bounding_box()
        return (
            Pos(-(box.min.X + box.max.X) / 2, -(box.min.Y + box.max.Y) / 2, -box.min.Z) * part.solid
        )
    return part.solid


def order_rows(body: BodyParts) -> list[str]:
    rows = []
    for family, parts in families(body).items():
        first = parts[0]
        orientation = "upright" if family.startswith("post") else "flat, large face down"
        rows.append(f"| {family}.stl | {first.material} | {len(parts)} | {orientation} | body |")
    return rows


def mass_json(body: BodyParts, leg: LegParts, components) -> dict:
    budget = link_masses(leg, components).chassis_budget()
    return {
        "printed_kg": body.printed_mass_kg,
        "coxa_servos_kg": 6 * components.servo.mass,
        "electronics_placeholders_kg": body.electronics_mass_kg,
        "total_kg": body.mass_kg,
        "chassis_budget_kg": budget,
        "margin_kg": budget - body.mass_kg,
        "design_mass_kg": DESIGN_MASS,
        "note": "Printed masses are estimates; the Pi, battery, regulator and wiring are "
        "placeholders until version 1.5c picks them. Weigh the parts and the robot (version 3).",
        "parts": {
            name: part_mass_properties(p[0]) | {"count": len(p)}
            for name, p in families(body).items()
        },
    }
