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


def order_rows(body: BodyParts, analyses: dict | None = None) -> list[str]:
    rows = []
    for family, parts in families(body).items():
        first = parts[0]
        found = (analyses or {}).get(family)
        orientation = "upright" if family.startswith("post") else "flat, large face down"
        if found:
            orientation += f" ({'+' if found.up_sign > 0 else '-'}{first.print_axes} up)"
        supports = "-" if found is None else ("yes, small" if found.supports else "no")
        cells = (f"{family}.stl", first.material, len(parts), orientation, supports, "body")
        rows.append("| " + " | ".join(str(c) for c in cells) + " |")
    return rows


def hardware_rows(body: BodyParts) -> list[str]:
    rows = [
        "",
        "### Body hardware",
        "",
        "| Where | Thread | Count | Length (mm) | Nut |",
        "|---|---|---|---|---|",
    ]
    for where, thread, count, length, nut in body.screws:
        length_text = "-" if thread == "insert" else f"{length:g}"
        rows.append(f"| {where} | {thread} | {count} | {length_text} | {'yes' if nut else 'no'} |")
    rows.append("")
    rows.append(
        "Use low-profile (button or countersunk) M3 screws on the underside of the bottom plate, "
        "or the real sit height is the plate thickness plus a head."
    )
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
        "placeholders until the chosen parts are bought and measured. Weigh the parts and the "
        "robot (version 3).",
        "parts": {
            name: part_mass_properties(p[0]) | {"count": len(p)}
            for name, p in families(body).items()
        },
    }
