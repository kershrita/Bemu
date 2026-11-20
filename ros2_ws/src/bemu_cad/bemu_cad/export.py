"""Write the CAD outputs: STEP and STL per part, mass properties, and the print order."""

from __future__ import annotations

import json
from pathlib import Path

from build123d import export_step, export_stl

from bemu_cad.components import Components, unverified_fields
from bemu_cad.coupon import build_coupon
from bemu_cad.leg import LegParts, Part, build_leg
from bemu_cad.servo import M_TO_MM
from bemu_params import RobotParams

# Fields every fit-critical feature depends on: the order is NOT READY until each is `measured`
# on the real servo (docs/08).
RELEASE_FIELDS = (
    "servo.case_hole_thread",
    "servo.case_z_range",
    "servo.horn_disc_diameter",
    "servo.horn_z_range",
    "servo.rear_idler_z_range",
    "servo.rear_hub_diameter",
    "servo.rear_hub_z_range",
    "printing.pocket_allowance",
)
LEGS = 6
DENSITY_G_PER_MM3 = {"PETG": 1.27e-3, "TPU": 1.21e-3}
FILL_FACTOR = 0.6  # perimeters plus 35% infill: an estimate until the printed parts are weighed
# screw mass model, grams (steel): M2 0.2 g + 0.025 g/mm, M3 0.5 g + 0.055 g/mm (head plus shank)
SCREW_G = {"M2": (0.2, 0.025), "M3": (0.5, 0.055)}
SPARE_COPIES = (
    1  # one extra of every leg part (spec section 5.4: a failed part means another order)
)
SPARES = {"foot": 2}  # the part that wears: two extra
SETTINGS = "0.2 mm layers, 4 perimeters, 35% infill (docs/08)"
ORIENTATION = {
    "y": "lie flat on the large face (the leg's lateral axis points up)",
    "z": "base plate down (the leg's vertical axis points up)",
}


def spares(name: str) -> int:
    """Spare copies of one leg part in the order."""
    return SPARES.get(name, SPARE_COPIES)


def part_mass_properties(part: Part) -> dict:
    """Mass, centre of mass (m, in the part's frame) and inertia about it (kg m^2)."""
    solid = part.solid
    mass = solid.volume * DENSITY_G_PER_MM3[part.material] * FILL_FACTOR / 1000.0
    scale = mass / solid.volume  # kg per mm^3 of solid
    matrix = solid.matrix_of_inertia
    centre = solid.center()
    return {
        "mass_kg": mass,
        "center_m": [centre.X / M_TO_MM, centre.Y / M_TO_MM, centre.Z / M_TO_MM],
        "frame": "leg frame at zero joint angles (x outward from the hip axis, y lateral, z up)",
        "inertia_kg_m2": [[float(matrix[i][j]) * scale * 1e-6 for j in range(3)] for i in range(3)],
        "material": part.material,
    }


def screw_mass_kg(thread: str, length: float) -> float:
    head, per_mm = SCREW_G[thread]
    return (head + per_mm * length) / 1000.0


def _mass_json(leg: LegParts, components: Components) -> dict:
    parts = {p.name: part_mass_properties(p) for p in leg.parts}
    printed = sum(entry["mass_kg"] for entry in parts.values())  # one of each part per leg
    servos = 3 * components.servo.mass
    fasteners = sum(
        count * screw_mass_kg(thread, length) for _, thread, count, length in leg.screws
    )
    return {
        "parts": parts,
        "leg": {
            "printed_kg": printed,
            "servo_mass_kg": servos,
            "fasteners_kg": fasteners,
            "total_kg": printed + servos + fasteners,
        },
        "assumptions": {
            "status": "estimate",
            "fill_factor": FILL_FACTOR,
            "density_g_per_cm3": {k: v * 1000 for k, v in DENSITY_G_PER_MM3.items()},
            "note": "Printed masses are estimates; weigh the printed parts and the finished robot "
            "(version 3) and replace them.",
        },
    }


def release_blockers(components: Components) -> list[str]:
    """The fit-critical values not yet measured on the real part or the printed coupon."""
    blockers = []
    for dotted in RELEASE_FIELDS:
        component, name = dotted.split(".", 1)
        field = components.fields.get(component, {}).get(name)
        if field is None or field.status != "measured":
            blockers.append(dotted)
    return blockers


def _print_order(
    leg: LegParts,
    components: Components,
    body_rows: list[str] | None = None,
    body_hardware: list[str] | None = None,
    analyses: dict | None = None,
) -> str:
    from bemu_cad.printability import support_label

    unverified = unverified_fields(components)
    blockers = release_blockers(components)
    if blockers:
        status = [
            "**Status: NOT READY TO ORDER.** These fit-critical values are not yet `measured` "
            "(caliper on the first servo, or the printed coupon): "
            + ", ".join(f"`{b}`" for b in blockers)
            + ".",
        ]
    else:
        status = ["**Status: READY once the coupon has been printed and measured.**"]
    lines = [
        "# Print order",
        "",
        *status,
        "",
        "Two steps come first (docs/08): print `tolerance_coupon` alone and measure it, and "
        "caliper the first servo received (and check which screws come in its accessory bag: the "
        "datasheet does not give the bracket-hole thread).",
        "",
        f"All values still marked unverified in `config/components.yaml` ({len(unverified)}): "
        + ", ".join(f"`{name}`" for name in unverified)
        + ".",
        "",
        f"Settings for every part unless stated: {SETTINGS}. Quantities are for six legs plus "
        "spares; the body rows are the whole body. The electronics trays are designed after the "
        "first parts arrive and are measured (docs/08).",
        "",
        "| File | Material | Quantity | Orientation | Supports | Note |",
        "|---|---|---|---|---|---|",
        "| tolerance_coupon.stl | PETG | 1 | base down | print this first, alone |",
    ]
    for part in leg.parts:
        quantity = LEGS * part.count + spares(part.name)
        note = "rubber tips may replace the printed foot" if part.name == "foot" else ""
        orientation = ORIENTATION[part.print_axes]
        if part.name == "foot":
            orientation = "as modelled: dome down, stem up; the dome needs a small support"
        found = (analyses or {}).get(part.name)
        up = ""
        if found:
            up = f" ({'+' if found.up_sign > 0 else '-'}{part.print_axes} up)"
        supports = "-" if found is None else support_label(found)
        lines.append(
            f"| {part.name}.stl | {part.material} | {quantity} | {orientation}{up} | "
            f"{supports} | {note} |"
        )
    lines += body_rows or []
    lines += [
        "",
        "## Hardware (buy with the parts)",
        "",
        "Screw lengths are computed from the stack heights plus 3 mm of thread engagement and "
        "rounded up to a standard ISO 4762 length. They assume the horn holes and the servo's "
        "bracket holes are as in the STEP models; **check against the first servo** before buying "
        "in bulk (the datasheet does not state the bracket-hole thread; M2 is assumed).",
        "",
        "| Where | Thread | Per leg | Length (mm) |",
        "|---|---|---|---|",
    ]
    for joint, thread, count, length in leg.screws:
        lines.append(f"| {joint} | {thread} | {count} | {length:g} |")
    lines += [
        "",
        "Plus per servo: the centre screw that comes with the horn (M3 x 6), and 3 servos per leg.",
        *(body_hardware or []),
    ]
    return "\n".join(lines) + "\n"


def export_all(
    params: RobotParams,
    components: Components,
    out_dir: Path,
    *,
    allowance: float | None = None,
) -> dict[str, Path]:
    """Build the leg and the coupon and write everything into `out_dir`."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    leg = (
        build_leg(params, components)
        if allowance is None
        else build_leg(params, components, allowance=allowance)
    )
    coupon = build_coupon(components)
    from bemu_cad.body import build_body  # imported here: the body module imports this one
    from bemu_cad.export_body import (
        canonical_solid,
        families,
        hardware_rows,
        mass_json,
        order_rows,
    )
    from bemu_cad.export_printability import part_analyses, printability_report

    body = build_body(params, components, leg)
    written = {}
    outputs = [(part.name, part.solid) for part in (*leg.parts, coupon.part)]
    for family, members in families(body).items():
        outputs.append((family, canonical_solid(body, members[0])))
    for name, solid in outputs:
        for suffix, writer in (("step", export_step), ("stl", export_stl)):
            path = out_dir / f"{name}.{suffix}"
            if not writer(solid, str(path)):
                raise ValueError(f"could not write {path}")
            written[f"{name}.{suffix}"] = path
    mass = out_dir / "mass_properties.json"
    data = _mass_json(leg, components)
    data["body"] = mass_json(body, leg, components)
    mass.write_text(json.dumps(data, indent=2) + "\n")
    order = out_dir / "print_order.md"
    analyses = part_analyses(leg, body)
    order.write_text(
        _print_order(leg, components, order_rows(body, analyses), hardware_rows(body), analyses)
    )
    written["mass_properties.json"], written["print_order.md"] = mass, order
    review = out_dir / "printability.md"
    review.write_text(printability_report(leg, body, components, analyses))
    written["printability.md"] = review
    return written


def export_coupon(components: Components, out_dir: Path) -> dict[str, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    part = build_coupon(components).part
    written = {}
    for suffix, writer in (("step", export_step), ("stl", export_stl)):
        path = out_dir / f"{part.name}.{suffix}"
        if not writer(part.solid, str(path)):
            raise ValueError(f"could not write {path}")
        written[f"{part.name}.{suffix}"] = path
    return written
