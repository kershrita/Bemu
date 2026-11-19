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

LEGS = 6
DENSITY_G_PER_MM3 = {"PETG": 1.27e-3, "TPU": 1.21e-3}
FILL_FACTOR = 0.6  # perimeters plus 35% infill: an estimate until the printed parts are weighed
FASTENERS_PER_LEG_KG = 0.012  # about twenty small screws and inserts: an estimate
SPARES = {"coxa_bracket": 1, "foot": 2}  # the most loaded part, and the part that wears
SETTINGS = "0.2 mm layers, 4 perimeters, 35% infill (docs/08)"
ORIENTATION = {
    "y": "lie flat on the large face (the leg's lateral axis points up)",
    "z": "base plate down (the leg's vertical axis points up)",
}


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
        "inertia_kg_m2": [[float(matrix[i][j]) * scale * 1e-6 for j in range(3)] for i in range(3)],
        "material": part.material,
    }


def _mass_json(leg: LegParts, components: Components) -> dict:
    parts = {p.name: part_mass_properties(p) for p in leg.parts}
    printed = sum(entry["mass_kg"] for entry in parts.values())  # one of each part per leg
    servos = 3 * components.servo.mass
    return {
        "parts": parts,
        "leg": {
            "printed_kg": printed,
            "servo_mass_kg": servos,
            "fasteners_kg": FASTENERS_PER_LEG_KG,
            "total_kg": printed + servos + FASTENERS_PER_LEG_KG,
        },
        "assumptions": {
            "status": "estimate",
            "fill_factor": FILL_FACTOR,
            "density_g_per_cm3": {k: v * 1000 for k, v in DENSITY_G_PER_MM3.items()},
            "note": "Printed masses are estimates; weigh the printed parts and the finished robot "
            "(version 3) and replace them.",
        },
    }


def _print_order(leg: LegParts, components: Components) -> str:
    unverified = unverified_fields(components)
    lines = [
        "# Print order",
        "",
        "**Status: NOT READY TO ORDER.** Two steps come first (docs/08):",
        "",
        "1. Print `tolerance_coupon` alone, measure it, and set the pocket and hole allowances.",
        "2. Measure the first servo received with a caliper and check which screws come in its "
        "accessory bag (the datasheet does not give the bracket-hole thread).",
        "",
        f"Values still marked unverified in `config/components.yaml` ({len(unverified)}): "
        + ", ".join(f"`{name}`" for name in unverified)
        + ".",
        "",
        f"Settings for every part unless stated: {SETTINGS}. Quantities are for six legs plus "
        "spares. The body parts come in version 1.5b.",
        "",
        "| File | Material | Quantity | Orientation | Note |",
        "|---|---|---|---|---|",
        "| tolerance_coupon.stl | PETG | 1 | base down | print this first, alone |",
    ]
    for part in leg.parts:
        quantity = LEGS * part.count + SPARES.get(part.name, 0)
        note = "rubber tips may replace the printed foot" if part.name == "foot" else ""
        lines.append(
            f"| {part.name}.stl | {part.material} | {quantity} | {ORIENTATION[part.print_axes]} "
            f"| {note} |"
        )
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
    written = {}
    for part in (*leg.parts, coupon.part):
        for suffix, writer in (("step", export_step), ("stl", export_stl)):
            path = out_dir / f"{part.name}.{suffix}"
            if not writer(part.solid, str(path)):
                raise ValueError(f"could not write {path}")
            written[f"{part.name}.{suffix}"] = path
    mass = out_dir / "mass_properties.json"
    mass.write_text(json.dumps(_mass_json(leg, components), indent=2) + "\n")
    order = out_dir / "print_order.md"
    order.write_text(_print_order(leg, components))
    written["mass_properties.json"], written["print_order.md"] = mass, order
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
