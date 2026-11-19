"""The printability report: sizes, supports and material for every printed part."""

from __future__ import annotations

import dataclasses

from bemu_cad.body import BodyParts
from bemu_cad.components import Components
from bemu_cad.export import LEGS, SPARES
from bemu_cad.export_body import canonical_solid, families
from bemu_cad.leg import LegParts, Part
from bemu_cad.printability import SUPPORT_AREA, PrintAnalysis, analyze, support_label

BED = (200.0, 200.0, 200.0)
SHRINK_ALLOWANCE = 1.15  # +15% for supports, purge and a failed print


def _canonical_parts(leg: LegParts, body: BodyParts) -> list[tuple[Part, int]]:
    """(part in its own frame, copies in the robot) for every printed file except the coupon."""
    result = [(p, LEGS * p.count + SPARES.get(p.name, 0)) for p in leg.parts]
    for members in families(body).values():
        first = members[0]
        result.append(
            (dataclasses.replace(first, solid=canonical_solid(body, first)), len(members))
        )
    return result


def part_analyses(leg: LegParts, body: BodyParts) -> dict[str, PrintAnalysis]:
    found = {}
    for part, _ in _canonical_parts(leg, body):
        found[part.family or part.name] = analyze(part)
        found[part.name] = found[part.family or part.name]
    return found


def printability_report(
    leg: LegParts,
    body: BodyParts,
    components: Components,
    analyses: dict[str, PrintAnalysis] | None = None,
) -> str:
    """The report; `analyses` (from part_analyses) is reused so the print order and this report
    quote the same numbers."""
    analyses = analyses or part_analyses(leg, body)
    rows, grams = [], {"PETG": 0.0, "TPU": 0.0}
    largest = 0.0
    needing = []
    for part, copies in _canonical_parts(leg, body):
        name = part.family or part.name
        a = analyses[name]
        rows.append(
            f"| {name} | {copies} | {a.size_mm[0]:.0f} × {a.size_mm[1]:.0f} × {a.size_mm[2]:.0f} "
            f"| {'+' if a.up_sign > 0 else '-'}{part.print_axes} up "
            f"| {support_label(a)} | {a.mass_g:.1f} |"
        )
        if a.supports:
            needing.append(f"`{name}` ({a.overhang_mm2:.0f} mm²)")
        grams[part.material] += a.mass_g * copies
        largest = max(largest, a.size_mm[0], a.size_mm[1])
    fits = largest <= BED[0]
    lines = [
        "# Printability review",
        "",
        "Computed from the CAD (`bemu_cad.printability`). The support estimate counts surfaces "
        "facing down more steeply than 45 degrees above the first layer, in the better of the two "
        f"directions along the print axis. A part needs supports when that area is over "
        f"{SUPPORT_AREA:.0f} mm²; the column gives the area. Masses use 60% fill (an estimate). "
        "Sizes are footprint × height in the print orientation.",
        "",
        f"**Largest footprint: {largest:.0f} mm. Fits a 200 × 200 mm bed: "
        f"{'yes' if fits else 'NO'}** (the plates are split at x = ±40 mm for this; a bigger bed "
        "would allow fewer pieces).",
        "",
        "| Part | Copies | Size (mm) | Orientation | Supports (overhang) | One part (g) |",
        "|---|---|---|---|---|---|",
        *rows,
        "",
        f"Material for the whole robot: PETG {grams['PETG']:.0f} g, TPU {grams['TPU']:.0f} g; "
        f"with {round((SHRINK_ALLOWANCE - 1) * 100)}% for supports, purge and a failed print: PETG "
        f"{grams['PETG'] * SHRINK_ALLOWANCE:.0f} g (one 1 kg spool is enough), TPU "
        f"{grams['TPU'] * SHRINK_ALLOWANCE:.0f} g (or buy rubber tips).",
        "",
        'Orientation: "+y up" means the part is placed so that the +y axis of its file points up; '
        "plates lie flat, the coxa bracket and the bridge sit on their base. Where a direction is "
        "given as minus, flip the part about a horizontal axis (the better side up). The foot is "
        "printed as modelled: dome down on a small support, stem up (rubber tips can replace it). "
        "Every part fits with its large face on the bed.",
        "",
        "## What a print service must offer",
        "",
        "- Build volume of at least 200 × 200 × 80 mm (the tibia plates are 138 mm long, the "
        "centre plates 190 mm); a larger bed lets the plates be printed in fewer pieces.",
        "- PETG (or ABS/ASA/nylon) with 0.2 mm layers; TPU for the feet is optional.",
        "- Settings from the print order: 4 perimeters (1.6 mm walls), 35% infill.",
        "- The hole allowances (servo pockets, screw and insert holes) are not known until the "
        "tolerance coupon is printed and measured: order it first, alone, and measure it with a "
        "caliper.",
        "",
        "## Parts to ask about",
        "",
        "- Parts that need supports (over "
        + f"{SUPPORT_AREA:.0f}"
        + " mm² of overhang): "
        + (", ".join(needing) or "none")
        + ". Everything else prints without.",
        "- The posts are 8 mm wide and 50 mm tall: if they wobble on the bed, ask for a brim.",
        "- The 3 mm plates are thinner than four perimeters side by side are wide, so they print "
        "solid (the walls fill the thickness).",
    ]
    return "\n".join(lines) + "\n"
