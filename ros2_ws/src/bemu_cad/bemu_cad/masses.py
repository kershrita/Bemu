"""Per-link masses of the leg from the CAD, for the simulation model.

The coxa link is the coxa bracket with the femur servo and the clevis screws; the femur link is
the femur plates and knee spacers with the knee servo and its screws; the tibia link is the tibia
plates, foot bridge and foot with their screws. The coxa servo belongs to the body. Printed masses
use the same fill-factor estimate as the export (weigh the printed parts to replace it).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from bemu_cad.components import Components
from bemu_cad.export import part_mass_properties, screw_mass_kg
from bemu_cad.leg import LegParts

# which link each screw group moves with (see leg.screws): it rotates with the plate it clamps
SCREW_LINK = {
    "coxa horn": "coxa",
    "clevis walls": "coxa",
    "femur horn side": "femur",
    "femur back side": "femur",
    "knee servo": "femur",
    "knee horn side": "tibia",
    "knee back side": "tibia",
    "foot bridge": "tibia",
}
PART_LINK = {"fixed": "coxa", "femur": "femur", "tibia": "tibia"}
SERVO_LINK = {"femur_servo": "coxa", "knee_servo": "femur"}  # the coxa servo is body mass


DESIGN_MASS = 2.5  # kg, the whole robot (docs/08)


@dataclass(frozen=True)
class LinkMasses:
    coxa: float  # kg
    femur: float
    tibia: float

    @property
    def leg(self) -> float:
        return self.coxa + self.femur + self.tibia

    def chassis_budget(self, design_mass: float = DESIGN_MASS) -> float:
        """What the body may weigh (kg) so that six legs plus the body equal the design mass."""
        return design_mass - 6 * self.leg


@dataclass(frozen=True)
class LinkInertia:
    """A link's centre of mass from its joint (mm) and inertia about it (g mm^2), in leg axes."""

    com_mm: tuple[float, float, float]
    inertia_g_mm2: tuple[float, float, float, float, float, float]  # xx yy zz xy xz yz


def _matrix_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def link_inertias(leg: LegParts, components: Components) -> dict[str, LinkInertia]:
    """Centre of mass and inertia of each link from the CAD: printed parts and servos as solids
    (uniform density), screws as point masses at the centre of mass of the rest."""
    members = {"coxa": [], "femur": [], "tibia": []}
    for part in leg.parts:
        props = part_mass_properties(part)
        link = PART_LINK[part.group]
        members[link].append((props["mass_kg"], props["center_m"], props["inertia_kg_m2"]))
    for name, link in SERVO_LINK.items():
        solid = leg.servo_solids[name]
        scale = components.servo.mass / solid.volume
        matrix = solid.matrix_of_inertia
        centre = solid.center()
        members[link].append(
            (
                components.servo.mass,
                [centre.X * 1e-3, centre.Y * 1e-3, centre.Z * 1e-3],
                [[float(matrix[i][j]) * scale * 1e-6 for j in range(3)] for i in range(3)],
            )
        )
    origin = {
        "coxa": leg.axes_mm["coxa"],
        "femur": leg.axes_mm["femur"],
        "tibia": leg.axes_mm["knee"],
    }
    result = {}
    for link, items in members.items():
        total = sum(m for m, _, _ in items)
        com = [sum(m * c[k] for m, c, _ in items) / total for k in range(3)]
        tensor = [[0.0] * 3 for _ in range(3)]
        for mass, centre, inertia in items:
            d = [centre[k] - com[k] for k in range(3)]
            d2 = sum(v * v for v in d)
            shifted = [
                [inertia[i][j] + mass * ((d2 if i == j else 0.0) - d[i] * d[j]) for j in range(3)]
                for i in range(3)
            ]
            tensor = _matrix_add(tensor, shifted)
        to_g_mm2 = 1e9  # kg m^2 -> g mm^2
        result[link] = LinkInertia(
            com_mm=tuple(com[k] * 1e3 - origin[link][k] for k in range(3)),
            inertia_g_mm2=(
                tensor[0][0] * to_g_mm2,
                tensor[1][1] * to_g_mm2,
                tensor[2][2] * to_g_mm2,
                tensor[0][1] * to_g_mm2,
                tensor[0][2] * to_g_mm2,
                tensor[1][2] * to_g_mm2,
            ),
        )
    return result


def link_masses(leg: LegParts, components: Components) -> LinkMasses:
    total = {"coxa": 0.0, "femur": 0.0, "tibia": 0.0}
    for part in leg.parts:
        total[PART_LINK[part.group]] += part_mass_properties(part)["mass_kg"] * part.count
    for name, link in SERVO_LINK.items():
        assert name in leg.servo_solids
        total[link] += components.servo.mass
    for joint, thread, count, length in leg.screws:
        total[SCREW_LINK[joint]] += count * screw_mass_kg(thread, length)
    return LinkMasses(**total)


def sync_robot_params(
    path: Path,
    masses: LinkMasses,
    body_thickness: float | None = None,
    inertias: dict[str, LinkInertia] | None = None,
) -> None:
    """Rewrite the three `link_mass` numbers and the chassis budget (grams) in the parameter file,
    keeping its comments."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    # the file keeps 0.1 g: take the chassis budget from the rounded masses so the total is exact
    masses = LinkMasses(
        *(round(v * 1000, 1) / 1000 for v in (masses.coxa, masses.femur, masses.tibia))
    )
    start = text.find("link_mass:")
    if start < 0:
        raise ValueError(f"{path}: could not find link_mass")
    head, block = text[:start], text[start:]
    for name, value in (("coxa", masses.coxa), ("femur", masses.femur), ("tibia", masses.tibia)):
        pattern = re.compile(rf"^(\s+{name}:\s*)[0-9.]+(\s*(?:#.*)?)$", re.MULTILINE)
        grams = f"{value * 1000:.1f}".rstrip("0").rstrip(".")
        block, count = pattern.subn(rf"\g<1>{grams}\g<2>", block, count=1)
        if count != 1:
            raise ValueError(f"{path}: could not find the link_mass line for {name}")
    text = head + block
    chassis = re.compile(r"^(\s+mass:\s*)[0-9.]+(\s*#\s*chassis.*)$", re.MULTILINE)
    grams = f"{masses.chassis_budget() * 1000:.1f}".rstrip("0").rstrip(".")
    text, count = chassis.subn(rf"\g<1>{grams}\g<2>", text, count=1)
    if count != 1:
        raise ValueError(f"{path}: could not find the chassis mass line")
    if body_thickness is not None:  # mm in the file; the sim's body box is the CAD body's height
        size = re.compile(
            r"^(\s+size:\s*\{x:\s*[0-9.]+,\s*y:\s*[0-9.]+,\s*z:\s*)[0-9.]+(\}.*)$", re.MULTILINE
        )
        text, count = size.subn(rf"\g<1>{body_thickness:.2f}\g<2>", text, count=1)
        if count != 1:
            raise ValueError(f"{path}: could not find the body size line")
    if inertias is not None:
        text = _write_inertia_block(text, inertias)
    path.write_text(text, encoding="utf-8")


def _write_inertia_block(text: str, inertias: dict[str, LinkInertia]) -> str:
    """Replace (or add, after the link_mass block) the generated link_inertia block."""
    lines = text.split("\n")
    start = next((i for i, ln in enumerate(lines) if ln.strip().startswith("link_inertia:")), None)
    if start is not None:
        end = start + 1
        while end < len(lines) and (lines[end].startswith("    ") or lines[end].startswith("   #")):
            end += 1
        del lines[start:end]
        insert_at = start
    else:
        mass_at = next(i for i, ln in enumerate(lines) if ln.strip().startswith("link_mass:"))
        tibia = next(
            i for i, ln in enumerate(lines) if i > mass_at and ln.lstrip().startswith("tibia:")
        )
        insert_at = tibia + 1
    block = [
        "  link_inertia:         # from the CAD (--sync-masses): COM from the joint in mm, inertia",
        "    # about the COM in g mm^2 (xx yy zz xy xz yz), leg axes at zero joint angle",
    ]
    for name in ("coxa", "femur", "tibia"):
        item = inertias[name]
        com = ", ".join(f"{v:.2f}" for v in item.com_mm)
        inertia = ", ".join(
            f"{v + 0.0:.0f}".replace("-0", "0") if abs(v) < 0.5 else f"{v:.0f}"
            for v in item.inertia_g_mm2
        )
        block.append(f"    {name}: {{com_mm: [{com}], inertia_g_mm2: [{inertia}]}}")
    lines[insert_at:insert_at] = block
    return "\n".join(lines)
