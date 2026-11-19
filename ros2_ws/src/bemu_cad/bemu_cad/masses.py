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


def sync_robot_params(path: Path, masses: LinkMasses) -> None:
    """Rewrite the three `link_mass` numbers and the chassis budget (grams) in the parameter file,
    keeping its comments."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    # the file keeps 0.1 g: take the chassis budget from the rounded masses so the total is exact
    masses = LinkMasses(
        *(round(v * 1000, 1) / 1000 for v in (masses.coxa, masses.femur, masses.tibia))
    )
    for name, value in (("coxa", masses.coxa), ("femur", masses.femur), ("tibia", masses.tibia)):
        pattern = re.compile(rf"^(\s+{name}:\s*)[0-9.]+(\s*(?:#.*)?)$", re.MULTILINE)
        grams = f"{value * 1000:.1f}".rstrip("0").rstrip(".")
        text, count = pattern.subn(rf"\g<1>{grams}\g<2>", text, count=1)
        if count != 1:
            raise ValueError(f"{path}: could not find the link_mass line for {name}")
    chassis = re.compile(r"^(\s+mass:\s*)[0-9.]+(\s*#\s*chassis.*)$", re.MULTILINE)
    grams = f"{masses.chassis_budget() * 1000:.1f}".rstrip("0").rstrip(".")
    text, count = chassis.subn(rf"\g<1>{grams}\g<2>", text, count=1)
    if count != 1:
        raise ValueError(f"{path}: could not find the chassis mass line")
    path.write_text(text, encoding="utf-8")
