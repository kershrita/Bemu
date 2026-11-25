"""The tolerance coupon: the first thing to print and measure (docs/08, "Printing through someone
else"). It carries servo pockets at three allowances, hole rows for the screws and heat-set inserts,
and pilot holes for the servo's own bracket screws. The owner measures the printed coupon, picks the
allowance and hole sizes, and puts them in the parameter file before ordering real parts.

Millimetres, in the coupon's own frame: x 0..170, y 0..60, z 0..10 (print flat, z up).
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Box, Cylinder, Pos

from bemu_cad.components import Components
from bemu_cad.leg import Part

LENGTH, WIDTH, HEIGHT = 170.0, 60.0, 10.0
FLOOR = 4.0  # mm of material under a blind pocket or hole
POCKET_ALLOWANCES = (0.1, 0.2, 0.3)  # mm per side
M2_CLEARANCE = (2.2, 2.3, 2.4, 2.5, 2.6)
M3_CLEARANCE = (3.2, 3.3, 3.4, 3.5)
INSERT_HOLES = (4.0, 4.1, 4.2)  # heat-set insert pockets, mm
PILOT_HOLES = (1.5, 1.6, 1.7)  # for the servo's own self-tapping bracket screws, mm
HOLE_ROW_Y = 12.0
POCKET_Y = 40.0
NOTCH_Y = (18.0, 21.0)


@dataclass(frozen=True)
class Cell:
    kind: str  # servo_pocket, clearance_m2, clearance_m3, insert_m3 or pilot
    x: float  # mm, centre
    y: float
    z_mid: float  # a height inside the feature, for point tests
    diameter: float  # mm (0 for a pocket)
    allowance: float  # mm per side (0 for holes)
    notches: int  # grooves cut next to the cell: its position in its row, from 1


@dataclass(frozen=True)
class Coupon:
    part: Part
    cells: tuple[Cell, ...]


def _hole(x: float, y: float, diameter: float, z0: float, z1: float):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def build_coupon(components: Components) -> Coupon:
    servo = components.servo
    length, width = servo.size[0] * 1000, servo.size[1] * 1000
    body = Pos(LENGTH / 2, WIDTH / 2, HEIGHT / 2) * Box(LENGTH, WIDTH, HEIGHT)
    cells: list[Cell] = []

    def notch(x: float, count: int):
        nonlocal body
        for i in range(count):
            centre = x + (i - (count - 1) / 2) * 1.6
            body -= Pos(centre, sum(NOTCH_Y) / 2, HEIGHT - 0.3) * Box(
                0.8, NOTCH_Y[1] - NOTCH_Y[0], 0.6
            )

    for index, allowance in enumerate(POCKET_ALLOWANCES, 1):
        x = 29.0 + (index - 1) * 56.0
        body -= Pos(x, POCKET_Y, (FLOOR + HEIGHT) / 2 + 0.5) * Box(
            length + 2 * allowance, width + 2 * allowance, HEIGHT - FLOOR + 1.0
        )
        cells.append(Cell("servo_pocket", x, POCKET_Y, (FLOOR + HEIGHT) / 2, 0.0, allowance, index))
        notch(x, index)

    rows = (
        ("clearance_m2", M2_CLEARANCE, 10.0, True),
        ("clearance_m3", M3_CLEARANCE, 65.0, True),
        ("insert_m3", INSERT_HOLES, 110.0, False),
        ("pilot", PILOT_HOLES, 145.0, False),
    )
    for kind, diameters, x0, through in rows:
        pitch = 10.0
        for index, diameter in enumerate(diameters, 1):
            x = x0 + (index - 1) * pitch
            z0 = -1.0 if through else FLOOR
            body -= _hole(x, HOLE_ROW_Y, diameter, z0, HEIGHT + 1.0)
            cells.append(Cell(kind, x, HOLE_ROW_Y, (FLOOR + HEIGHT) / 2, diameter, 0.0, index))
            notch(x, index)

    part = Part(
        name="tolerance_coupon",
        solid=body,
        count=1,
        material="PETG",
        outline=(0.0, LENGTH, 0.0, WIDTH),
        holes=(),
        print_axes="z",
        group="fixed",
    )
    return Coupon(part, tuple(cells))
