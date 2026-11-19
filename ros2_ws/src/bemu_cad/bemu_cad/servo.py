"""The servo as a solid, its pocket, and its mass properties, in the servo frame.

Frame: origin on the output axis; x along the body toward the long side (the far end is 35.11 mm
away); y across the width; z along the output axis toward the horn. build123d works in millimetres,
so this module converts the library's metres at the edge: callers pass and receive SI values for
numbers, and millimetre solids.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Box, Cylinder, Pos, Solid

from bemu_cad.components import ServoSpec

M_TO_MM = 1000.0


@dataclass(frozen=True)
class MassProperties:
    mass: float  # kg
    center: tuple[float, float, float]  # m, in the part frame
    inertia: tuple[tuple[float, float, float], ...]  # kg m^2 about the centre of mass


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Solid:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _horn(diameter: float, z0: float, z1: float) -> Solid:
    return Pos(0, 0, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def _dimensions(spec: ServoSpec, grow: float = 0.0):
    """Case box and horn disc in mm, each grown by `grow` mm on every face."""
    length, width, _ = (v * M_TO_MM for v in spec.size)
    near = spec.output_axis[0] * M_TO_MM
    z0, z1 = (v * M_TO_MM for v in spec.case_z)
    h0, h1 = (v * M_TO_MM for v in spec.horn_z)
    case = _box(
        -near - grow,
        length - near + grow,
        -width / 2 - grow,
        width / 2 + grow,
        z0 - grow,
        z1 + grow,
    )
    horn = _horn(spec.horn_disc_diameter * M_TO_MM + 2 * grow, h0 - grow, h1 + grow)
    return case, horn


def servo_solid(spec: ServoSpec) -> Solid:
    """The case box and the output horn disc (the envelope a bracket must not touch)."""
    case, horn = _dimensions(spec)
    return case + horn


def servo_pocket(spec: ServoSpec, allowance: float) -> Solid:
    """The solid a bracket cuts to hold the servo: the envelope grown by `allowance` (m)."""
    if not (math.isfinite(allowance) and allowance > 0):
        raise ValueError(f"allowance must be a positive length in metres, got {allowance!r}")
    case, horn = _dimensions(spec, allowance * M_TO_MM)
    return case + horn


def servo_mass_properties(spec: ServoSpec) -> MassProperties:
    """55 g spread evenly over the envelope solid (a real servo's motor makes it denser at the
    centre; the weighed robot in version 3 checks the total, not this split)."""
    solid = servo_solid(spec)
    scale = spec.mass / solid.volume  # kg per mm^3
    center = solid.center()
    matrix = solid.matrix_of_inertia  # mm^5 for unit density
    inertia = tuple(
        tuple(float(matrix[i][j]) * scale * 1e-6 for j in range(3))  # mm^2 -> m^2
        for i in range(3)
    )
    return MassProperties(
        mass=spec.mass,
        center=(center.X / M_TO_MM, center.Y / M_TO_MM, center.Z / M_TO_MM),
        inertia=inertia,
    )
