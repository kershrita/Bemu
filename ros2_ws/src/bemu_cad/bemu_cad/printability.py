"""Printability review of the printed parts, computed from the CAD: size in the print orientation,
the support each part needs (an estimate from downward-facing surfaces), and the material used.

Orientation: each part names the leg or body axis that points up when printing (Part.print_axes).
Either direction along it is possible; the better one (less overhang) is recommended.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bemu_cad.export import DENSITY_G_PER_MM3, FILL_FACTOR
from bemu_cad.leg import Part

OVERHANG_ANGLE = 45.0  # degrees from vertical: steeper-than-this undersides need support
FIRST_LAYER = 0.3  # mm: faces this close to the bed are the first layer, not overhangs
SUPPORT_AREA = 25.0  # mm^2 of overhang below which a part is called supportless
AXIS = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class PrintAnalysis:
    name: str
    size_mm: tuple[float, float, float]  # footprint x, footprint y, height in the print orientation
    up_sign: int  # +1 or -1 along the part's print axis (the recommended direction)
    overhang_mm2: float  # downward-facing area above the first layer, in the recommended direction
    overhang_other_mm2: float  # in the opposite direction
    supports: bool
    mass_g: float  # one part, at the export's fill factor


def _overhang(vertices: np.ndarray, triangles: np.ndarray, axis: int, sign: int) -> float:
    tri = vertices[triangles]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(normal, axis=1)
    area = length / 2.0
    n_up = sign * normal[:, axis] / np.where(length == 0, 1, length)
    height = sign * tri[:, :, axis].min(axis=1)
    floor = (sign * vertices[:, axis]).min()
    steep = n_up < -math.cos(math.radians(OVERHANG_ANGLE))
    return float(area[steep & (height > floor + FIRST_LAYER)].sum())


def analyze(part: Part) -> PrintAnalysis:
    vertices, triangles = part.solid.tessellate(0.05, 0.2)
    vertices = np.array([[v.X, v.Y, v.Z] for v in vertices])
    triangles = np.array(triangles)
    axis = AXIS[part.print_axes]
    plus = _overhang(vertices, triangles, axis, +1)
    minus = _overhang(vertices, triangles, axis, -1)
    sign, best, other = (1, plus, minus) if plus <= minus else (-1, minus, plus)
    box = part.solid.bounding_box()
    size = [box.size.X, box.size.Y, box.size.Z]
    height = size.pop(axis)
    footprint = sorted(size, reverse=True)
    mass = part.solid.volume * DENSITY_G_PER_MM3[part.material] * FILL_FACTOR
    return PrintAnalysis(
        part.name,
        (footprint[0], footprint[1], height),
        sign,
        best,
        other,
        best > SUPPORT_AREA,
        mass,
    )
