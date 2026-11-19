"""Read hole positions out of a manufacturer's STEP model of the servo.

The servo frame (docs/reference/sts3215/README.md): the output axis is the origin's axis, x runs
along the body toward the far end, y across the width, z along the output axis toward the horn.
The STEP files put the output axis along their own Y, so frame x = X - axis X, y = Z - axis Z.
Lengths here are in millimetres, as in the STEP files.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

AXIS_TOLERANCE = 1e-3
HORN_SCREW_HOLE = 3.2  # the hole for the horn's centre screw: it sits on the output axis


@dataclass(frozen=True)
class BracketHole:
    side: str  # "horn" (the output face) or "back"
    x: float
    y: float
    pilot_diameter: float
    counterbore_diameter: float
    depth: float


@dataclass(frozen=True)
class ServoFeatures:
    body_length: float
    body_width: float
    axis_from_end: float
    horn_holes: tuple[tuple[float, float, float], ...]  # x, y, diameter
    bracket_holes: tuple[BracketHole, ...]


def _cylinders(shape):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder

    for face in shape.faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = surface.Cylinder()
        axis = cylinder.Axis()
        direction = axis.Direction()
        if abs(abs(direction.Y()) - 1.0) > AXIS_TOLERANCE:
            continue  # only holes along the output axis direction
        point = axis.Location()
        box = face.bounding_box()
        yield 2 * cylinder.Radius(), point.X(), point.Z(), box.min.Y, box.max.Y


def _merge(values, key_digits=1):
    merged = {}
    for key, value in values:
        merged.setdefault(tuple(round(k, key_digits) for k in key), value)
    return merged


def servo_features(step_path: str | os.PathLike) -> ServoFeatures:
    from build123d import import_step

    try:
        shape = import_step(os.fspath(step_path))
    except Exception as exc:  # OCP raises several unrelated types for a bad file
        raise ValueError(f"cannot read STEP file {step_path}: {exc}") from exc
    box = shape.bounding_box()

    screw_holes = [c for c in _cylinders(shape) if abs(c[0] - HORN_SCREW_HOLE) < 0.03 and c[4] > 8]
    if not screw_holes:
        raise ValueError(
            f"{step_path}: no output-axis screw hole (diameter {HORN_SCREW_HOLE}) found"
        )
    axis_x, axis_z = screw_holes[0][1], screw_holes[0][2]

    horn, bracket_pilots, counterbores = {}, {}, {}
    for diameter, x, z, y_min, y_max in _cylinders(shape):
        fx, fy = x - axis_x, z - axis_z
        radius = (fx**2 + fy**2) ** 0.5
        if abs(diameter - 2.5) < 0.1 and abs(radius - 7.0) < 0.1 and y_max > 6.0:
            horn[(round(fx, 1), round(fy, 1))] = (fx, fy, diameter)
        elif abs(diameter - 1.6) < 0.1 and radius > 9.0 and (y_max - y_min) >= 1.2:
            if y_max >= 6.0:
                side = "horn"
            elif y_min <= -24.5:
                side = "back"
            else:
                continue
            bracket_pilots[(side, round(fx, 1), round(fy, 2))] = (fx, fy, diameter, y_max - y_min)
        elif abs(diameter - 3.5) < 0.1 and radius > 9.0:
            counterbores[(round(fx, 1), round(fy, 2))] = diameter

    holes = []
    for (side, rx, ry), (fx, fy, diameter, depth) in sorted(bracket_pilots.items()):
        bore = counterbores.get((rx, ry))
        if bore is None:
            continue  # no counterbore: an internal feature, not a bracket hole
        holes.append(BracketHole(side, fx, fy, diameter, bore, depth))
    return ServoFeatures(
        body_length=box.size.X,
        body_width=box.size.Z,
        axis_from_end=axis_x - box.min.X,
        horn_holes=tuple(sorted(horn.values())),
        bracket_holes=tuple(holes),
    )
