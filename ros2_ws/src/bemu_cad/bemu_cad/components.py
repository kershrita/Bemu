"""Load config/components.yaml: the dimensions of every bought part that touches a printed part.

Every value carries a source and a status; the loader refuses a value without them, a status it
does not know, a missing servo field, or a size that is not positive. Units in the file are mm
and g; the loader returns SI (m, kg).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

STATUSES = ("datasheet", "cad", "standard", "unverified", "measured")
DEFAULT_PATH = Path(__file__).resolve().parents[4] / "config" / "components.yaml"
MM = 1e-3
G = 1e-3


class ComponentsError(ValueError):
    """The component file is missing, malformed or has an impossible value."""


@dataclass(frozen=True)
class Field:
    value: Any
    source: str
    status: str


@dataclass(frozen=True)
class Hole:
    x: float  # m, in the component frame
    y: float
    diameter: float
    thread: str


@dataclass(frozen=True)
class ServoSpec:
    model: str
    variant: str
    size: tuple[float, float, float]  # m: length, width, height without the horns
    output_axis: tuple[float, float]  # m: from the nearer end, from the width centre line
    drawing_heights_mm: tuple[float, ...]
    horn_circle: float  # m
    horn_screw: str
    horn_holes: tuple[Hole, ...]
    case_holes: tuple[Hole, ...]
    mass: float  # kg
    sources: dict[str, str]
    status: dict[str, str]


@dataclass(frozen=True)
class Components:
    servo: ServoSpec
    fields: dict[str, dict[str, Field]]  # component -> field name -> Field (values in file units)


def _positive(value: Any, where: str) -> None:
    numbers = value if isinstance(value, list) else [value]
    for n in numbers:
        if isinstance(n, list):
            _positive(n, where)
        elif isinstance(n, bool) or not isinstance(n, int | float):
            continue  # a text value (a thread name, a model)
        elif not n > 0:
            raise ComponentsError(f"{where}: value {n!r} must be positive")


def _parse(raw: dict) -> dict[str, dict[str, Field]]:
    fields: dict[str, dict[str, Field]] = {}
    for component, body in raw.items():
        if component == "units":
            continue
        if not isinstance(body, dict):
            raise ComponentsError(f"{component}: expected a mapping of fields")
        fields[component] = {}
        for name, node in body.items():
            where = f"{component}.{name}"
            if not isinstance(node, dict) or "value" not in node:
                raise ComponentsError(f"{where}: needs value, source and status")
            source = str(node.get("source") or "").strip()
            if not source:
                raise ComponentsError(f"{where}: missing source")
            status = node.get("status")
            if status not in STATUSES:
                raise ComponentsError(f"{where}: status must be one of {', '.join(STATUSES)}")
            if name not in ("horn_holes", "case_holes", "output_axis"):  # positions may be 0 or < 0
                _positive(node["value"], where)
            fields[component][name] = Field(node["value"], source, status)
    return fields


def _holes(field: Field, where: str) -> tuple[Hole, ...]:
    try:
        return tuple(
            Hole(h["x"] * MM, h["y"] * MM, h["diameter"] * MM, str(h["thread"]))
            for h in field.value
        )
    except (KeyError, TypeError) as exc:
        raise ComponentsError(f"{where}: holes need x, y, diameter and thread ({exc})") from exc


def _servo(fields: dict[str, dict[str, Field]]) -> ServoSpec:
    body = fields.get("servo")
    if body is None:
        raise ComponentsError("servo: missing component")
    needed = (
        "model",
        "variant",
        "size",
        "output_axis",
        "drawing_heights",
        "horn_circle",
        "horn_screw",
        "horn_holes",
        "case_holes",
        "mass",
    )
    for name in needed:
        if name not in body:
            raise ComponentsError(f"servo.{name}: missing")
    size = tuple(v * MM for v in body["size"].value)
    axis = tuple(v * MM for v in body["output_axis"].value)
    if len(size) != 3 or len(axis) != 2:
        raise ComponentsError("servo.size needs 3 numbers and servo.output_axis needs 2")
    return ServoSpec(
        model=str(body["model"].value),
        variant=str(body["variant"].value),
        size=size,
        output_axis=axis,
        drawing_heights_mm=tuple(float(v) for v in body["drawing_heights"].value),
        horn_circle=body["horn_circle"].value * MM,
        horn_screw=str(body["horn_screw"].value),
        horn_holes=_holes(body["horn_holes"], "servo.horn_holes"),
        case_holes=_holes(body["case_holes"], "servo.case_holes"),
        mass=body["mass"].value * G,
        sources={k: v.source for k, v in body.items()},
        status={k: v.status for k, v in body.items()},
    )


def load_components(path: str | os.PathLike | None = None) -> Components:
    path = Path(path) if path is not None else DEFAULT_PATH
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ComponentsError(f"cannot read component file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ComponentsError(f"{path} is not valid YAML: {exc}") from exc
    if not isinstance(raw, dict):
        raise ComponentsError(f"{path}: expected a mapping")
    fields = _parse(raw)
    return Components(servo=_servo(fields), fields=fields)


def unverified_fields(components: Components) -> list[str]:
    """Dotted names (component.field) of every value whose status is unverified."""
    return [
        f"{component}.{name}"
        for component, body in components.fields.items()
        for name, field in body.items()
        if field.status == "unverified"
    ]
