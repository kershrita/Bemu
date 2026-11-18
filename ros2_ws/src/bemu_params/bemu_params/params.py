"""Load config/robot_params.yaml into SI-unit dataclasses.

The YAML uses mm, deg and g; everything this module returns is metres, radians and kilograms.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any

import yaml

LEG_CODES: tuple[str, ...] = ("lf", "lm", "lr", "rf", "rm", "rr")
JOINTS: tuple[str, ...] = ("coxa", "femur", "tibia")
ENV_VAR = "BEMU_PARAMS"
EXPECTED_UNITS = {"length": "mm", "angle": "deg", "mass": "g"}

MM = 1e-3
DEG = math.pi / 180.0
GRAM = 1e-3


class ParamsError(ValueError):
    """The parameter file is missing, malformed or physically impossible."""


@dataclass(frozen=True)
class LegParams:
    coxa_length: float  # m
    femur_length: float  # m
    tibia_length: float  # m
    coxa_limits: tuple[float, float]  # rad, (lower, upper)
    femur_limits: tuple[float, float]
    tibia_limits: tuple[float, float]
    coxa_mass: float  # kg
    femur_mass: float
    tibia_mass: float


@dataclass(frozen=True)
class MountParams:
    x: float  # m
    y: float  # m
    yaw: float  # rad


@dataclass(frozen=True)
class StanceParams:
    foot_radius: float  # m
    body_height: float  # m


@dataclass(frozen=True)
class ServoParams:
    model: str
    mass: float  # kg
    stall_torque: float  # N·m
    max_speed: float  # rad/s


@dataclass(frozen=True)
class RobotParams:
    leg: LegParams
    mounts: dict[str, MountParams]  # keys: lf lm lr rf rm rr
    stance: StanceParams
    servo: ServoParams


def _get(node: Any, dotted: str, key: str) -> Any:
    """Return node[key]; `dotted` is the full dotted path of the value, for error messages."""
    if not isinstance(node, dict) or key not in node:
        raise ParamsError(f"missing key: {dotted}")
    return node[key]


def _number(node: Any, parent: str, key: str, *, positive: bool = False) -> float:
    dotted = f"{parent}.{key}"
    value = _get(node, dotted, key)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise ParamsError(f"{dotted}: expected a number, got {value!r}")
    if positive and value <= 0:
        raise ParamsError(f"{dotted}: must be positive, got {value!r}")
    return float(value)


def _limits(node: Any, parent: str, key: str) -> tuple[float, float]:
    dotted = f"{parent}.{key}"
    value = _get(node, dotted, key)
    ok = isinstance(value, list) and len(value) == 2
    ok = ok and all(
        isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v) for v in value
    )
    if not ok:
        raise ParamsError(f"{dotted}: expected [lower, upper] numbers, got {value!r}")
    lower, upper = float(value[0]), float(value[1])
    if lower >= upper:
        raise ParamsError(f"{dotted}: lower limit must be below upper, got {value!r}")
    return lower * DEG, upper * DEG


def _check_units(root: Any) -> None:
    units = _get(root, "units", "units")
    for kind, expected in EXPECTED_UNITS.items():
        got = _get(units, f"units.{kind}", kind)
        if got != expected:
            raise ParamsError(f"units.{kind}: only {expected!r} is supported, got {got!r}")


def _parse_leg(root: Any) -> LegParams:
    leg = _get(root, "leg", "leg")
    limits = _get(leg, "leg.joint_limits", "joint_limits")
    masses = _get(leg, "leg.link_mass", "link_mass")
    lengths = {j: _number(leg, "leg", f"{j}_length", positive=True) * MM for j in JOINTS}
    lims = {j: _limits(limits, "leg.joint_limits", j) for j in JOINTS}
    mass = {j: _number(masses, "leg.link_mass", j, positive=True) * GRAM for j in JOINTS}
    return LegParams(
        coxa_length=lengths["coxa"],
        femur_length=lengths["femur"],
        tibia_length=lengths["tibia"],
        coxa_limits=lims["coxa"],
        femur_limits=lims["femur"],
        tibia_limits=lims["tibia"],
        coxa_mass=mass["coxa"],
        femur_mass=mass["femur"],
        tibia_mass=mass["tibia"],
    )


def _parse_mounts(root: Any) -> dict[str, MountParams]:
    body = _get(root, "body", "body")
    mounts = _get(body, "body.mounts", "mounts")
    if not isinstance(mounts, dict) or set(mounts) != set(LEG_CODES):
        raise ParamsError(f"body.mounts: expected exactly the legs {', '.join(LEG_CODES)}")
    result = {}
    for code in LEG_CODES:
        where = f"body.mounts.{code}"
        mount = mounts[code]
        result[code] = MountParams(
            x=_number(mount, where, "x") * MM,
            y=_number(mount, where, "y") * MM,
            yaw=_number(mount, where, "yaw") * DEG,
        )
    return result


def _parse_stance(root: Any, leg: LegParams) -> StanceParams:
    stance = _get(root, "stance", "stance")
    foot_radius = _number(stance, "stance", "foot_radius", positive=True) * MM
    body_height = _number(stance, "stance", "body_height", positive=True) * MM
    # Neutral stance: the foot sits foot_radius from the coxa axis, body_height below it.
    # The femur/tibia plane must be able to reach it.
    r = foot_radius - leg.coxa_length
    d = math.hypot(r, body_height)
    lo = abs(leg.femur_length - leg.tibia_length)
    hi = leg.femur_length + leg.tibia_length
    if r <= 0 or not lo <= d <= hi:
        raise ParamsError(
            f"stance: neutral foot position is out of reach (femur-to-foot distance "
            f"{d * 1000:.1f} mm, reachable range {lo * 1000:.1f} to {hi * 1000:.1f} mm, "
            f"horizontal reach past the coxa {r * 1000:.1f} mm)"
        )
    return StanceParams(foot_radius=foot_radius, body_height=body_height)


def _parse_servo(root: Any) -> ServoParams:
    servo = _get(root, "servo", "servo")
    model = _get(servo, "servo.model", "model")
    if not isinstance(model, str) or not model:
        raise ParamsError(f"servo.model: expected a non-empty string, got {model!r}")
    return ServoParams(
        model=model,
        mass=_number(servo, "servo", "mass", positive=True) * GRAM,
        stall_torque=_number(servo, "servo", "stall_torque_nm", positive=True),
        max_speed=_number(servo, "servo", "max_speed_rad_s", positive=True),
    )


def load_params(path: str | os.PathLike | None = None) -> RobotParams:
    """Load the parameter file at `path`, or at $BEMU_PARAMS when `path` is None."""
    if path is None:
        path = os.environ.get(ENV_VAR)
        if not path:
            raise ParamsError(f"no parameter file given and {ENV_VAR} is not set")
    try:
        with open(path, encoding="utf-8") as f:
            root = yaml.safe_load(f)
    except OSError as e:
        raise ParamsError(f"cannot read parameter file {path}: {e}") from e
    except yaml.YAMLError as e:
        raise ParamsError(f"{path} is not valid YAML: {e}") from e
    if not isinstance(root, dict):
        raise ParamsError(f"{path}: expected a mapping at the top level")

    _check_units(root)
    leg = _parse_leg(root)
    return RobotParams(
        leg=leg,
        mounts=_parse_mounts(root),
        stance=_parse_stance(root, leg),
        servo=_parse_servo(root),
    )
