"""Actuator model for the simulation: loaded from bemu_sim/config/actuators.yaml."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import yaml

from bemu_params import ParamsError, ServoParams

KEYS = (
    "kp",
    "kv",
    "max_torque_fraction_of_stall",
    "armature",
    "damping",
    "frictionloss",
)


@dataclass(frozen=True)
class ActuatorModel:
    kp: float  # N·m/rad
    kv: float  # N·m·s/rad
    max_torque: float  # N·m
    armature: float  # kg·m^2
    damping: float  # N·m·s/rad
    frictionloss: float  # N·m


def load_actuator_model(path: str | os.PathLike, servo: ServoParams) -> ActuatorModel:
    """Load the actuator file; max_torque is the configured fraction of the servo stall torque."""
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise ParamsError(f"cannot read actuator file {path}: {e}") from e
    except yaml.YAMLError as e:
        raise ParamsError(f"{path} is not valid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ParamsError(f"{path}: expected a mapping at the top level")
    values = {}
    for key in KEYS:
        value = data.get(key)
        if key not in data:
            raise ParamsError(f"missing key: {key}")
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ParamsError(f"{key}: expected a number, got {value!r}")
        if not math.isfinite(value) or value < 0:
            raise ParamsError(f"{key}: must be a finite non-negative number, got {value!r}")
        values[key] = float(value)
    return ActuatorModel(
        kp=values["kp"],
        kv=values["kv"],
        max_torque=values["max_torque_fraction_of_stall"] * servo.stall_torque,
        armature=values["armature"],
        damping=values["damping"],
        frictionloss=values["frictionloss"],
    )
