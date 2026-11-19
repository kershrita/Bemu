"""Gait defaults from gait.yaml, converted to SI."""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from bemu_gait.gaits import GAITS

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / "gait.yaml"
MM = 1e-3


class GaitParamsError(ValueError):
    """The gait parameter file is missing, malformed or has an impossible value."""


@dataclass(frozen=True)
class GaitParams:
    cycle_time: Mapping[str, float]  # s, one per gait name
    step_height: float  # m
    max_stride: float  # m, per leg per stance
    max_linear_accel: float  # m/s²
    max_angular_accel: float  # rad/s²
    lift_ramp_stride: float  # m: the stride at which the full step height is reached
    blend_cycles: float  # a gait change takes this many cycles of the old gait
    stability_threshold: float  # m
    scale_fall_per_s: float  # the stability (lift) scale may fall this fast (full = 1.0)
    scale_rise_per_s: float  # and recover this fast
    max_pose_shift_rate: float  # m/s: how fast a body height or shift command is applied
    max_pose_tilt_rate: float  # rad/s: and a roll, pitch or yaw command


def _positive(node: Any, parent: str, key: str) -> float:
    dotted = f"{parent}.{key}" if parent else key
    if not isinstance(node, dict) or key not in node:
        raise GaitParamsError(f"missing key: {dotted}")
    value = node[key]
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise GaitParamsError(f"{dotted}: expected a number, got {value!r}")
    if value <= 0:
        raise GaitParamsError(f"{dotted}: must be positive, got {value!r}")
    return float(value)


def load_gait_params(path: str | os.PathLike | None = None) -> GaitParams:
    """Load gait.yaml: the file given, or the one shipped with the package."""
    path = Path(path) if path is not None else DEFAULT_PATH
    try:
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as e:
        raise GaitParamsError(f"cannot read gait file {path}: {e}") from e
    except yaml.YAMLError as e:
        raise GaitParamsError(f"{path} is not valid YAML: {e}") from e
    if not isinstance(root, dict):
        raise GaitParamsError(f"{path}: expected a mapping at the top level")

    times = root.get("cycle_time_s")
    if not isinstance(times, dict):
        raise GaitParamsError("missing key: cycle_time_s (expected one cycle time per gait)")
    unknown = [name for name in times if name not in GAITS]
    if unknown:
        raise GaitParamsError(
            f"cycle_time_s: unknown gait {unknown[0]!r}; known: {', '.join(GAITS)}"
        )
    cycle_time = {name: _positive(times, "cycle_time_s", name) for name in GAITS}

    return GaitParams(
        cycle_time=cycle_time,
        step_height=_positive(root, "", "step_height_mm") * MM,
        max_stride=_positive(root, "", "max_stride_mm") * MM,
        max_linear_accel=_positive(root, "", "max_linear_accel_mm_s2") * MM,
        max_angular_accel=_positive(root, "", "max_angular_accel_rad_s2"),
        lift_ramp_stride=_positive(root, "", "lift_ramp_stride_mm") * MM,
        blend_cycles=_positive(root, "", "blend_cycles"),
        stability_threshold=_positive(root, "", "stability_threshold_mm") * MM,
        scale_fall_per_s=_positive(root, "", "scale_fall_per_s"),
        scale_rise_per_s=_positive(root, "", "scale_rise_per_s"),
        max_pose_shift_rate=_positive(root, "", "max_pose_shift_mm_s") * MM,
        max_pose_tilt_rate=_positive(root, "", "max_pose_tilt_rad_s"),
    )
