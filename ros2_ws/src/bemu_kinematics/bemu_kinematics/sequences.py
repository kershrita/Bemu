"""Stand-up, sit-down and pose-transition sequences: one set of 18 targets per control tick."""

from __future__ import annotations

import math

from bemu_kinematics.body import BodyPose, pose_angles, stand_height
from bemu_kinematics.leg import JointAngles
from bemu_params import RobotParams

Targets = dict[str, JointAngles]


def sit_height(params: RobotParams) -> float:
    """Hip-plane height (m) with the body resting on the floor."""
    return params.body.size[2] / 2.0


def ramp(s: float) -> float:
    """Smooth 0 → 1 over s in [0, 1] with zero slope at both ends; clamps outside."""
    s = min(1.0, max(0.0, s))
    return s * s * (3.0 - 2.0 * s)


def _ticks(duration: float, rate: float) -> int:
    if not (math.isfinite(duration) and duration > 0 and math.isfinite(rate) and rate > 0):
        raise ValueError(f"duration and rate must be positive, got {duration!r} and {rate!r}")
    return round(duration * rate)


def pose_transition(
    params: RobotParams,
    start: BodyPose,
    end: BodyPose,
    *,
    duration: float = 1.0,
    rate: float = 50.0,
) -> list[Targets]:
    """Targets moving the body from `start` to `end` with the feet planted, smoothly."""
    steps = _ticks(duration, rate)
    fields = ("height", "x", "y", "roll", "pitch", "yaw")
    targets = []
    for k in range(steps + 1):
        s = ramp(k / steps)
        pose = BodyPose(
            **{f: getattr(start, f) + s * (getattr(end, f) - getattr(start, f)) for f in fields}
        )
        targets.append(pose_angles(params, pose))
    return targets


def stand_up_targets(
    params: RobotParams, *, duration: float = 2.0, rate: float = 50.0
) -> list[Targets]:
    """Raise the body from resting on the floor to standing height."""
    return pose_transition(
        params,
        BodyPose(sit_height(params)),
        BodyPose(stand_height(params)),
        duration=duration,
        rate=rate,
    )


def sit_down_targets(
    params: RobotParams, *, duration: float = 2.0, rate: float = 50.0
) -> list[Targets]:
    """Lower the body from standing height to resting on the floor."""
    return stand_up_targets(params, duration=duration, rate=rate)[::-1]
