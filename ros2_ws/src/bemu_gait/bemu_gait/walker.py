"""The phase-driven walker: a velocity command in, 18 joint angles out (docs/07-locomotion.md).

Feet are planned in the nominal frame (the ground frame with the body at x = y = yaw = 0) and
turned into joint angles by `bemu_kinematics.pose_angles`, which also applies the body pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bemu_gait.command import Command
from bemu_gait.footpath import foot_offset, stride_vector
from bemu_gait.gaits import GAITS, Gait
from bemu_gait.params import GaitParams
from bemu_kinematics import (
    BodyPose,
    BodyPoseError,
    JointAngles,
    neutral_feet,
    pose_angles,
    ramp,
    stability_margin,
    stand_height,
)
from bemu_params import LEG_CODES, RobotParams

MAX_DT = 0.1  # s: a longer step means a stalled control loop, which must not move the feet
BISECTION_STEPS = 12
GROUND_EPS = 0.002  # m: a foot lifted less than this counts as on the ground


@dataclass(frozen=True)
class WalkerOutput:
    angles: dict[str, JointAngles]  # all six legs, ready for RobotSim.command
    feet: dict[str, np.ndarray]  # (3,) each, nominal frame, before the body pose
    phases: dict[str, float]  # each leg's phase in [0, 1)
    in_stance: dict[str, bool]
    stability_margin: float  # m: the COM at (pose.x, pose.y) against the stance feet
    scale: float  # stride scale applied, 0 to 1
    gait: str  # the gait being walked, or blended toward
    blending: bool


def _wrap(delta: float) -> float:
    """Shortest signed way round the phase circle, in (-0.5, 0.5]."""
    delta = (delta + 0.5) % 1.0 - 0.5
    return 0.5 if delta == -0.5 else delta


def _check_gait(name: str) -> Gait:
    if name not in GAITS:
        raise ValueError(f"unknown gait {name!r}; valid gaits: {', '.join(GAITS)}")
    return GAITS[name]


class Walker:
    def __init__(
        self, params: RobotParams, gait_params: GaitParams, *, gait: str = "tripod"
    ) -> None:
        target = _check_gait(gait)
        self._params = params
        self._gp = gait_params
        self._neutral = neutral_feet(params)
        self._pose = BodyPose(stand_height(params))
        self._command = Command()
        self._applied = np.zeros(3)  # vx, vy, omega after slewing
        self._master = 0.0
        self._target = target
        self._start = (dict(target.offsets), target.duty, gait_params.cycle_time[target.name])
        self._progress = 1.0
        self._duration = 1.0
        self._stability_scale = 1.0  # slewed; the reach limit is applied separately

    # -- inputs ---------------------------------------------------------------------------------

    def set_command(self, command: Command) -> None:
        if not isinstance(command, Command):
            raise ValueError(f"command must be a Command, got {command!r}")
        values = (command.vx, command.vy, command.omega)
        if not all(
            isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v)
            for v in values
        ):
            raise ValueError(f"command must be three finite numbers, got {command}")
        self._command = command

    def set_gait(self, name: str) -> None:
        target = _check_gait(name)
        offsets, duty, cycle = self._effective()
        self._start = (offsets, duty, cycle)
        self._target = target
        self._progress = 0.0
        self._duration = self._gp.blend_cycles * cycle

    def set_body_pose(self, pose: BodyPose) -> None:
        pose_angles(self._params, pose)  # raises if standing at this pose is not possible
        self._pose = pose

    # -- gait state -----------------------------------------------------------------------------

    def _effective(self) -> tuple[dict[str, float], float, float]:
        """Offsets, duty factor and cycle time now, part-way through a blend."""
        start_offsets, start_duty, start_cycle = self._start
        p = self._progress
        target_cycle = self._gp.cycle_time[self._target.name]
        offsets = {
            leg: (start_offsets[leg] + p * _wrap(self._target.offsets[leg] - start_offsets[leg]))
            % 1.0
            for leg in LEG_CODES
        }
        duty = start_duty + p * (self._target.duty - start_duty)
        cycle = start_cycle + p * (target_cycle - start_cycle)
        return offsets, duty, cycle

    def _slew(self, dt: float) -> None:
        target = np.array([self._command.vx, self._command.vy, self._command.omega], dtype=float)
        linear = target[:2] - self._applied[:2]
        limit = self._gp.max_linear_accel * dt
        norm = float(np.linalg.norm(linear))
        self._applied[:2] = (
            target[:2] if norm <= limit else self._applied[:2] + linear / norm * limit
        )
        angular = target[2] - self._applied[2]
        limit = self._gp.max_angular_accel * dt
        self._applied[2] = (
            target[2] if abs(angular) <= limit else self._applied[2] + math.copysign(limit, angular)
        )

    # -- planning -------------------------------------------------------------------------------

    def _plan(self, scale, strides, largest, top, phases, duty):
        # Lift follows the command (none at zero command) and falls linearly with the scale, so a
        # stability reaction lowers feet gradually instead of dropping them off a cliff.
        presence = ramp(top * largest / self._gp.lift_ramp_stride)
        lift = self._gp.step_height * presence * (scale / top if top > 0 else 0.0)
        feet = {}
        for leg in LEG_CODES:
            offset = foot_offset(phases[leg], duty, strides[leg] * scale, lift)
            feet[leg] = self._neutral[leg] + offset
        down = [
            feet[leg][:2] for leg in LEG_CODES if feet[leg][2] - self._neutral[leg][2] < GROUND_EPS
        ]
        margin = stability_margin((self._pose.x, self._pose.y), np.array(down).reshape(-1, 2))
        try:
            angles = pose_angles(self._params, self._pose, feet)
        except BodyPoseError:
            return None, feet, margin
        return angles, feet, margin

    def step(self, dt: float) -> WalkerOutput:
        if not (isinstance(dt, int | float) and math.isfinite(dt) and 0.0 < dt <= MAX_DT):
            raise ValueError(f"dt must be in (0, {MAX_DT}] s, got {dt!r}")

        if self._progress < 1.0:
            self._progress = min(1.0, self._progress + dt / self._duration)
        offsets, duty, cycle = self._effective()
        self._master = (self._master + dt / cycle) % 1.0
        self._slew(dt)

        phases = {leg: (self._master + offsets[leg]) % 1.0 for leg in LEG_CODES}
        vx, vy, omega = self._applied
        stance_time = duty * cycle
        strides = {
            leg: stride_vector(vx, vy, omega, self._neutral[leg][:2], stance_time)
            for leg in LEG_CODES
        }
        largest = max(float(np.linalg.norm(s)) for s in strides.values())
        top = 1.0 if largest == 0.0 else min(1.0, self._gp.max_stride / largest)

        def largest_scale(ok, limit):
            """The largest scale in [0, limit] for which ok(scale) holds (0 if none does)."""
            if ok(limit):
                return limit
            low, high = 0.0, limit
            for _ in range(BISECTION_STEPS):
                mid = (low + high) / 2.0
                low, high = (mid, high) if ok(mid) else (low, mid)
            return low

        def reachable(scale):
            return self._plan(scale, strides, largest, top, phases, duty)[0] is not None

        def stable(scale):
            angles, _, margin = self._plan(scale, strides, largest, top, phases, duty)
            return angles is not None and margin >= self._gp.stability_threshold

        # Reach is a hard limit, applied at once. Stability is soft: its scale is slew-limited so
        # a safety reaction never moves a foot by a jump.
        hard = largest_scale(reachable, top)
        wanted = largest_scale(stable, hard)
        if wanted < self._stability_scale:
            self._stability_scale = max(
                wanted, self._stability_scale - self._gp.scale_fall_per_s * dt
            )
        else:
            self._stability_scale = min(
                wanted, self._stability_scale + self._gp.scale_rise_per_s * dt
            )
        scale = min(hard, self._stability_scale)
        angles, feet, margin = self._plan(scale, strides, largest, top, phases, duty)
        assert angles is not None
        return WalkerOutput(
            angles=angles,
            feet=feet,
            phases=phases,
            in_stance={leg: phases[leg] < duty for leg in LEG_CODES},
            stability_margin=margin,
            scale=scale,
            gait=self._target.name,
            blending=self._progress < 1.0,
        )
