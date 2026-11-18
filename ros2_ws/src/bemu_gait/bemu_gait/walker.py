"""The phase-driven walker: a velocity command in, 18 joint angles out (docs/07-locomotion.md).

Phases decide *when* each leg is in stance or swing. Foot *positions* are state: in stance a foot
moves at exactly the speed the ground moves under the body, whatever the command or the gait does,
so a planted foot never slips when the speed or the gait changes; in swing it travels from where it
lifted off to where the stance will begin. Feet are planned in the nominal frame (the ground frame
with the body at x = y = yaw = 0) and turned into joint angles by `bemu_kinematics.pose_angles`,
which also applies the body pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bemu_gait.command import Command
from bemu_gait.footpath import stride_vector
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
GROUND_EPS = 0.0005  # m: a foot lifted less than this counts as on the ground (it lags a tick)


@dataclass(frozen=True)
class WalkerOutput:
    angles: dict[str, JointAngles]  # all six legs, ready for RobotSim.command
    feet: dict[str, np.ndarray]  # (3,) each, nominal frame, before the body pose
    phases: dict[str, float]  # each leg's phase in [0, 1)
    in_stance: dict[str, bool]
    stability_margin: float  # m: the COM at (pose.x, pose.y) against the feet on the ground
    scale: float  # position scale applied only when a leg would run out of reach, 0 to 1
    speed_scale: float  # how much of the commanded speed the stride cap allowed, 0 to 1
    lift_scale: float  # lift scale applied by the stability reaction, 0 to 1
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
        self._neutral_xy = {leg: self._neutral[leg][:2] for leg in LEG_CODES}
        self._pose = BodyPose(stand_height(params))  # the commanded body pose
        self._applied_pose = self._pose  # the pose in use: slews toward the commanded one
        self._command = Command()
        self._applied = np.zeros(3)  # vx, vy, omega after clamping and slewing
        self._speed_scale = 1.0
        self._master = 0.0
        self._target = target
        self._start = (dict(target.offsets), target.duty, gait_params.cycle_time[target.name])
        self._progress = 1.0
        self._duration = 1.0
        self._lift_scale = 1.0  # the stability reaction's lift scale, slew-limited
        self._xy = {leg: np.zeros(2) for leg in LEG_CODES}  # foot offset from neutral, body frame
        self._swing_start = {leg: np.zeros(2) for leg in LEG_CODES}
        self._was_stance = dict.fromkeys(LEG_CODES, True)

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
        """Start blending to a gait. Asking again for the gait already being walked (or blended
        toward) changes nothing, so a node may republish its request every tick."""
        target = _check_gait(name)
        if target.name == self._target.name:
            return
        offsets, duty, cycle = self._effective()
        self._start = (offsets, duty, cycle)
        self._target = target
        self._progress = 0.0
        self._duration = self._gp.blend_cycles * cycle

    def set_body_pose(self, pose: BodyPose) -> None:
        pose_angles(self._params, pose)  # raises if standing at this pose is not possible
        self._pose = pose  # applied gradually: see _slew_pose

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

    def _strides(self, velocity: np.ndarray, stance_time: float) -> dict[str, np.ndarray]:
        vx, vy, omega = velocity
        return {
            leg: stride_vector(vx, vy, omega, self._neutral_xy[leg], stance_time)
            for leg in LEG_CODES
        }

    def _slew(self, dt: float, stance_time: float) -> None:
        """Clamp the commanded velocity to what the stride cap allows, then slew toward it.

        Clamping first means a command above top speed cannot wind the applied velocity up, so
        stopping always takes top speed / acceleration.
        """
        target = np.array([self._command.vx, self._command.vy, self._command.omega], dtype=float)
        largest = max(float(np.linalg.norm(s)) for s in self._strides(target, stance_time).values())
        self._speed_scale = 1.0 if largest == 0.0 else min(1.0, self._gp.max_stride / largest)
        target = target * self._speed_scale
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

    def _slew_pose(self, dt: float) -> None:
        """Move the applied body pose toward the commanded one, but never into a pose from which
        standing is impossible: the line between two reachable poses can leave the reachable set,
        and then the robot simply holds the last pose it could stand in."""
        a, b = self._applied_pose, self._pose
        move = np.array([b.x - a.x, b.y - a.y, b.height - a.height])
        norm = float(np.linalg.norm(move))
        limit = self._gp.max_pose_shift_rate * dt
        if norm > limit:
            move *= limit / norm
        turn = self._gp.max_pose_tilt_rate * dt

        def toward(now: float, goal: float) -> float:
            return now + max(-turn, min(turn, goal - now))

        candidate = BodyPose(
            height=a.height + move[2],
            x=a.x + move[0],
            y=a.y + move[1],
            roll=toward(a.roll, b.roll),
            pitch=toward(a.pitch, b.pitch),
            yaw=toward(a.yaw, b.yaw),
        )
        try:
            pose_angles(self._params, candidate)
        except BodyPoseError:
            return
        self._applied_pose = candidate

    # -- feet -----------------------------------------------------------------------------------

    def _move_feet(self, dt: float, phases, duty: float, stance_time: float) -> dict[str, float]:
        """Advance every foot's position state; return each swinging leg's progress u (0..1)."""
        vx, vy, omega = self._applied
        swing_u = {}
        for leg in LEG_CODES:
            stance = phases[leg] < duty
            here = self._neutral_xy[leg] + self._xy[leg]
            if stance:
                # the ground moves under the body at -(v + ω × r): a rigid motion about the centre
                ground = np.array([vx - omega * here[1], vy + omega * here[0]])
                self._xy[leg] = self._xy[leg] - ground * dt
            else:
                if self._was_stance[leg]:
                    self._swing_start[leg] = self._xy[leg].copy()
                u = (phases[leg] - duty) / (1.0 - duty)
                touchdown = stride_vector(vx, vy, omega, self._neutral_xy[leg], stance_time) / 2.0
                start = self._swing_start[leg]
                self._xy[leg] = start + ramp(u) * (touchdown - start)
                swing_u[leg] = u
            self._was_stance[leg] = stance
        return swing_u

    def _plan(self, scale, lift_scale, swing_u, largest):
        # Lift follows the command (none at zero command), falls linearly with the scales, and is
        # what the stability reaction lowers.
        presence = ramp(largest / self._gp.lift_ramp_stride)
        lift = self._gp.step_height * presence * scale * lift_scale
        feet = {}
        for leg in LEG_CODES:
            z = lift * math.sin(math.pi * swing_u[leg]) if leg in swing_u else 0.0
            feet[leg] = self._neutral[leg] + np.array([*(scale * self._xy[leg]), z])
        down = [
            feet[leg][:2] for leg in LEG_CODES if feet[leg][2] - self._neutral[leg][2] < GROUND_EPS
        ]
        pose = self._applied_pose
        margin = stability_margin((pose.x, pose.y), np.array(down).reshape(-1, 2))
        try:
            angles = pose_angles(self._params, pose, feet)
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
        stance_time = duty * cycle
        self._slew(dt, stance_time)
        self._slew_pose(dt)

        phases = {leg: (self._master + offsets[leg]) % 1.0 for leg in LEG_CODES}
        swing_u = self._move_feet(dt, phases, duty, stance_time)
        largest = max(
            float(np.linalg.norm(s)) for s in self._strides(self._applied, stance_time).values()
        )

        def largest_value(ok, limit):
            """The largest value in [0, limit] for which ok(value) holds (0 if none does)."""
            if ok(limit):
                return limit
            low, high = 0.0, limit
            for _ in range(BISECTION_STEPS):
                mid = (low + high) / 2.0
                low, high = (mid, high) if ok(mid) else (low, mid)
            return low

        def plan(scale, lift_scale):
            return self._plan(scale, lift_scale, swing_u, largest)

        # Reach is a hard limit, applied at once.
        scale = largest_value(lambda sc: plan(sc, 1.0)[0] is not None, 1.0)

        # Stability is soft. If the feet on the ground would leave the margin too small, the lift
        # is lowered so lifted feet come down and join the support. It falls fast and recovers
        # slowly, so the reaction itself never moves a foot by a jump.
        def stable(lift_scale):
            angles, _, margin = plan(scale, lift_scale)
            return angles is not None and margin >= self._gp.stability_threshold

        wanted = largest_value(stable, 1.0)
        if wanted < self._lift_scale:
            self._lift_scale = max(wanted, self._lift_scale - self._gp.scale_fall_per_s * dt)
        else:
            self._lift_scale = min(wanted, self._lift_scale + self._gp.scale_rise_per_s * dt)

        # The scales were found separately; the pair in use must itself be reachable. Lowering a
        # foot can push it out of reach (it ends up farther from the hip), so check, and fall back.
        angles, feet, margin = plan(scale, self._lift_scale)
        if angles is None:
            scale = largest_value(lambda sc: plan(sc, self._lift_scale)[0] is not None, scale)
            angles, feet, margin = plan(scale, self._lift_scale)
        if angles is None:
            self._lift_scale = 0.0
            scale = largest_value(lambda sc: plan(sc, 0.0)[0] is not None, 1.0)
            angles, feet, margin = plan(scale, 0.0)
        if angles is None:  # not reachable even standing: the applied pose is always reachable
            raise RuntimeError("walker invariant broken: the applied pose is not reachable")
        return WalkerOutput(
            angles=angles,
            feet=feet,
            phases=phases,
            in_stance={leg: phases[leg] < duty for leg in LEG_CODES},
            stability_margin=margin,
            scale=scale,
            speed_scale=self._speed_scale,
            lift_scale=self._lift_scale,
            gait=self._target.name,
            blending=self._progress < 1.0,
        )
