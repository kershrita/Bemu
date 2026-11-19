"""The walker's behaviour, with no ROS in it: stand and sit sequences, the command timeout,
joint matching by name, and the 1b-1 Walker. The node only moves messages in and out."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from bemu_description.names import joint_order
from bemu_gait import Command, GaitParams, Walker, WalkerOutput
from bemu_gait.gaits import GAITS
from bemu_kinematics import (
    BodyPose,
    neutral_feet,
    pose_angles,
    sit_down_targets,
    sit_height,
    stand_height,
    stand_up_targets,
)
from bemu_params import LEG_CODES, RobotParams

SIT_POSE_TOLERANCE = 0.3  # rad: how far from the sit pose a stand-up may still start
MIN_STOP_TIME = 1.5  # s: a sit request waits at least this long (the body pose slews back)
MAX_STOP_TIME = 10.0  # s: after this the sit request is cancelled
SETTLED_TICKS = 10  # consecutive ticks with every foot at its neutral position
SETTLED_DISTANCE = 0.003  # m


class Mode(StrEnum):
    SITTING = "sitting"
    STANDING_UP = "standing_up"
    STANDING = "standing"
    STOPPING = "stopping"
    SITTING_DOWN = "sitting_down"


@dataclass(frozen=True)
class Tick:
    command: tuple[float, ...]  # 18 joint targets in joint_order()
    mode: Mode
    gait: str
    phase: tuple[float, ...]  # 6, in LEG_CODES order (0.0 when not walking)
    foot_down: tuple[bool, ...]  # 6 (all True when not walking)
    stability_margin: float  # m (0.0 when not walking)


def _flat(angles) -> tuple[float, ...]:
    return tuple(float(a) for leg in LEG_CODES for a in angles[leg])


class LocomotionCore:
    def __init__(
        self,
        params: RobotParams,
        gait_params: GaitParams,
        *,
        auto_stand: bool = True,
        cmd_timeout: float = 0.5,
        rate: float = 50.0,
    ) -> None:
        if not (
            math.isfinite(rate) and rate > 0 and math.isfinite(cmd_timeout) and cmd_timeout > 0
        ):
            raise ValueError(
                f"rate and cmd_timeout must be positive, got {rate!r}, {cmd_timeout!r}"
            )
        self._params = params
        self._gait_params = gait_params
        self._dt = 1.0 / rate
        self._auto_stand = auto_stand
        self._cmd_timeout = cmd_timeout
        self._stand_up = [_flat(t) for t in stand_up_targets(params, rate=rate)]
        self._sit_down = [_flat(t) for t in sit_down_targets(params, rate=rate)]
        self._sit_pose = _flat(pose_angles(params, BodyPose(sit_height(params))))
        self._neutral = neutral_feet(params)
        self._state: tuple[float, ...] | None = None
        self._mode = Mode.SITTING
        self._gait = "tripod"
        self._walker: Walker | None = None
        self._sequence: list[tuple[float, ...]] = []
        self._index = 0
        self._velocity: tuple[float, float, float] | None = None
        self._velocity_time = 0.0
        self._stop_ticks = 0
        self._settled_ticks = 0
        self._auto_stand_tried = False
        self._notices: list[str] = []

    @property
    def mode(self) -> Mode:
        return self._mode

    def pop_notices(self) -> list[str]:
        """Messages for the node to log (a refused auto-stand, a cancelled sit)."""
        notices, self._notices = self._notices, []
        return notices

    # -- inputs ---------------------------------------------------------------------------------

    def on_joint_state(self, names: Sequence[str], positions: Sequence[float]) -> bool:
        """Remember the joint angles by name. Returns False, storing nothing, if a joint is
        missing or a value is not a finite number; joints that are not ours are ignored."""
        if len(names) != len(positions):
            return False
        by_name = dict(zip(names, positions, strict=True))
        try:
            values = tuple(float(by_name[name]) for name in joint_order())
        except (KeyError, TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in values):
            return False
        self._state = values
        return True

    def set_velocity(self, vx: float, vy: float, omega: float, now: float) -> None:
        values = (vx, vy, omega)
        if not all(isinstance(v, int | float) and math.isfinite(v) for v in values):
            raise ValueError(f"velocity must be three finite numbers, got {values}")
        if self._mode == Mode.STANDING:
            self._velocity = (float(vx), float(vy), float(omega))
            self._velocity_time = now

    def set_body_pose(self, pose: BodyPose) -> None:
        pose_angles(self._params, pose)  # raises ValueError if it cannot be reached
        if self._mode == Mode.STANDING and self._walker is not None:
            self._walker.set_body_pose(pose)

    def set_gait(self, name: str) -> None:
        if name not in GAITS:
            raise ValueError(f"unknown gait {name!r}; valid gaits: {', '.join(GAITS)}")
        self._gait = name
        if self._walker is not None:
            self._walker.set_gait(name)

    def request_stand(self) -> tuple[bool, str]:
        if self._mode != Mode.SITTING:
            return False, f"cannot stand up: the robot is {self._mode.value}"
        if self._state is None:
            return False, "no joint state received yet"
        worst = max(abs(a - b) for a, b in zip(self._state, self._sit_pose, strict=True))
        if worst > SIT_POSE_TOLERANCE:
            return False, "legs are not in the sit pose"
        self._sequence, self._index, self._mode = self._stand_up, 0, Mode.STANDING_UP
        return True, "standing up"

    def request_sit(self) -> tuple[bool, str]:
        if self._mode != Mode.STANDING:
            return False, f"cannot sit down: the robot is {self._mode.value}"
        self._velocity = None
        assert self._walker is not None
        self._walker.set_command(Command())
        self._walker.set_body_pose(BodyPose(stand_height(self._params)))
        self._stop_ticks = self._settled_ticks = 0
        self._mode = Mode.STOPPING
        return True, "stopping, then sitting down"

    # -- the 50 Hz step ---------------------------------------------------------------------------

    def tick(self, now: float) -> Tick | None:
        if self._state is None:
            return None
        if self._mode == Mode.SITTING:
            if self._auto_stand and not self._auto_stand_tried:
                self._auto_stand_tried = True
                ok, message = self.request_stand()
                if not ok:
                    self._notices.append(f"auto stand skipped: {message}")
            if self._mode == Mode.SITTING:
                return self._idle(self._sit_pose)
        if self._mode in (Mode.STANDING_UP, Mode.SITTING_DOWN):
            return self._play()
        return self._walk(now)

    def _idle(self, command: tuple[float, ...]) -> Tick:
        return Tick(command, self._mode, self._gait, (0.0,) * 6, (True,) * 6, 0.0)

    def _play(self) -> Tick:
        mode = self._mode
        command = self._sequence[self._index]
        self._index += 1
        if self._index >= len(self._sequence):
            if mode == Mode.STANDING_UP:
                self._walker = Walker(self._params, self._gait_params, gait=self._gait)
                self._velocity = None
                self._mode = Mode.STANDING
            else:
                self._walker = None
                self._mode = Mode.SITTING
        return Tick(command, mode, self._gait, (0.0,) * 6, (True,) * 6, 0.0)

    def _walk(self, now: float) -> Tick:
        assert self._walker is not None
        mode = self._mode
        stale = (
            self._velocity is None
            or now < self._velocity_time  # the clock restarted: do not trust the old stamp
            or now - self._velocity_time > self._cmd_timeout
        )
        velocity = (0.0, 0.0, 0.0) if mode == Mode.STOPPING or stale else self._velocity
        self._walker.set_command(Command(*velocity))
        out = self._walker.step(self._dt)
        if mode == Mode.STOPPING:
            self._advance_stop(out)
        return Tick(
            _flat(out.angles),
            mode,
            out.gait,
            tuple(float(out.phases[leg]) for leg in LEG_CODES),
            tuple(bool(out.in_stance[leg]) for leg in LEG_CODES),
            float(out.stability_margin),
        )

    def _settled(self, out: WalkerOutput) -> bool:
        # in_stance follows the gait phase even at zero stride, so look at where the feet are
        return all(
            float(np.linalg.norm(out.feet[leg] - self._neutral[leg])) < SETTLED_DISTANCE
            for leg in LEG_CODES
        )

    def _advance_stop(self, out: WalkerOutput) -> None:
        self._stop_ticks += 1
        self._settled_ticks = self._settled_ticks + 1 if self._settled(out) else 0
        elapsed = self._stop_ticks * self._dt
        if elapsed >= MIN_STOP_TIME and self._settled_ticks >= SETTLED_TICKS:
            self._sequence, self._index, self._mode = self._sit_down, 0, Mode.SITTING_DOWN
        elif elapsed >= MAX_STOP_TIME:
            self._mode = Mode.STANDING
            self._velocity = None
            self._notices.append("sit cancelled: the feet did not settle in time")
