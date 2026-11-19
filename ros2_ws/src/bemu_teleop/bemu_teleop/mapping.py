"""Keys and sticks to velocity, gait and pose requests. No ROS in here."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from bemu_gait import GAITS, GaitParams
from bemu_kinematics import neutral_feet, stand_height
from bemu_params import RobotParams

KEY_STEP = 0.2  # of the top speed, per key press
HEIGHT_STEP = 0.005  # m
HEIGHT_RANGE = 0.010  # m, either side of the standing height
AXIS_LIMIT = 1.0001  # a joystick axis is in [-1, 1]


def top_speeds(
    params: RobotParams, gait_params: GaitParams, gait: str
) -> tuple[float, float, float]:
    """The gait's fastest forward and sideways speed (m/s) and turn rate (rad/s)."""
    if gait not in GAITS:
        raise ValueError(f"unknown gait {gait!r}; valid gaits: {', '.join(GAITS)}")
    straight = gait_params.max_stride / (GAITS[gait].duty * gait_params.cycle_time[gait])
    farthest = max(math.hypot(*foot[:2]) for foot in neutral_feet(params).values())
    return straight, straight, straight / farthest


@dataclass(frozen=True)
class Action:
    """What a key or button asks for besides motion."""

    kind: str  # "stand", "sit", "gait", "height" or "quit"
    gait: str = ""
    height: float = 0.0  # the new absolute hip height, metres


class KeyboardMapper:
    """Latched keyboard control: keys step the commanded speed; a terminal has no key release."""

    GAIT_KEYS = {"1": "tripod", "2": "ripple", "3": "wave"}

    def __init__(self, params: RobotParams, gait_params: GaitParams, gait: str = "tripod") -> None:
        self._params = params
        self._gait_params = gait_params
        self._base_height = stand_height(params)
        self._offset = 0.0
        self.gait = gait
        self.velocity = (0.0, 0.0, 0.0)
        self._top = top_speeds(params, gait_params, gait)

    @property
    def height(self) -> float:
        return self._base_height + self._offset

    def _bump(self, axis: int, direction: int) -> None:
        values = list(self.velocity)
        top = self._top[axis]
        values[axis] = max(-top, min(top, values[axis] + direction * KEY_STEP * top))
        self.velocity = tuple(values)

    def press(self, key: str) -> Action | None:
        motion = {
            "w": (0, 1),
            "s": (0, -1),
            "a": (1, 1),
            "d": (1, -1),
            "q": (2, 1),
            "e": (2, -1),
        }
        if key in motion:
            self._bump(*motion[key])
        elif key == " ":
            self.velocity = (0.0, 0.0, 0.0)
        elif key in self.GAIT_KEYS:
            self.gait = self.GAIT_KEYS[key]
            self._top = top_speeds(self._params, self._gait_params, self.gait)
            self.velocity = tuple(
                max(-top, min(top, v)) for v, top in zip(self.velocity, self._top, strict=True)
            )
            return Action("gait", gait=self.gait)
        elif key in ("r", "f"):
            step = HEIGHT_STEP if key == "r" else -HEIGHT_STEP
            self._offset = max(-HEIGHT_RANGE, min(HEIGHT_RANGE, self._offset + step))
            return Action("height", height=self.height)
        elif key == "h":
            return Action("stand")
        elif key == "j":
            return Action("sit")
        elif key == "x":
            return Action("quit")
        return None


@dataclass(frozen=True)
class JoyConfig:
    """Indices and tuning for the gamepad; the defaults suit an Xbox pad under Linux."""

    axis_x: int = 0
    axis_y: int = 1
    axis_turn: int = 3
    button_stand: int = 0
    button_sit: int = 1
    button_tripod: int = 2
    button_ripple: int = 3
    button_wave: int = 5
    button_slow: int = 4
    deadzone: float = 0.1
    slow_scale: float = 0.3


def _button(buttons: Sequence[int], index: int) -> bool:
    return 0 <= index < len(buttons) and bool(buttons[index])


def _stick(value: float, deadzone: float) -> float:
    if not math.isfinite(value) or abs(value) > AXIS_LIMIT:
        raise ValueError(f"joystick axis value out of range: {value!r}")
    if abs(value) <= deadzone:
        return 0.0
    rescaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(min(1.0, rescaled), value)


def joy_command(
    axes: Sequence[float],
    buttons: Sequence[int],
    config: JoyConfig,
    top: tuple[float, float, float],
) -> tuple[float, float, float]:
    """vx, vy, omega from the sticks, scaled to `top`; raises ValueError for a bad message."""
    needed = max(config.axis_x, config.axis_y, config.axis_turn) + 1
    if len(axes) < needed:
        raise ValueError(f"joystick message has {len(axes)} axes, need at least {needed}")
    scale = config.slow_scale if _button(buttons, config.button_slow) else 1.0
    forward = _stick(axes[config.axis_y], config.deadzone)
    left = _stick(axes[config.axis_x], config.deadzone)
    turn = _stick(axes[config.axis_turn], config.deadzone)
    return (forward * top[0] * scale, left * top[1] * scale, turn * top[2] * scale)


def joy_actions(buttons: Sequence[int], previous: Sequence[int], config: JoyConfig) -> list[Action]:
    """Actions for buttons that went down since the last message."""
    wanted = (
        (config.button_stand, Action("stand")),
        (config.button_sit, Action("sit")),
        (config.button_tripod, Action("gait", gait="tripod")),
        (config.button_ripple, Action("gait", gait="ripple")),
        (config.button_wave, Action("gait", gait="wave")),
    )
    return [
        action
        for index, action in wanted
        if _button(buttons, index) and not _button(previous, index)
    ]
