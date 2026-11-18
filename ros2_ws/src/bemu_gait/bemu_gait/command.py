"""The velocity command the walker follows."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    vx: float = 0.0  # m/s forward
    vy: float = 0.0  # m/s to the left
    omega: float = 0.0  # rad/s counter-clockwise seen from above
