"""Gait definitions: a duty factor and a phase offset per leg (docs/07-locomotion.md, section 3)."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from bemu_params import LEG_CODES


@dataclass(frozen=True)
class Gait:
    name: str
    duty: float  # fraction of the cycle a leg spends on the ground
    offsets: Mapping[str, float]  # leg -> phase offset in [0, 1)


def _gait(name: str, duty: float, offsets: tuple[float, ...]) -> Gait:
    return Gait(name, duty, dict(zip(LEG_CODES, offsets, strict=True)))


# Offsets in the order LF, LM, LR, RF, RM, RR.
TRIPOD = _gait("tripod", 1 / 2, (0, 1 / 2, 0, 1 / 2, 0, 1 / 2))
RIPPLE = _gait("ripple", 2 / 3, (0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0))
WAVE = _gait("wave", 5 / 6, (1 / 2, 2 / 3, 5 / 6, 0, 1 / 6, 1 / 3))
GAITS: dict[str, Gait] = {g.name: g for g in (TRIPOD, RIPPLE, WAVE)}


def feet_down(gait: Gait, master_phase: float) -> int:
    """Legs on the ground at a master phase: a leg's phase is (master + offset) mod 1."""
    return sum(((master_phase + gait.offsets[leg]) % 1.0) < gait.duty for leg in LEG_CODES)
