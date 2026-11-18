"""Match joint positions to a leg by joint name. Imports nothing from ROS."""

from __future__ import annotations

from collections.abc import Sequence

from bemu_description.names import joint_names
from bemu_kinematics import JointAngles


def angles_by_name(names: Sequence[str], positions: Sequence[float], leg_code: str) -> JointAngles:
    """Pick the leg's three joints out of a /joint_states-style list, whatever its order."""
    if len(names) != len(positions):
        raise ValueError(f"{len(names)} names but {len(positions)} positions")
    by_name = dict(zip(names, positions, strict=True))
    wanted = joint_names(leg_code)
    for name in wanted:
        if name not in by_name:
            raise KeyError(f"joint {name} is not in the joint state")
    return JointAngles(*(float(by_name[n]) for n in wanted))
