"""Measure how a posed robot is doing: height, tilt, foot slip, torque, stability."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from bemu_kinematics import stability_margin
from bemu_sim.robot import RobotSim


@dataclass(frozen=True)
class PoseReport:
    height: float  # m, hip plane above the floor
    tilt: float  # rad
    roll: float  # rad
    pitch: float
    yaw: float
    planted_feet: int  # feet touching the floor
    max_foot_slip: float  # m, largest horizontal distance of any foot from its planted point
    peak_torque: float  # N·m
    stability_margin: float  # m


def measure(sim: RobotSim, planted: Mapping[str, ArrayLike], peak_torque: float) -> PoseReport:
    """Read the robot's state; `planted` are the ground points the feet should still be on."""
    feet = sim.foot_positions()
    contacts = sim.foot_contacts()
    slip = max(
        float(np.linalg.norm(feet[leg][:2] - np.asarray(planted[leg], dtype=float)[:2]))
        for leg in feet
    )
    touching = [feet[leg] for leg, down in contacts.items() if down]
    roll, pitch, yaw = sim.body_euler()
    return PoseReport(
        height=float(sim.body_position()[2]),
        tilt=sim.tilt(),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        planted_feet=len(touching),
        max_foot_slip=slip,
        peak_torque=float(peak_torque),
        stability_margin=stability_margin(sim.center_of_mass(), np.array(touching).reshape(-1, 3)),
    )
