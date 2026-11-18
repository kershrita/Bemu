"""Headless MuJoCo simulation of one leg (ROS-free).

Leg frame: origin on the coxa axis, x out, y left, z up. SI units throughout.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from bemu_description.names import foot_site, hip_site, joint_names
from bemu_kinematics import JointAngles, JointLimitError, UnreachableError, inverse
from bemu_params import LegParams

CONTROL_RATE = 50.0  # Hz


@dataclass(frozen=True)
class TrackingResult:
    max_error: float  # m
    rms_error: float  # m
    target: np.ndarray  # (n, 3), leg frame
    actual: np.ndarray  # (n, 3), leg frame
    peak_torque: np.ndarray  # (3,), N·m, absolute, while following the path


class LegSim:
    """One leg in MuJoCo, starting at the "neutral" keyframe."""

    def __init__(self, mjcf: str, leg_code: str = "lf") -> None:
        self._model = mujoco.MjModel.from_xml_string(mjcf)
        self._data = mujoco.MjData(self._model)
        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
        mujoco.mj_forward(self._model, self._data)
        self._steps_per_control = round(1.0 / (CONTROL_RATE * self._model.opt.timestep))
        joints = [self._model.joint(name) for name in joint_names(leg_code)]
        self._qpos = [j.qposadr[0] for j in joints]
        self._actuators = [self._model.actuator(name).id for name in joint_names(leg_code)]
        self._foot = self._model.site(foot_site(leg_code)).id
        self._hip = self._model.site(hip_site(leg_code)).id

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    def command(self, angles: JointAngles) -> None:
        """Set the joint targets (rad) and advance one control period."""
        for actuator, angle in zip(self._actuators, angles, strict=True):
            self._data.ctrl[actuator] = angle
        for _ in range(self._steps_per_control):
            mujoco.mj_step(self._model, self._data)

    def foot_position(self) -> np.ndarray:
        """Foot (3,), m, relative to the hip, in the leg frame."""
        return self._data.site_xpos[self._foot] - self._data.site_xpos[self._hip]

    def joint_angles(self) -> JointAngles:
        return JointAngles(*(float(self._data.qpos[i]) for i in self._qpos))

    def joint_torques(self) -> np.ndarray:
        """Actuator torques (3,), N·m."""
        return np.array([self._data.actuator_force[i] for i in self._actuators])


def _solve_path(leg: LegParams, path: np.ndarray) -> list[JointAngles]:
    solved = []
    for i, point in enumerate(path):
        try:
            solved.append(inverse(leg, point))
        except UnreachableError as exc:
            raise UnreachableError(f"path point {i}: {exc}", exc.reason) from exc
        except JointLimitError as exc:
            raise JointLimitError(f"path point {i}: {exc}", exc.joint) from exc
    return solved


def track_path(
    sim: LegSim, leg: LegParams, path: np.ndarray, *, settle_time: float = 1.0
) -> TrackingResult:
    """Move to the first point over `settle_time`, then follow `path`, one point per period.

    Every point is converted with `inverse` before the simulation moves, so an unreachable
    point raises (naming its index) and leaves the simulation untouched.
    """
    path = np.asarray(path, dtype=float)
    if path.size == 0:
        raise ValueError("path is empty")
    if path.ndim != 2 or path.shape[1] != 3:
        raise ValueError(f"path must have shape (n, 3), got {path.shape}")
    if not math.isfinite(settle_time) or settle_time < 0:
        raise ValueError(f"settle_time must be finite and not negative, got {settle_time!r}")
    targets = _solve_path(leg, path)

    start = np.array(sim.joint_angles())
    goal = np.array(targets[0])
    settle_steps = max(1, round(settle_time * CONTROL_RATE))
    for k in range(1, settle_steps + 1):
        sim.command(JointAngles(*(start + (goal - start) * k / settle_steps)))

    actual = np.empty_like(path)
    peak = np.zeros(3)
    for i, angles in enumerate(targets):
        sim.command(angles)
        actual[i] = sim.foot_position()
        peak = np.maximum(peak, np.abs(sim.joint_torques()))

    errors = np.linalg.norm(actual - path, axis=1)
    return TrackingResult(
        max_error=float(errors.max()),
        rms_error=float(np.sqrt(np.mean(errors**2))),
        target=path,
        actual=actual,
        peak_torque=peak,
    )
