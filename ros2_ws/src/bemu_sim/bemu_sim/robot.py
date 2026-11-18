"""The full 18-joint robot in MuJoCo, headless (no ROS). Starts at the "sit" keyframe."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

import mujoco
import numpy as np

from bemu_description.names import BODY, foot_site, joint_names, joint_order
from bemu_kinematics import JointAngles
from bemu_params import LEG_CODES
from bemu_sim.headless import CONTROL_RATE

CONTACT_TOLERANCE = 1e-4  # m: a foot this close to the floor counts as touching


class RobotSim:
    def __init__(self, mjcf: str) -> None:
        self._model = mujoco.MjModel.from_xml_string(mjcf)
        self._data = mujoco.MjData(self._model)
        self._steps_per_control = round(1.0 / (CONTROL_RATE * self._model.opt.timestep))
        self._body = self._model.body(BODY).id
        self._floor = self._model.geom("floor").id
        self._foot_geom = {leg: self._model.geom(f"{leg}_foot_tip").id for leg in LEG_CODES}
        self._foot_site = {leg: self._model.site(foot_site(leg)).id for leg in LEG_CODES}
        order = {name: i for i, name in enumerate(joint_order())}
        self._actuator = {leg: [order[n] for n in joint_names(leg)] for leg in LEG_CODES}
        self._qpos = {
            leg: [self._model.joint(n).qposadr[0] for n in joint_names(leg)] for leg in LEG_CODES
        }
        self._peak = 0.0
        self.reset("sit")

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def time(self) -> float:
        return float(self._data.time)

    @property
    def peak_torque(self) -> float:
        """Largest absolute actuator torque (N·m) seen on any physics step since clear_peak()."""
        return self._peak

    def clear_peak(self) -> None:
        self._peak = 0.0

    def reset(self, keyframe: str = "sit") -> None:
        key = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
        if key < 0:
            raise ValueError(f"unknown keyframe {keyframe!r}")
        mujoco.mj_resetDataKeyframe(self._model, self._data, key)
        mujoco.mj_forward(self._model, self._data)

    def command(self, targets: Mapping[str, JointAngles]) -> None:
        """Set the joint targets (rad) for all six legs and advance one control period."""
        missing = [leg for leg in LEG_CODES if leg not in targets]
        if missing:
            raise ValueError(f"targets is missing the legs: {', '.join(missing)}")
        extra = [leg for leg in targets if leg not in LEG_CODES]
        if extra:
            raise ValueError(f"targets has unknown legs: {', '.join(extra)}")
        values = {}
        for leg in LEG_CODES:
            angles = np.asarray(tuple(targets[leg]), dtype=float)
            if angles.shape != (3,) or not np.all(np.isfinite(angles)):
                raise ValueError(f"targets for leg {leg} must be three finite angles")
            values[leg] = angles
        for leg, angles in values.items():
            self._data.ctrl[self._actuator[leg]] = angles
        for _ in range(self._steps_per_control):
            mujoco.mj_step(self._model, self._data)
            # A position actuator spikes on the first physics steps after a new target and has
            # settled by the end of the control period, so look at every step, not just the last.
            self._peak = max(self._peak, float(np.max(np.abs(self._data.actuator_force))))

    def _rotation(self) -> np.ndarray:
        return self._data.xmat[self._body].reshape(3, 3)

    def body_position(self) -> np.ndarray:
        """Hip-plane centre of the body, ground frame (3,), m."""
        return self._data.xpos[self._body].copy()

    def body_euler(self) -> tuple[float, float, float]:
        """Roll, pitch, yaw (rad) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
        r = self._rotation()
        pitch = -math.asin(max(-1.0, min(1.0, r[2, 0])))
        return (math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0]))

    def tilt(self) -> float:
        """Angle (rad) between the body's up axis and vertical."""
        return math.acos(max(-1.0, min(1.0, self._rotation()[2, 2])))

    def joint_angles(self) -> dict[str, JointAngles]:
        return {
            leg: JointAngles(*(float(self._data.qpos[i]) for i in self._qpos[leg]))
            for leg in LEG_CODES
        }

    def joint_torques(self) -> dict[str, np.ndarray]:
        """Actuator torques per leg, (3,) N·m."""
        return {leg: self._data.actuator_force[self._actuator[leg]].copy() for leg in LEG_CODES}

    def foot_positions(self) -> dict[str, np.ndarray]:
        """Foot sphere centres in the ground frame, (3,) m."""
        return {leg: self._data.site_xpos[self._foot_site[leg]].copy() for leg in LEG_CODES}

    def foot_contacts(self) -> dict[str, bool]:
        """Whether each foot is touching the floor: within CONTACT_TOLERANCE of it.

        A distance test, not MuJoCo's contact list: a foot resting exactly on the floor sits at
        +1e-17 m, which the solver does not list as a contact.
        """
        return {
            leg: bool(
                mujoco.mj_geomDistance(
                    self._model, self._data, self._foot_geom[leg], self._floor, 0.01, None
                )
                <= CONTACT_TOLERANCE
            )
            for leg in LEG_CODES
        }

    def center_of_mass(self) -> np.ndarray:
        return self._data.subtree_com[self._body].copy()


def play(sim: RobotSim, targets: Iterable[Mapping[str, JointAngles]]) -> float:
    """Send each target set in turn; return the largest absolute joint torque (N·m) on any step."""
    sim.clear_peak()
    for step in targets:
        sim.command(step)
    return sim.peak_torque
