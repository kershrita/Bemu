import dataclasses

import mujoco
import numpy as np
import pytest

from bemu_description.names import joint_order
from bemu_description.robot_mjcf import build_robot_mjcf
from bemu_kinematics import BodyPose, pose_angles, sit_height, stand_height
from bemu_params import LEG_CODES
from bemu_sim.robot import RobotSim


def targets(params):
    return pose_angles(params, BodyPose(stand_height(params) + 0.005))


def collect_by_hand(mjcf, params, ticks):
    """Step an identical simulation ourselves and record every physics step."""
    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(
        model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "sit")
    )
    ctrl = [angle for leg in LEG_CODES for angle in targets(params)[leg]]
    dofs = [model.joint(name).dofadr[0] for name in joint_order()]
    torques, speeds = [], []
    for _ in range(ticks):
        data.ctrl[:] = ctrl
        for _ in range(10):
            mujoco.mj_step(model, data)
            torques.append(np.array(data.actuator_force))
            speeds.append(np.array([data.qvel[d] for d in dofs]))
    return np.array(torques), np.array(speeds)


def test_statistics_match_a_hand_computation(robot_mjcf, params):
    sim = RobotSim(robot_mjcf)
    sim.clear_stats()
    for _ in range(3):
        sim.command(targets(params))
    torque, speed = collect_by_hand(robot_mjcf, params, 3)
    stats = sim.stats()
    assert stats.joints == joint_order()
    assert stats.steps == 30
    assert stats.peak_torque == pytest.approx(np.abs(torque).max(axis=0), abs=1e-12)
    assert stats.rms_torque == pytest.approx(np.sqrt((torque**2).mean(axis=0)), abs=1e-12)
    assert stats.mean_abs_torque == pytest.approx(np.abs(torque).mean(axis=0), abs=1e-12)
    assert stats.peak_speed == pytest.approx(np.abs(speed).max(axis=0), abs=1e-12)
    assert stats.peak_torque.shape == (18,)


def test_overall_peak_is_the_largest_joint_peak(robot_mjcf, params):
    sim = RobotSim(robot_mjcf)
    sim.clear_stats()
    sim.command(targets(params))
    assert sim.peak_torque == pytest.approx(sim.stats().peak_torque.max(), abs=1e-12)
    assert sim.peak_torque > 0.0


def test_clear_resets_everything(robot_mjcf, params):
    sim = RobotSim(robot_mjcf)
    sim.command(targets(params))
    assert sim.stats().steps > 0
    sim.clear_stats()
    stats = sim.stats()
    assert stats.steps == 0
    for array in (stats.peak_torque, stats.rms_torque, stats.mean_abs_torque, stats.peak_speed):
        assert array.shape == (18,) and not array.any()
    assert not stats.steps_at_limit.any()
    assert sim.peak_torque == 0.0


def test_clear_peak_is_the_same_as_clear_stats(robot_mjcf, params):
    sim = RobotSim(robot_mjcf)
    sim.command(targets(params))
    sim.clear_peak()
    assert sim.stats().steps == 0 and sim.peak_torque == 0.0


def test_steps_at_limit_counts_saturation(params, actuators):
    weak = dataclasses.replace(actuators, max_torque=0.05)
    sim = RobotSim(build_robot_mjcf(params, weak))
    sim.clear_stats()
    for _ in range(3):
        sim.command(targets(params))
    stats = sim.stats()
    assert stats.steps_at_limit.sum() > 0
    assert stats.steps_at_limit.max() <= stats.steps
    assert stats.peak_torque.max() == pytest.approx(0.05)

    strong = dataclasses.replace(actuators, max_torque=2.94)
    sim = RobotSim(build_robot_mjcf(params, strong))
    sim.clear_stats()
    for _ in range(3):  # holding the pose it starts in: no demand near the stall torque
        sim.command(pose_angles(params, BodyPose(sit_height(params))))
    assert not sim.stats().steps_at_limit.any()
