import math

import mujoco
import numpy as np
import pytest

from bemu_description.names import joint_order
from bemu_kinematics import (
    BodyPose,
    JointAngles,
    neutral_feet,
    pose_angles,
    sit_height,
    stand_height,
)
from bemu_params import LEG_CODES
from bemu_sim.robot import RobotSim, play


@pytest.fixture
def sim(robot_mjcf):
    return RobotSim(robot_mjcf)


def test_starts_sitting_at_rest(sim, params):
    assert sim.body_position()[2] == pytest.approx(sit_height(params), abs=1e-3)
    assert sim.tilt() < math.radians(0.5)
    assert sim.foot_contacts() == dict.fromkeys(LEG_CODES, True)


def test_reset_to_stand_puts_feet_on_the_neutral_points(sim, params):
    sim.reset("stand")
    assert sim.body_position()[2] == pytest.approx(stand_height(params), abs=1e-9)
    for leg, planted in neutral_feet(params).items():
        assert sim.foot_positions()[leg] == pytest.approx(planted, abs=1e-6), leg
    assert sim.joint_angles()["lm"] == pytest.approx(
        tuple(pose_angles(params, BodyPose(stand_height(params)))["lm"]), abs=1e-9
    )


@pytest.mark.parametrize(
    "euler", [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.0, -0.3, 0.1), (0.1, 0.2, -0.4)]
)
def test_body_euler_reads_back_a_known_orientation(sim, euler):
    roll, pitch, yaw = euler
    rx = np.array(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
    )
    ry = np.array(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]
    )
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    )
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (rz @ ry @ rx).flatten())
    sim.data.qpos[3:7] = quat
    mujoco.mj_forward(sim.model, sim.data)
    assert sim.body_euler() == pytest.approx(euler, abs=1e-9)


def test_command_advances_one_control_period(sim, params):
    before = sim.time
    sim.command(pose_angles(params, BodyPose(sit_height(params))))
    assert sim.time - before == pytest.approx(0.02, abs=1e-9)


def test_center_of_mass_is_near_the_body_centre(sim):
    com = sim.center_of_mass()
    assert abs(com[0]) < 0.03 and abs(com[1]) < 0.03


def snapshot(sim):
    return (sim.time, {leg: tuple(a) for leg, a in sim.joint_angles().items()})


def test_command_with_a_missing_leg_is_rejected_and_changes_nothing(sim, params):
    targets = pose_angles(params, BodyPose(sit_height(params)))
    del targets["rm"]
    before = snapshot(sim)
    with pytest.raises(ValueError, match="rm"):
        sim.command(targets)
    assert snapshot(sim) == before


def test_command_with_an_unknown_leg_is_rejected(sim, params):
    targets = pose_angles(params, BodyPose(sit_height(params)))
    targets["xx"] = JointAngles(0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="xx"):
        sim.command(targets)


def test_command_with_a_nan_angle_is_rejected_and_changes_nothing(sim, params):
    targets = pose_angles(params, BodyPose(sit_height(params)))
    targets["lf"] = JointAngles(float("nan"), 0.0, -1.0)
    before = snapshot(sim)
    with pytest.raises(ValueError, match="lf"):
        sim.command(targets)
    assert snapshot(sim) == before


def test_peak_torque_sees_the_spike_inside_a_control_period(sim, params):
    sim.reset("stand")
    sim.clear_peak()
    sim.command(pose_angles(params, BodyPose(stand_height(params) + 0.02)))
    sampled = max(float(np.max(np.abs(t))) for t in sim.joint_torques().values())
    assert sim.peak_torque > 1.2 * sampled


def test_clear_peak_resets_the_record(sim):
    sim.command(sim.joint_angles())
    assert sim.peak_torque > 0.0
    sim.clear_peak()
    assert sim.peak_torque == 0.0


def test_a_lifted_foot_does_not_read_as_touching(sim, params):
    sim.reset("stand")
    targets = dict(pose_angles(params, BodyPose(stand_height(params))))
    lm = targets["lm"]
    targets["lm"] = JointAngles(lm.coxa, lm.femur + 0.5, lm.tibia)
    play(sim, [targets] * 25)
    contacts = sim.foot_contacts()
    assert contacts["lm"] is False
    assert sum(contacts.values()) >= 4


def test_play_returns_the_peak_torque(sim, params):
    held = pose_angles(params, BodyPose(sit_height(params)))
    peak = play(sim, [held, held])
    assert isinstance(peak, float) and peak >= 0.0
    assert set(sim.joint_torques()) == set(LEG_CODES)
    assert len(joint_order()) == 18
