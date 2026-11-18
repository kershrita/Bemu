import math

import pytest

from bemu_gait import Command, Walker
from bemu_kinematics import BodyPose, stand_height
from bemu_sim.robot import RobotSim
from bemu_sim.walking import max_stance_slip, stand_robot, walk

DEG = math.pi / 180.0
START, END = 8.0, 24.0  # the pose is commanded at START; measured over the last seconds before END


def poses(h):
    return {
        "height+10mm": BodyPose(h + 0.010),
        "height-10mm": BodyPose(h - 0.010),
        "roll+5": BodyPose(h, roll=5 * DEG),
        "roll-5": BodyPose(h, roll=-5 * DEG),
        "pitch+5": BodyPose(h, pitch=5 * DEG),
        "pitch-5": BodyPose(h, pitch=-5 * DEG),
        "combined": BodyPose(h + 0.005, roll=4 * DEG, pitch=-4 * DEG),
    }


@pytest.mark.parametrize("name", list(poses(0.098)))
def test_body_pose_changes_while_walking_do_not_slip_feet(
    robot_mjcf, params, gait_params, actuators, name
):
    sim = RobotSim(robot_mjcf)
    stand_robot(sim, params)
    walker = Walker(params, gait_params)
    pose = poses(stand_height(params))[name]
    before = float(sim.body_position()[2])

    def schedule(state, walker):
        walker.set_command(Command(vx=0.08))
        if state.t >= START:
            walker.set_body_pose(pose)

    log = walk(sim, walker, END, schedule=schedule)
    last = log.t >= END - 3.0
    assert log.position[last, 2].mean() - before == pytest.approx(
        pose.height - stand_height(params), abs=0.003
    )
    expected_tilt = math.acos(math.cos(pose.roll) * math.cos(pose.pitch))
    assert log.tilt[last].mean() == pytest.approx(expected_tilt, abs=1.0 * DEG)
    assert max_stance_slip(log) < 0.005
    assert log.margin.min() > 0.015
    assert log.peak_torque < actuators.max_torque
