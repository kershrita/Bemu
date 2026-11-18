"""Build the full 18-joint MuJoCo model (XML text) from the parameter file.

A box body on a free joint, six legs at their mounts, floor, 18 position actuators named as
their joints, and keyframes `sit` (body resting on the floor) and `stand`. Only the body box and
the foot spheres collide (with the floor); leg-to-leg collisions come with the gaits.
"""

from __future__ import annotations

from bemu_description.actuators import ActuatorModel
from bemu_description.mjcf import (
    TIMESTEP,
    joint_defaults_xml,
    leg_actuators_xml,
    leg_body_xml,
    num,
    vec,
)
from bemu_description.names import BODY, ROOT_JOINT, joint_order
from bemu_kinematics import BodyPose, pose_angles, sit_height, stand_height
from bemu_params import LEG_CODES, RobotParams


def _keyframe(params: RobotParams, name: str, height: float) -> str:
    angles = pose_angles(params, BodyPose(height))
    joints = [angle for leg in LEG_CODES for angle in angles[leg]]
    qpos = vec(0.0, 0.0, height, 1.0, 0.0, 0.0, 0.0, *joints)
    return f'<key name="{name}" qpos="{qpos}" ctrl="{vec(*joints)}"/>'


def build_robot_mjcf(params: RobotParams, actuators: ActuatorModel) -> str:
    sx, sy, sz = params.body.size
    legs = "\n    ".join(
        leg_body_xml(params, leg, pos=vec(m.x, m.y, 0.0), euler=vec(0.0, 0.0, m.yaw))
        for leg, m in params.mounts.items()
    )
    actuator_xml = "\n".join(leg_actuators_xml(params, actuators, leg) for leg in LEG_CODES)
    assert tuple(a.split('name="')[1].split('"')[0] for a in actuator_xml.splitlines()) == (
        joint_order()
    )
    return f"""<mujoco model="bemu_robot">
  <compiler angle="radian"/>
  <option timestep="{num(TIMESTEP)}"/>
  <default>
    {joint_defaults_xml(actuators)}
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" contype="1" conaffinity="1"/>
    <body name="{BODY}" pos="0 0 {num(stand_height(params))}">
      <freejoint name="{ROOT_JOINT}"/>
      <geom name="body_box" type="box" size="{vec(sx / 2, sy / 2, sz / 2)}" \
mass="{num(params.body.mass)}" contype="1" conaffinity="1"/>
      {legs}
    </body>
  </worldbody>
  <actuator>
{actuator_xml}
  </actuator>
  <keyframe>
    {_keyframe(params, "sit", sit_height(params))}
    {_keyframe(params, "stand", stand_height(params))}
  </keyframe>
</mujoco>
"""
