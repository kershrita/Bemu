"""Build the MuJoCo model of one leg hanging from a fixed stand (XML text; no mujoco import).

Leg frame: origin on the coxa axis, x out, y left, z up. The femur and tibia hinges point
along -y so that a positive angle raises the link.
"""

from __future__ import annotations

from xml.sax.saxutils import quoteattr

from bemu_description.actuators import ActuatorModel
from bemu_description.names import foot_site, hip_site, joint_names
from bemu_kinematics import inverse
from bemu_params import RobotParams

TIMESTEP = 0.002  # s
STAND_HEIGHT = 0.20  # m, hip above the floor
LINK_RADIUS = 0.012  # m
FOOT_RADIUS = 0.008  # m
FOOT_FRICTION = 1.0


def neutral_angles(params: RobotParams) -> tuple[float, float, float]:
    """Joint angles (rad) of the neutral stance, from the stance parameters."""
    foot = (params.stance.foot_radius, 0.0, -params.stance.body_height)
    return tuple(inverse(params.leg, foot))


def _num(value: float) -> str:
    return repr(float(value))


def _vec(*values: float) -> str:
    return " ".join(_num(v) for v in values)


def build_leg_mjcf(params: RobotParams, actuators: ActuatorModel, leg_code: str = "lf") -> str:
    leg = params.leg
    coxa, femur, tibia = joint_names(leg_code)
    limits = (leg.coxa_limits, leg.femur_limits, leg.tibia_limits)
    neutral = neutral_angles(params)
    q = quoteattr

    def joint(name: str, axis: tuple[float, float, float], rng: tuple[float, float]) -> str:
        return (
            f'<joint name={q(name)} type="hinge" axis="{_vec(*axis)}" limited="true" '
            f'range="{_vec(*rng)}"/>'
        )

    actuator_lines = "\n".join(
        f'    <position name={q(name)} joint={q(name)} kp="{_num(actuators.kp)}" '
        f'kv="{_num(actuators.kv)}" ctrllimited="true" ctrlrange="{_vec(*rng)}" '
        f'forcelimited="true" forcerange="{_vec(-actuators.max_torque, actuators.max_torque)}"/>'
        for name, rng in zip((coxa, femur, tibia), limits, strict=True)
    )
    qpos = _vec(*neutral)
    return f"""<mujoco model="bemu_{leg_code}_leg">
  <compiler angle="radian"/>
  <option timestep="{_num(TIMESTEP)}"/>
  <default>
    <joint armature="{_num(actuators.armature)}" damping="{_num(actuators.damping)}" \
frictionloss="{_num(actuators.frictionloss)}"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="{leg_code}_coxa" pos="0 0 {_num(STAND_HEIGHT)}">
      {joint(coxa, (0, 0, 1), limits[0])}
      <site name={q(hip_site(leg_code))} pos="0 0 0" size="0.004"/>
      <geom type="capsule" fromto="0 0 0 {_vec(leg.coxa_length, 0, 0)}" \
size="{_num(LINK_RADIUS)}" mass="{_num(leg.coxa_mass)}"/>
      <body name="{leg_code}_femur" pos="{_vec(leg.coxa_length, 0, 0)}">
        {joint(femur, (0, -1, 0), limits[1])}
        <geom type="capsule" fromto="0 0 0 {_vec(leg.femur_length, 0, 0)}" \
size="{_num(LINK_RADIUS)}" mass="{_num(leg.femur_mass)}"/>
        <body name="{leg_code}_tibia" pos="{_vec(leg.femur_length, 0, 0)}">
          {joint(tibia, (0, -1, 0), limits[2])}
          <geom type="capsule" fromto="0 0 0 {_vec(leg.tibia_length, 0, 0)}" \
size="{_num(LINK_RADIUS)}" mass="{_num(leg.tibia_mass)}"/>
          <geom name="{leg_code}_foot_tip" type="sphere" pos="{_vec(leg.tibia_length, 0, 0)}" \
size="{_num(FOOT_RADIUS)}" density="0" friction="{_num(FOOT_FRICTION)} 0.005 0.0001"/>
          <site name={q(foot_site(leg_code))} pos="{_vec(leg.tibia_length, 0, 0)}" size="0.004"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
{actuator_lines}
  </actuator>
  <keyframe>
    <key name="neutral" qpos="{qpos}" ctrl="{qpos}"/>
  </keyframe>
</mujoco>
"""
