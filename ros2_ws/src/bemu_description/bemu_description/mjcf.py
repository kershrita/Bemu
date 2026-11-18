"""MuJoCo model text for one leg on a stand, and the leg chain the full-robot model reuses.

No mujoco import: these build XML text.

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
FOOT_FRICTION = 1.0


def neutral_angles(params: RobotParams) -> tuple[float, float, float]:
    """Joint angles (rad) of the neutral stance, from the stance parameters."""
    foot = (params.stance.foot_radius, 0.0, -params.stance.body_height)
    return tuple(inverse(params.leg, foot))


def num(value: float) -> str:
    return repr(float(value))


def vec(*values: float) -> str:
    return " ".join(num(v) for v in values)


def joint_defaults_xml(actuators: ActuatorModel) -> str:
    return (
        f'<joint armature="{num(actuators.armature)}" damping="{num(actuators.damping)}" '
        f'frictionloss="{num(actuators.frictionloss)}"/>'
    )


def leg_body_xml(params: RobotParams, leg_code: str, *, pos: str, euler: str | None = None) -> str:
    """The nested coxa, femur and tibia bodies of one leg, with its hip and foot sites.

    `pos` (and `euler`) place the coxa body in its parent. The foot sphere's centre is the tibia
    end; it collides explicitly so a model whose default geoms do not collide still has feet.
    """
    leg = params.leg
    coxa, femur, tibia = joint_names(leg_code)
    limits = (leg.coxa_limits, leg.femur_limits, leg.tibia_limits)
    q = quoteattr

    def joint(name: str, axis: tuple[float, float, float], rng: tuple[float, float]) -> str:
        return (
            f'<joint name={q(name)} type="hinge" axis="{vec(*axis)}" limited="true" '
            f'range="{vec(*rng)}"/>'
        )

    orientation = f' euler="{euler}"' if euler else ""
    return f"""<body name="{leg_code}_coxa" pos="{pos}"{orientation}>
      {joint(coxa, (0, 0, 1), limits[0])}
      <site name={q(hip_site(leg_code))} pos="0 0 0" size="0.004"/>
      <geom type="capsule" fromto="0 0 0 {vec(leg.coxa_length, 0, 0)}" \
size="{num(LINK_RADIUS)}" mass="{num(leg.coxa_mass)}"/>
      <body name="{leg_code}_femur" pos="{vec(leg.coxa_length, 0, 0)}">
        {joint(femur, (0, -1, 0), limits[1])}
        <geom type="capsule" fromto="0 0 0 {vec(leg.femur_length, 0, 0)}" \
size="{num(LINK_RADIUS)}" mass="{num(leg.femur_mass)}"/>
        <body name="{leg_code}_tibia" pos="{vec(leg.femur_length, 0, 0)}">
          {joint(tibia, (0, -1, 0), limits[2])}
          <geom type="capsule" fromto="0 0 0 {vec(leg.tibia_length, 0, 0)}" \
size="{num(LINK_RADIUS)}" mass="{num(leg.tibia_mass)}"/>
          <geom name="{leg_code}_foot_tip" type="sphere" pos="{vec(leg.tibia_length, 0, 0)}" \
size="{num(leg.foot_radius)}" density="0" contype="1" conaffinity="1" \
friction="{num(FOOT_FRICTION)} 0.005 0.0001"/>
          <site name={q(foot_site(leg_code))} pos="{vec(leg.tibia_length, 0, 0)}" size="0.004"/>
        </body>
      </body>
    </body>"""


def leg_actuators_xml(params: RobotParams, actuators: ActuatorModel, leg_code: str) -> str:
    leg = params.leg
    limits = (leg.coxa_limits, leg.femur_limits, leg.tibia_limits)
    q = quoteattr
    return "\n".join(
        f'    <position name={q(name)} joint={q(name)} kp="{num(actuators.kp)}" '
        f'kv="{num(actuators.kv)}" ctrllimited="true" ctrlrange="{vec(*rng)}" '
        f'forcelimited="true" forcerange="{vec(-actuators.max_torque, actuators.max_torque)}"/>'
        for name, rng in zip(joint_names(leg_code), limits, strict=True)
    )


def build_leg_mjcf(params: RobotParams, actuators: ActuatorModel, leg_code: str = "lf") -> str:
    qpos = vec(*neutral_angles(params))
    body = leg_body_xml(params, leg_code, pos=vec(0, 0, STAND_HEIGHT))
    return f"""<mujoco model="bemu_{leg_code}_leg">
  <compiler angle="radian"/>
  <option timestep="{num(TIMESTEP)}"/>
  <default>
    {joint_defaults_xml(actuators)}
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    {body}
  </worldbody>
  <actuator>
{leg_actuators_xml(params, actuators, leg_code)}
  </actuator>
  <keyframe>
    <key name="neutral" qpos="{qpos}" ctrl="{qpos}"/>
  </keyframe>
</mujoco>
"""
