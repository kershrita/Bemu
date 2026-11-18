"""Build the URDF (with its ros2_control block) of one leg (XML text; no mujoco import)."""

from __future__ import annotations

from xml.sax.saxutils import quoteattr

from bemu_description.mjcf import neutral_angles
from bemu_description.names import joint_names
from bemu_params import RobotParams

PLUGIN = "mujoco_ros2_control/MujocoSystemInterface"
KEYFRAME = "neutral"


def _num(value: float) -> str:
    return repr(float(value))


def build_leg_urdf(
    params: RobotParams,
    leg_code: str = "lf",
    *,
    mujoco_model_path: str,
    headless: bool = False,
) -> str:
    leg = params.leg
    coxa, femur, tibia = joint_names(leg_code)
    q = quoteattr
    limits = (leg.coxa_limits, leg.femur_limits, leg.tibia_limits)
    axes = ("0 0 1", "0 -1 0", "0 -1 0")
    # (child link, joint origin x in the parent link frame, mass)
    links = (
        (f"{leg_code}_coxa", 0.0, leg.coxa_mass),
        (f"{leg_code}_femur", leg.coxa_length, leg.femur_mass),
        (f"{leg_code}_tibia", leg.femur_length, leg.tibia_mass),
    )
    parents = (f"{leg_code}_hip_link", *(name for name, _, _ in links[:2]))
    torque = params.servo.stall_torque
    speed = params.servo.max_speed

    link_xml = [f'  <link name="{leg_code}_hip_link"/>']
    joint_xml = []
    control_xml = []
    neutral = neutral_angles(params)
    for jname, (child, x, mass), parent, axis, (lo, hi), angle in zip(
        (coxa, femur, tibia), links, parents, axes, limits, neutral, strict=True
    ):
        link_xml.append(
            f'  <link name="{child}">\n'
            f'    <inertial><mass value="{_num(mass)}"/>'
            f'<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>\n'
            f"  </link>"
        )
        joint_xml.append(
            f'  <joint name={q(jname)} type="revolute">\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <origin xyz="{_num(x)} 0 0" rpy="0 0 0"/>\n'
            f'    <axis xyz="{axis}"/>\n'
            f'    <limit lower="{_num(lo)}" upper="{_num(hi)}" effort="{_num(torque)}" '
            f'velocity="{_num(speed)}"/>\n'
            f"  </joint>"
        )
        control_xml.append(
            f"    <joint name={q(jname)}>\n"
            f'      <command_interface name="position"/>\n'
            f'      <state_interface name="position"><param name="initial_value">{_num(angle)}'
            f"</param></state_interface>\n"
            f'      <state_interface name="velocity"/>\n'
            f'      <state_interface name="effort"/>\n'
            f"    </joint>"
        )
    headless_text = "true" if headless else "false"
    return (
        f'<?xml version="1.0"?>\n<robot name="bemu_{leg_code}_leg">\n'
        + "\n".join(link_xml)
        + f'\n  <link name="{leg_code}_foot_link"/>\n'
        + "\n".join(joint_xml)
        + f'\n  <joint name="{leg_code}_foot_fixed" type="fixed">\n'
        f'    <parent link="{leg_code}_tibia"/>\n'
        f'    <child link="{leg_code}_foot_link"/>\n'
        f'    <origin xyz="{_num(leg.tibia_length)} 0 0" rpy="0 0 0"/>\n'
        f"  </joint>\n"
        f'  <ros2_control name="BemuLeg" type="system">\n'
        f"    <hardware>\n"
        f"      <plugin>{PLUGIN}</plugin>\n"
        f'      <param name="mujoco_model">{mujoco_model_path}</param>\n'
        f'      <param name="headless">{headless_text}</param>\n'
        f'      <param name="initial_keyframe">{KEYFRAME}</param>\n'
        f"    </hardware>\n" + "\n".join(control_xml) + "\n  </ros2_control>\n</robot>\n"
    )
