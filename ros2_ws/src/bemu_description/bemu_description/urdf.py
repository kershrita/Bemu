"""Build the URDF (with its ros2_control block) of one leg (XML text; no mujoco import)."""

from __future__ import annotations

from xml.sax.saxutils import escape, quoteattr

from bemu_description.mjcf import neutral_angles
from bemu_description.names import BODY, joint_names, joint_order
from bemu_kinematics import BodyPose, pose_angles, sit_height
from bemu_params import LEG_CODES, RobotParams

PLUGIN = "mujoco_ros2_control/MujocoSystemInterface"
KEYFRAME = "neutral"


def _num(value: float) -> str:
    return repr(float(value))


def _leg_parts(
    params: RobotParams, leg_code: str, initial: tuple[float, float, float]
) -> tuple[str, str]:
    """The XML of one leg: its links (hip link to foot link), its joints, and its ros2_control
    joint blocks. `initial` is the starting angle of each joint."""
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
    for jname, (child, x, mass), parent, axis, (lo, hi), angle in zip(
        (coxa, femur, tibia), links, parents, axes, limits, initial, strict=True
    ):
        link_xml.append(
            f'  <link name="{child}">\n'
            f"    {_inertial_xml(params, child.removeprefix(leg_code + '_'), mass)}\n"
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
    links_text = (
        "\n".join(link_xml)
        + f'\n  <link name="{leg_code}_foot_link"/>\n'
        + "\n".join(joint_xml)
        + f'\n  <joint name="{leg_code}_foot_fixed" type="fixed">\n'
        f'    <parent link="{leg_code}_tibia"/>\n'
        f'    <child link="{leg_code}_foot_link"/>\n'
        f'    <origin xyz="{_num(leg.tibia_length)} 0 0" rpy="0 0 0"/>\n'
        f"  </joint>"
    )
    return links_text, "\n".join(control_xml)


def _hardware(
    model_name: str, mujoco_model_path: str, headless: bool, keyframe: str, controls: str
) -> str:
    headless_text = "true" if headless else "false"
    return (
        f'  <ros2_control name="{model_name}" type="system">\n'
        f"    <hardware>\n"
        f"      <plugin>{PLUGIN}</plugin>\n"
        f'      <param name="mujoco_model">{escape(mujoco_model_path)}</param>\n'
        f'      <param name="headless">{headless_text}</param>\n'
        f'      <param name="initial_keyframe">{keyframe}</param>\n'
        f"    </hardware>\n" + controls + "\n  </ros2_control>\n"
    )


def _inertial_xml(params: RobotParams, joint: str, mass: float) -> str:
    """The link's inertial block: the CAD centre of mass and inertia, or a placeholder."""
    item = None if params.leg.link_inertia is None else params.leg.link_inertia[joint]
    if item is None:
        return (
            f'<inertial><mass value="{_num(mass)}"/>'
            f'<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>'
        )
    xx, yy, zz, xy, xz, yz = item.inertia
    cx, cy, cz = item.com
    return (
        f'<inertial><origin xyz="{_num(cx)} {_num(cy)} {_num(cz)}" rpy="0 0 0"/>'
        f'<mass value="{_num(mass)}"/>'
        f'<inertia ixx="{_num(xx)}" ixy="{_num(xy)}" ixz="{_num(xz)}" iyy="{_num(yy)}" '
        f'iyz="{_num(yz)}" izz="{_num(zz)}"/></inertial>'
    )


def build_leg_urdf(
    params: RobotParams,
    leg_code: str = "lf",
    *,
    mujoco_model_path: str,
    headless: bool = False,
) -> str:
    body, controls = _leg_parts(params, leg_code, neutral_angles(params))
    hardware = _hardware("BemuLeg", mujoco_model_path, headless, KEYFRAME, controls)
    return (
        f'<?xml version="1.0"?>\n<robot name="bemu_{leg_code}_leg">\n'
        + body
        + "\n"
        + hardware
        + "</robot>\n"
    )


def build_robot_urdf(params: RobotParams, *, mujoco_model_path: str, headless: bool = False) -> str:
    """The full robot: a body link, six legs fixed at their mounts, and one ros2_control system
    over all 18 joints (in joint_order()), starting in the sit pose."""
    sit = pose_angles(params, BodyPose(sit_height(params)))
    legs, controls = [], {}
    for leg_code in LEG_CODES:
        mount = params.mounts[leg_code]
        body, leg_controls = _leg_parts(params, leg_code, sit[leg_code])
        controls[leg_code] = leg_controls
        legs.append(
            body + f'\n  <joint name="{leg_code}_hip_fixed" type="fixed">\n'
            f'    <parent link="{BODY}"/>\n'
            f'    <child link="{leg_code}_hip_link"/>\n'
            f'    <origin xyz="{_num(mount.x)} {_num(mount.y)} 0" rpy="0 0 {_num(mount.yaw)}"/>\n'
            f"  </joint>"
        )
    ordered = "\n".join(controls[leg] for leg in LEG_CODES)
    assert ordered.count("<joint name=") == len(joint_order())
    hardware = _hardware("BemuRobot", mujoco_model_path, headless, "sit", ordered)
    return (
        '<?xml version="1.0"?>\n<robot name="bemu_robot">\n'
        f'  <link name="{BODY}">\n'
        f'    <inertial><mass value="{_num(params.body.mass)}"/>'
        f'<inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/></inertial>\n'
        f"  </link>\n" + "\n".join(legs) + "\n" + hardware + "</robot>\n"
    )
