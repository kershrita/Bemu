import xml.etree.ElementTree as ET

import mujoco
import pytest

from bemu_description.generate import write_robot_models
from bemu_description.names import joint_order
from bemu_description.urdf import build_leg_urdf, build_robot_urdf
from bemu_kinematics import BodyPose, pose_angles, sit_height
from bemu_params import LEG_CODES


@pytest.fixture
def root(params):
    return ET.fromstring(build_robot_urdf(params, mujoco_model_path="/tmp/robot.xml"))


def test_eighteen_revolute_joints_in_order(root):
    revolute = [j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"]
    assert revolute == list(joint_order())


def test_ros2_control_lists_the_joints_in_joint_order(root):
    control = root.find("ros2_control")
    assert control.get("name") == "BemuRobot"
    assert [j.get("name") for j in control.findall("joint")] == list(joint_order())
    hardware = control.find("hardware")
    assert hardware.find("param[@name='initial_keyframe']").text == "sit"
    assert hardware.find("param[@name='mujoco_model']").text == "/tmp/robot.xml"


def test_hips_are_fixed_at_the_mounts(root, params):
    for leg in LEG_CODES:
        joint = next(j for j in root.findall("joint") if j.get("name") == f"{leg}_hip_fixed")
        assert joint.get("type") == "fixed"
        assert joint.find("parent").get("link") == "body"
        assert joint.find("child").get("link") == f"{leg}_hip_link"
        origin = joint.find("origin")
        x, y, z = (float(v) for v in origin.get("xyz").split())
        _, _, yaw = (float(v) for v in origin.get("rpy").split())
        mount = params.mounts[leg]
        assert (x, y, z, yaw) == pytest.approx((mount.x, mount.y, 0.0, mount.yaw))


def test_initial_values_are_the_sit_pose(root, params):
    sit = pose_angles(params, BodyPose(sit_height(params)))
    expected = [angle for leg in LEG_CODES for angle in sit[leg]]
    values = [
        float(j.find("state_interface[@name='position']/param").text)
        for j in root.find("ros2_control").findall("joint")
    ]
    assert values == pytest.approx(expected, abs=1e-9)


def test_headless_flag_and_path_escaping(params):
    text = build_robot_urdf(params, mujoco_model_path="/tmp/a&b/robot.xml", headless=True)
    hardware = ET.fromstring(text).find("ros2_control/hardware")
    assert hardware.find("param[@name='headless']").text == "true"
    assert hardware.find("param[@name='mujoco_model']").text == "/tmp/a&b/robot.xml"


def test_leg_joints_match_the_single_leg_builder(root, params):
    leg = ET.fromstring(build_leg_urdf(params, "lf", mujoco_model_path="x"))

    def describe(tree, name):
        joint = next(j for j in tree.findall("joint") if j.get("name") == name)
        return (
            joint.find("axis").get("xyz"),
            joint.find("limit").attrib,
            joint.find("origin").get("xyz"),
        )

    for name in ("lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint"):
        assert describe(root, name) == describe(leg, name)


def test_write_robot_models(params, actuators, tmp_path):
    files = write_robot_models(params, actuators, tmp_path)
    assert files["mjcf"].name == "robot.xml" and files["urdf"].name == "robot.urdf"
    urdf = ET.fromstring(files["urdf"].read_text())
    assert urdf.find("ros2_control/hardware/param[@name='mujoco_model']").text == str(files["mjcf"])
    model = mujoco.MjModel.from_xml_path(str(files["mjcf"]))
    assert model.nu == 18


def test_urdf_links_carry_the_cad_centre_of_mass(root, params):
    link = next(x for x in root.findall("link") if x.get("name") == "lf_femur")
    origin = link.find("inertial/origin")
    assert float(origin.get("xyz").split()[0]) == pytest.approx(
        params.leg.link_inertia["femur"].com[0]
    )
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(params.leg.femur_mass)
