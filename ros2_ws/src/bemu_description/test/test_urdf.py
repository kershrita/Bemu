from xml.etree import ElementTree

import pytest

from bemu_description.generate import write_models
from bemu_description.urdf import build_leg_urdf

NEUTRAL = (0.0, 0.300494, -1.753631)
JOINTS = ("lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint")


@pytest.fixture
def root(params):
    return ElementTree.fromstring(build_leg_urdf(params, mujoco_model_path="/tmp/leg.xml"))


def test_ros2_control_block(root):
    control = root.find("ros2_control")
    assert control.get("name") == "BemuLeg"
    assert control.get("type") == "system"
    hw = root.find("ros2_control/hardware")
    assert hw.findtext("plugin") == "mujoco_ros2_control/MujocoSystemInterface"
    assert {p.get("name"): p.text for p in hw.findall("param")} == {
        "mujoco_model": "/tmp/leg.xml",
        "headless": "false",
        "initial_keyframe": "neutral",
    }


def test_headless_flag(params):
    urdf = build_leg_urdf(params, mujoco_model_path="/tmp/leg.xml", headless=True)
    hw = ElementTree.fromstring(urdf).find("ros2_control/hardware")
    assert {p.get("name"): p.text for p in hw.findall("param")}["headless"] == "true"


def test_joints_limits_and_interfaces(root, params):
    leg = params.leg
    limits = dict(zip(JOINTS, [leg.coxa_limits, leg.femur_limits, leg.tibia_limits], strict=True))
    for name in JOINTS:
        joint = next(j for j in root.findall("joint") if j.get("name") == name)
        assert joint.get("type") == "revolute"
        lim = joint.find("limit")
        assert float(lim.get("lower")) == pytest.approx(limits[name][0], abs=1e-6)
        assert float(lim.get("upper")) == pytest.approx(limits[name][1], abs=1e-6)

    control_joints = {j.get("name"): j for j in root.findall("ros2_control/joint")}
    assert set(control_joints) == set(JOINTS)
    for name, neutral in zip(JOINTS, NEUTRAL, strict=True):
        joint = control_joints[name]
        commands = joint.findall("command_interface")
        assert [c.get("name") for c in commands] == ["position"]
        states = {s.get("name"): s for s in joint.findall("state_interface")}
        assert set(states) == {"position", "velocity", "effort"}
        initial = float(states["position"].find("param").text)
        assert states["position"].find("param").get("name") == "initial_value"
        assert initial == pytest.approx(neutral, abs=1e-6)


def test_write_models_creates_both_files(tmp_path, params, actuators):
    import mujoco

    paths = write_models(params, actuators, tmp_path / "out")
    assert set(paths) == {"mjcf", "urdf"}
    assert paths["mjcf"].name == "leg.xml" and paths["urdf"].name == "leg.urdf"
    assert paths["mjcf"].is_file() and paths["urdf"].is_file()
    assert mujoco.MjModel.from_xml_path(str(paths["mjcf"])).nu == 3
    urdf = ElementTree.parse(paths["urdf"]).getroot()
    assert urdf.tag == "robot"
    mujoco_model = urdf.find("ros2_control/hardware/param[@name='mujoco_model']").text
    assert mujoco_model == str(paths["mjcf"].resolve())


def test_mujoco_model_path_is_xml_escaped(params):
    path = "/tmp/a&b/<leg>.xml"
    root = ElementTree.fromstring(build_leg_urdf(params, mujoco_model_path=path))
    values = {p.get("name"): p.text for p in root.iter("param")}
    assert values["mujoco_model"] == path
