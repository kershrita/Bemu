import re
import xml.etree.ElementTree as ET
from pathlib import Path

from bemu_params import LEG_CODES

PACKAGE = Path(__file__).resolve().parents[1]


def fields(name):
    out = []
    for line in (PACKAGE / "msg" / f"{name}.msg").read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            kind, field = re.split(r"\s+", line)[:2]
            out.append((kind, field))
    return out


def test_gait_command_fields():
    assert fields("GaitCommand") == [
        ("string", "gait"),
        ("float64", "step_height"),
        ("float64", "cycle_time"),
    ]


def test_gait_state_fields():
    legs = len(LEG_CODES)
    assert legs == 6
    assert fields("GaitState") == [
        ("std_msgs/Header", "header"),
        ("string", "gait"),
        ("string", "mode"),
        (f"float64[{legs}]", "phase"),
        (f"bool[{legs}]", "foot_down"),
        ("float64", "stability_margin"),
    ]


def test_package_declares_the_interface_build():
    root = ET.parse(PACKAGE / "package.xml").getroot()
    tags = {(e.tag, (e.text or "").strip()) for e in root}
    for needed in (
        ("buildtool_depend", "rosidl_default_generators"),
        ("exec_depend", "rosidl_default_runtime"),
        ("depend", "std_msgs"),
        ("member_of_group", "rosidl_interface_packages"),
    ):
        assert needed in tags, needed
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "msg/GaitCommand.msg" in cmake and "msg/GaitState.msg" in cmake
