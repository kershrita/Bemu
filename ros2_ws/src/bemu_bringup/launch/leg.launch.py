"""Launch the simulated leg: MuJoCo bridge, ros2_control controllers, optional foot trace."""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from bemu_description.actuators import load_actuator_model
from bemu_description.generate import write_models
from bemu_params import load_params


def _setup(context):
    if LaunchConfiguration("use_sim").perform(context).lower() != "true":
        raise RuntimeError("Real hardware arrives in version 2")
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    shape = LaunchConfiguration("shape").perform(context)
    if shape not in ("none", "square", "circle"):
        raise RuntimeError(f"shape must be none, square or circle, got {shape!r}")
    params_file = LaunchConfiguration("params_file").perform(context)

    params = load_params(params_file or None)
    actuators = load_actuator_model(
        Path(get_package_share_directory("bemu_sim")) / "config" / "actuators.yaml", params.servo
    )
    models = write_models(
        params, actuators, Path(tempfile.mkdtemp(prefix="bemu_model_")), headless=headless
    )
    urdf = models["urdf"].read_text()
    controllers = str(
        Path(get_package_share_directory("bemu_control")) / "config" / "leg_controllers.yaml"
    )

    simulator = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        # No robot_description here: this ros2_control version reads it from the
        # /robot_description topic, which robot_state_publisher below publishes.
        parameters=[{"use_sim_time": True}, controllers],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": urdf}],
    )
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    leg_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    actions = [
        simulator,
        state_publisher,
        broadcaster,
        # Spawn the position controller only after the broadcaster is up.
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[leg_controller])),
    ]
    if shape != "none":
        trace = Node(
            package="bemu_bringup",
            executable="foot_trace_node",
            output="screen",
            parameters=[
                {"use_sim_time": True, "shape": shape, "params_file": params_file},
            ],
        )
        actions.append(
            RegisterEventHandler(OnProcessExit(target_action=leg_controller, on_exit=[trace]))
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("shape", default_value="none"),
            DeclareLaunchArgument(
                "params_file", default_value=EnvironmentVariable("BEMU_PARAMS", default_value="")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
