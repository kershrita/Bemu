"""Launch the simulated robot: MuJoCo bridge, ros2_control controllers and the walker node."""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from bemu_description.actuators import load_actuator_model
from bemu_description.generate import write_robot_models
from bemu_params import load_params


def _setup(context):
    if LaunchConfiguration("use_sim").perform(context).lower() != "true":
        raise RuntimeError("Real hardware arrives in version 2")
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    params_file = LaunchConfiguration("params_file").perform(context)

    params = load_params(params_file or None)
    actuators = load_actuator_model(
        Path(get_package_share_directory("bemu_sim")) / "config" / "actuators.yaml", params.servo
    )
    models = write_robot_models(
        params, actuators, Path(tempfile.mkdtemp(prefix="bemu_model_")), headless=headless
    )
    urdf = models["urdf"].read_text()
    controllers = str(
        Path(get_package_share_directory("bemu_control")) / "config" / "robot_controllers.yaml"
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
        # The controller reads its own parameters (joints, interface_name) from the file
        # given to its spawner, not from the file given to the simulator node.
        arguments=[
            "leg_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers,
        ],
        parameters=[{"use_sim_time": True}],
    )
    walker = Node(
        package="bemu_locomotion",
        executable="locomotion_node",
        output="screen",
        parameters=[{"use_sim_time": True, "params_file": params_file}],
    )

    def start_walker_if_controller_loaded(event, context):
        if event.returncode == 0:
            return [walker]
        return [
            LogInfo(msg="leg_position_controller failed to start; not starting the walker"),
            EmitEvent(event=Shutdown(reason="leg_position_controller failed to start")),
        ]

    return [
        simulator,
        state_publisher,
        broadcaster,
        # Spawn the position controller only after the broadcaster is up.
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[leg_controller])),
        RegisterEventHandler(
            OnProcessExit(target_action=leg_controller, on_exit=start_walker_if_controller_loaded)
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "params_file", default_value=EnvironmentVariable("BEMU_PARAMS", default_value="")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
