"""
mowgli.launch.py

Main bringup launch file for the Mowgli robot mower (physical hardware).

Brings up:
  1. robot_state_publisher  – processes URDF/xacro and publishes /robot_description
                              plus static TF from URDF fixed joints.
  2. hardware_bridge        – serial bridge to the Mowgli firmware board.
  3. twist_mux              – priority-based cmd_vel multiplexer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/mowgli",
        description="Serial port connected to the Mowgli firmware board.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")

    # ------------------------------------------------------------------
    # URDF / xacro
    # ------------------------------------------------------------------
    xacro_file = os.path.join(bringup_dir, "urdf", "mowgli.urdf.xacro")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2. hardware_bridge
    hardware_bridge_params = os.path.join(
        bringup_dir, "config", "hardware_bridge.yaml"
    )

    hardware_bridge_node = Node(
        package="mowgli_hardware",
        executable="hardware_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=[
            hardware_bridge_params,
            # Allow command-line override of the serial port.
            {"serial_port": serial_port},
            {"use_sim_time": use_sim_time},
        ],
    )

    # 3. twist_mux
    twist_mux_params = os.path.join(bringup_dir, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time},
        ],
        # Remap the mux output to the topic consumed by hardware_bridge
        remappings=[("cmd_vel_out", "cmd_vel")],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            robot_state_publisher_node,
            hardware_bridge_node,
            twist_mux_node,
        ]
    )
