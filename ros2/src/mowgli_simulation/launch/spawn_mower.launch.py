# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
spawn_mower.launch.py

Spawns the Mowgli mower into an already-running Gazebo Ignition instance.

Intended usage:
  1. Start Gazebo separately (e.g. via `ros2 launch ros_gz_sim gz_sim.launch.py`).
  2. Run this file to inject the mower model and start the ROS2 side-car nodes.

Nodes launched:
  - ros_gz_sim create   : spawns the mowgli_mower SDF model into Gazebo.
  - robot_state_publisher: publishes /robot_description and static TF from URDF.
  - ros_gz_bridge        : bridges Gazebo and ROS2 topics.

Arguments:
  spawn_x   (default 0.0)  — X position in metres.
  spawn_y   (default 0.0)  — Y position in metres.
  spawn_z   (default 0.05) — Z position in metres (slightly above ground).
  spawn_yaw (default 0.0)  — Yaw angle in radians.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    sim_share = get_package_share_directory("mowgli_simulation")
    bringup_share = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Robot spawn X position (metres).",
    )

    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot spawn Y position (metres).",
    )

    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05",
        description="Robot spawn Z position (metres).",
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle (radians).",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="mowgli_mower",
        description="Name to assign the spawned model inside Gazebo.",
    )

    # ------------------------------------------------------------------
    # LaunchConfiguration handles
    # ------------------------------------------------------------------
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    model_name = LaunchConfiguration("model_name")

    # ------------------------------------------------------------------
    # robot_state_publisher — URDF description for RViz / TF
    # ------------------------------------------------------------------
    urdf_xacro = os.path.join(bringup_share, "urdf", "mowgli.urdf.xacro")

    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_xacro]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # Spawn the standalone Gazebo SDF model
    # ------------------------------------------------------------------
    mower_sdf = os.path.join(
        sim_share, "models", "mowgli_mower", "model.sdf"
    )

    spawn_mower_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mowgli",
        output="screen",
        arguments=[
            "-name", model_name,
            "-file", mower_sdf,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # ros_gz_bridge — start after spawner exits successfully
    # ------------------------------------------------------------------
    bridge_config = os.path.join(sim_share, "config", "gazebo_bridge.yaml")

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": True},
        ],
    )

    bridge_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_mower_node,
            on_exit=[bridge_node],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            model_name_arg,
            robot_state_publisher_node,
            spawn_mower_node,
            bridge_after_spawn,
        ]
    )
