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
full_system.launch.py

Complete Mowgli robot mower system launch.

Brings up all subsystems:
  1. mowgli.launch.py        — hardware bridge, RSP, twist_mux
  2. navigation.launch.py    — SLAM, dual EKF, Nav2
  3. Behavior tree node       — mowgli_behavior
  4. Map server               — mowgli_map
  5. Coverage server (optional) — opennav_coverage
  6. Wheel odometry            — mowgli_localization
  7. GPS pose converter        — mowgli_localization
  8. Localization monitor      — mowgli_localization
  9. Diagnostics               — mowgli_monitoring
  10. MQTT bridge (optional)   — mowgli_monitoring
  11. foxglove_bridge (optional)  — for GUI
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")
    behavior_dir = get_package_share_directory("mowgli_behavior")
    map_dir = get_package_share_directory("mowgli_map")
    monitoring_dir = get_package_share_directory("mowgli_monitoring")

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
        description="Serial port for the hardware bridge.",
    )

    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Run slam_toolbox when true; skip when using a pre-built map.",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file (used when slam=false).",
    )

    enable_mqtt_arg = DeclareLaunchArgument(
        "enable_mqtt",
        default_value="false",
        description="Launch the MQTT bridge node when true.",
    )

    enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="true",
        description="Launch foxglove_bridge for the GUI when true.",
    )

    enable_rosbridge_arg = DeclareLaunchArgument(
        "enable_rosbridge",
        default_value="true",
        description="Launch rosbridge_server for the openmower-gui when true.",
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="Port number for the rosbridge WebSocket server.",
    )

    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Enable LiDAR-dependent nodes (SLAM, obstacle tracker, slam heading). Set to false for GPS-only operation.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    enable_mqtt = LaunchConfiguration("enable_mqtt")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    nav2_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    localization_params = os.path.join(bringup_dir, "config", "localization.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    mqtt_params = os.path.join(monitoring_dir, "config", "mqtt_bridge.yaml")
    # Robot-specific config (bind-mounted from mowgli-docker/config/mowgli/)
    robot_config = "/ros2_ws/config/mowgli_robot.yaml"

    # Load robot config to extract mowgli parameters for nodes that need
    # explicit values (e.g. navsat_to_absolute_pose needs datum from mowgli).
    robot_params = {}
    if os.path.isfile(robot_config):
        with open(robot_config, "r") as f:
            robot_config_yaml = yaml.safe_load(f) or {}
        robot_params = robot_config_yaml.get("mowgli", {}).get("ros__parameters", {})

    # ------------------------------------------------------------------
    # 1. mowgli.launch.py — hardware bridge, RSP, twist_mux
    # ------------------------------------------------------------------
    mowgli_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "mowgli.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "serial_port": serial_port,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. navigation.launch.py — SLAM, dual EKF, Nav2
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam,
            "map": map_yaml,
            "slam_mode": "lifelong",
            "map_file_name": "/ros2_ws/maps/garden_map",
            "use_lidar": use_lidar,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3. Behavior tree node
    # ------------------------------------------------------------------
    behavior_tree_node = Node(
        package="mowgli_behavior",
        executable="behavior_tree_node",
        name="behavior_tree_node",
        output="screen",
        parameters=[
            behavior_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 4. Map server
    # ------------------------------------------------------------------
    map_server_node = Node(
        package="mowgli_map",
        executable="map_server_node",
        name="map_server_node",
        output="screen",
        parameters=[
            map_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 5. Wheel odometry
    # ------------------------------------------------------------------
    wheel_odometry_node = Node(
        package="mowgli_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 7a. NavSatFix → AbsolutePose converter (ublox_gps bridge)
    # ------------------------------------------------------------------
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            localization_params,
            {
                "datum_lat": robot_params.get("datum_lat", 0.0),
                "datum_lon": robot_params.get("datum_lon", 0.0),
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 7b. GPS pose converter (AbsolutePose → PoseWithCovarianceStamped)
    # ------------------------------------------------------------------
    gps_pose_converter_node = Node(
        package="mowgli_localization",
        executable="gps_pose_converter_node",
        name="gps_pose_converter_node",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 7c. SLAM heading extractor
    # ------------------------------------------------------------------
    # Extracts yaw from SLAM's slam_map→odom TF and publishes it as a
    # PoseWithCovarianceStamped for the EKF to fuse. Provides absolute
    # heading from LiDAR map matching — works without magnetometer,
    # when stationary, and under canopy.
    slam_heading_node = Node(
        condition=IfCondition(use_lidar),
        package="mowgli_localization",
        executable="slam_heading_node",
        name="slam_heading",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 8. Localization monitor
    # ------------------------------------------------------------------
    localization_monitor_node = Node(
        package="mowgli_localization",
        executable="localization_monitor_node",
        name="localization_monitor_node",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 9. Diagnostics
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[
            monitoring_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 10. MQTT bridge (optional)
    # ------------------------------------------------------------------
    mqtt_bridge_node = Node(
        condition=IfCondition(enable_mqtt),
        package="mowgli_monitoring",
        executable="mqtt_bridge_node",
        name="mqtt_bridge_node",
        output="screen",
        parameters=[
            mqtt_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 11. Foxglove Bridge (optional) — binary WebSocket bridge for GUI
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        condition=IfCondition(enable_foxglove),
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "send_buffer_limit": 10000000,
                "num_threads": 0,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 12. rosbridge_server + rosapi (optional) — WebSocket bridge for
    #     openmower-gui.  rosapi provides the service endpoints
    #     (e.g. /rosapi/topics_and_raw_types) that the GUI needs to
    #     discover available topics.
    # ------------------------------------------------------------------
    rosbridge_node = Node(
        condition=IfCondition(enable_rosbridge),
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {
                "port": 9090,
                "address": "0.0.0.0",
                "unregister_timeout": 10.0,
                "max_message_size": 10000000,
            },
        ],
    )

    rosapi_node = Node(
        condition=IfCondition(enable_rosbridge),
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # NOTE: docking_server is launched and lifecycle-managed by Nav2's
    # navigation_launch.py (in the lifecycle_nodes list). Do NOT launch
    # it here — duplicating it exhausts DDS participants and causes
    # lifecycle conflicts.

    # ------------------------------------------------------------------
    # 13. Obstacle tracker — persistent LiDAR obstacle detection
    # ------------------------------------------------------------------
    obstacle_tracker_params = os.path.join(
        map_dir, "config", "obstacle_tracker.yaml"
    )

    obstacle_tracker_node = Node(
        condition=IfCondition(use_lidar),
        package="mowgli_map",
        executable="obstacle_tracker_node",
        name="obstacle_tracker",
        output="screen",
        parameters=[
            obstacle_tracker_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            serial_port_arg,
            slam_arg,
            map_arg,
            enable_mqtt_arg,
            enable_foxglove_arg,
            foxglove_port_arg,
            enable_rosbridge_arg,
            rosbridge_port_arg,
            use_lidar_arg,
            # Subsystem includes
            mowgli_launch,
            navigation_launch,
            # Individual nodes
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            wheel_odometry_node,
            navsat_converter_node,
            gps_pose_converter_node,
            slam_heading_node,
            localization_monitor_node,
            diagnostics_node,
            mqtt_bridge_node,
            foxglove_bridge_node,
            rosbridge_node,
            rosapi_node,
        ]
    )
