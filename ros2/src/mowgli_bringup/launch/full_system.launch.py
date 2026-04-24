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
  2. navigation.launch.py    — SLAM, FusionCore, Nav2
  3. Behavior tree node       — mowgli_behavior
  4. Map server               — mowgli_map
  5. Wheel odometry            — mowgli_localization
  6. NavSat converter          — mowgli_localization (GPS for GUI/BT)
  7. Localization monitor      — mowgli_localization
  8. Diagnostics               — mowgli_monitoring
  9. MQTT bridge (optional)   — mowgli_monitoring
  10. foxglove_bridge — WebSocket bridge for GUI and Foxglove Studio

GNSS backend selection:
  - gps      — legacy/direct GPS path
  - ublox    — u-blox GNSS backend
  - unicore  — Unicore GNSS backend

Hardware backend selection:
  - mowgli   — direct Mowgli hardware bridge
  - mavros   — Pixhawk/MAVROS backend
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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
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

    hardware_backend_arg = DeclareLaunchArgument(
        "hardware_backend",
        default_value=EnvironmentVariable("HARDWARE_BACKEND", default_value="mowgli"),
        description="Hardware backend: 'mowgli' or 'mavros'.",
    )

    enable_mqtt_arg = DeclareLaunchArgument(
        "enable_mqtt",
        default_value="false",
        description="Launch the MQTT bridge node when true.",
    )

    gnss_backend_arg = DeclareLaunchArgument(
        "gnss_backend",
        default_value="none",
        description="GNSS backend to use: 'ublox', 'unicore', or 'none'.",
    )

    enable_ublox_gnss_arg = DeclareLaunchArgument(
        "enable_ublox_gnss",
        default_value="false",
        description="Launch the minimal u-blox GNSS V1 backend when true.",
    )

    ublox_device_family_arg = DeclareLaunchArgument(
        "ublox_device_family",
        default_value="F9P",
        description="u-blox device family: F9P, F9R, or X20P.",
    )

    ublox_device_serial_string_arg = DeclareLaunchArgument(
        "ublox_device_serial_string",
        default_value="",
        description="Optional USB serial string to select a specific u-blox receiver.",
    )

    ublox_frame_id_arg = DeclareLaunchArgument(
        "ublox_frame_id",
        default_value="gps_link",
        description="Frame ID reported by the u-blox backend.",
    )

    ublox_config_file_arg = DeclareLaunchArgument(
        "ublox_config_file",
        default_value="",
        description="Optional explicit u-blox TOML config file path.",
    )

    enable_unicore_gnss_arg = DeclareLaunchArgument(
        "enable_unicore_gnss",
        default_value="false",
        description="Launch the Unicore UM982 GNSS backend when true.",
    )

    unicore_serial_port_arg = DeclareLaunchArgument(
        "unicore_serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Unicore UM982 GNSS receiver.",
    )

    unicore_baudrate_arg = DeclareLaunchArgument(
        "unicore_baudrate",
        default_value="921600",
        description="Baudrate for Unicore UM982 serial communication.",
    )

    unicore_frame_id_arg = DeclareLaunchArgument(
        "unicore_frame_id",
        default_value="gnss",
        description="Frame ID reported by the Unicore backend.",
    )

    enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="true",
        description="Launch foxglove_bridge for the GUI when true.",
    )

    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Enable LiDAR-dependent nodes (Kinematic-ICP drift correction, obstacle layer, collision monitor scan). Set to false for GPS-only operation without a LiDAR.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    hardware_backend = LaunchConfiguration("hardware_backend")
    enable_mqtt = LaunchConfiguration("enable_mqtt")
    enable_ublox_gnss = LaunchConfiguration("enable_ublox_gnss")
    ublox_device_family = LaunchConfiguration("ublox_device_family")
    ublox_device_serial_string = LaunchConfiguration("ublox_device_serial_string")
    ublox_frame_id = LaunchConfiguration("ublox_frame_id")
    ublox_config_file = LaunchConfiguration("ublox_config_file")
    enable_unicore_gnss = LaunchConfiguration("enable_unicore_gnss")
    unicore_serial_port = LaunchConfiguration("unicore_serial_port")
    unicore_baudrate = LaunchConfiguration("unicore_baudrate")
    unicore_frame_id = LaunchConfiguration("unicore_frame_id")
    gnss_backend = LaunchConfiguration("gnss_backend")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # GNSS backend selection logic
    # ------------------------------------------------------------------
    # Auto-enable the appropriate backend based on gnss_backend argument
    enable_ublox_condition = IfCondition(
        PythonExpression(["'", gnss_backend, "' == 'ublox'"])
    )
    enable_unicore_condition = IfCondition(
        PythonExpression(["'", gnss_backend, "' == 'unicore'"])
    )

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
            "hardware_backend": hardware_backend,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 1b. Optional u-blox GNSS backend — vendor driver behind Mowgli adapter
    # ------------------------------------------------------------------
    ublox_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "ublox_gnss.launch.py")
        ),
        condition=enable_ublox_condition,
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ublox_device_family": ublox_device_family,
            "ublox_device_serial_string": ublox_device_serial_string,
            "ublox_frame_id": ublox_frame_id,
            "ublox_config_file": ublox_config_file,
        }.items(),
    )

    unicore_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "unicore_gnss.launch.py")
        ),
        condition=enable_unicore_condition,
        launch_arguments={
            "use_sim_time": use_sim_time,
            "unicore_serial_port": unicore_serial_port,
            "unicore_baudrate": unicore_baudrate,
            "unicore_frame_id": unicore_frame_id,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. navigation.launch.py — FusionCore + Nav2 (+ optional Kinematic-ICP)
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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
    # Wheel odometry is produced directly by hardware_bridge on
    # /wheel_odom (from the firmware's odom packet). The old
    # mowgli_localization/wheel_odometry_node subscribed to /wheel_ticks
    # and re-published /wheel_odom — but /wheel_ticks has no publisher
    # on this branch, so that node was dead weight and a duplicate
    # publisher for /wheel_odom. Keep the source in the package for now
    # (disabled) and rely on hardware_bridge alone.
    # ------------------------------------------------------------------
    # 7a. NavSat → AbsolutePose converter (for GUI + BT)
    # FusionCore takes /gps/fix directly; this node converts to
    # /gps/absolute_pose for the GUI and behavior tree.
    # ------------------------------------------------------------------
    datum_lat = float(robot_params.get("datum_lat", 0.0))
    datum_lon = float(robot_params.get("datum_lon", 0.0))
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            localization_params,
            {"datum_lat": datum_lat, "datum_lon": datum_lon},
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
    # 8b. IMU yaw calibration node (on-demand)
    # Exposes /calibrate_imu_yaw_node/calibrate — idle until called.
    # ------------------------------------------------------------------
    calibrate_imu_yaw_node = Node(
        package="mowgli_localization",
        executable="calibrate_imu_yaw_node.py",
        name="calibrate_imu_yaw_node",
        output="screen",
        parameters=[
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
    # 11. Foxglove Bridge — WebSocket bridge for GUI and Foxglove Studio
    # ------------------------------------------------------------------
    # No topic/service whitelists — all topics are available for Foxglove
    # Studio debugging. The GUI backend throttles subscriptions on its side.
    foxglove_bridge_node = Node(
        condition=IfCondition(enable_foxglove),
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": foxglove_port,
                "address": "0.0.0.0",
                "send_buffer_limit": 10000000,
                "num_threads": 0,
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                ],
            },
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

    # Obstacle tracker disabled by default — creates large persistent obstacles
    # from lidar that block the planner. Collision_monitor handles real-time
    # obstacle avoidance instead.
    obstacle_tracker_node = Node(
        condition=IfCondition("false"),
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
            hardware_backend_arg,
            enable_mqtt_arg,
            gnss_backend_arg,
            enable_ublox_gnss_arg,
            ublox_device_family_arg,
            ublox_device_serial_string_arg,
            ublox_frame_id_arg,
            ublox_config_file_arg,
            enable_unicore_gnss_arg,
            unicore_serial_port_arg,
            unicore_baudrate_arg,
            unicore_frame_id_arg,
            enable_foxglove_arg,
            foxglove_port_arg,
            use_lidar_arg,
            # Subsystem includes
            mowgli_launch,
            ublox_gnss_launch,
            unicore_gnss_launch,
            navigation_launch,
            # Individual nodes
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            navsat_converter_node,  # publishes /gps/absolute_pose for GUI + BT
            localization_monitor_node,
            calibrate_imu_yaw_node,
            diagnostics_node,
            mqtt_bridge_node,
            foxglove_bridge_node,
            # Dock heading is published by hardware_bridge at 1 Hz while
            # charging (~/dock_heading → /gnss/heading via mowgli.launch.py
            # remapping). No separate launch action needed.
        ]
    )
