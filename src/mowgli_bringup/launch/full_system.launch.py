"""
full_system.launch.py

Complete Mowgli robot mower system launch.

Brings up all subsystems:
  1. mowgli.launch.py        — hardware bridge, RSP, twist_mux
  2. navigation.launch.py    — SLAM, dual EKF, Nav2
  3. Behavior tree node       — mowgli_behavior
  4. Map server               — mowgli_map
  5. Coverage planner          — mowgli_coverage_planner
  6. Wheel odometry            — mowgli_localization
  7. GPS pose converter        — mowgli_localization
  8. Localization monitor      — mowgli_localization
  9. Diagnostics               — mowgli_monitoring
  10. MQTT bridge (optional)   — mowgli_monitoring
  11. foxglove_bridge (optional)  — for GUI
"""

import os

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
    coverage_dir = get_package_share_directory("mowgli_coverage_planner")
    localization_dir = get_package_share_directory("mowgli_localization")
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
        default_value="/dev/ttyUSB0",
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

    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
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

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    coverage_params = os.path.join(coverage_dir, "config", "coverage_planner.yaml")
    localization_params = os.path.join(localization_dir, "config", "localization.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    mqtt_params = os.path.join(monitoring_dir, "config", "mqtt_bridge.yaml")
    foxglove_params = os.path.join(bringup_dir, "config", "foxglove_bridge.yaml")

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
    # 5. Coverage planner
    # ------------------------------------------------------------------
    coverage_planner_node = Node(
        package="mowgli_coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            coverage_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 6. Wheel odometry
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
    # 7. GPS pose converter
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
            foxglove_params,
            {"port": foxglove_port},
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
            # Subsystem includes
            mowgli_launch,
            navigation_launch,
            # Individual nodes
            behavior_tree_node,
            map_server_node,
            coverage_planner_node,
            wheel_odometry_node,
            gps_pose_converter_node,
            localization_monitor_node,
            diagnostics_node,
            mqtt_bridge_node,
            foxglove_bridge_node,
        ]
    )
