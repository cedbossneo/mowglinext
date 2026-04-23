import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_config_path = "/ros2_ws/config/mowgli_robot.yaml"
    robot_params = {}

    if os.path.isfile(robot_config_path):
        with open(robot_config_path, "r", encoding="utf-8") as config_file:
            robot_config = yaml.safe_load(config_file) or {}
        robot_params = robot_config.get("mowgli", {}).get("ros__parameters", {})

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value=str(robot_params.get("ntrip_enabled", False)).lower(),
            ),
            DeclareLaunchArgument(
                "host", default_value=str(robot_params.get("ntrip_host", "127.0.0.1"))
            ),
            DeclareLaunchArgument(
                "port", default_value=str(robot_params.get("ntrip_port", 2101))
            ),
            DeclareLaunchArgument(
                "mountpoint",
                default_value=str(robot_params.get("ntrip_mountpoint", "")),
            ),
            DeclareLaunchArgument(
                "username", default_value=str(robot_params.get("ntrip_user", ""))
            ),
            DeclareLaunchArgument(
                "password", default_value=str(robot_params.get("ntrip_password", ""))
            ),
            DeclareLaunchArgument("frame_id", default_value="gps"),
            DeclareLaunchArgument(
                "user_agent", default_value="mowgli_ntrip_client/0.1"
            ),
            DeclareLaunchArgument("reconnect_delay_ms", default_value="5000"),
            DeclareLaunchArgument("connect_timeout_ms", default_value="5000"),
            DeclareLaunchArgument("read_timeout_ms", default_value="15000"),
            DeclareLaunchArgument("status_log_period_ms", default_value="10000"),
            Node(
                condition=IfCondition(LaunchConfiguration("enabled")),
                package="mowgli_ntrip_client",
                executable="mowgli_ntrip_client_node",
                name="mowgli_ntrip_client",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "mountpoint": LaunchConfiguration("mountpoint"),
                        "username": LaunchConfiguration("username"),
                        "password": LaunchConfiguration("password"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "user_agent": LaunchConfiguration("user_agent"),
                        "reconnect_delay_ms": LaunchConfiguration(
                            "reconnect_delay_ms"
                        ),
                        "connect_timeout_ms": LaunchConfiguration(
                            "connect_timeout_ms"
                        ),
                        "read_timeout_ms": LaunchConfiguration("read_timeout_ms"),
                        "status_log_period_ms": LaunchConfiguration(
                            "status_log_period_ms"
                        ),
                    }
                ],
                remappings=[("/rtcm", "/mavros/gps_rtk/send_rtcm")],
            ),
        ]
    )
