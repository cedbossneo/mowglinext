"""
navigation.launch.py

Navigation stack launch file for the Mowgli robot mower.

Brings up:
  1. slam_toolbox           – online SLAM (mapping mode by default).
  2. robot_localization     – dual EKF:
       ekf_odom: wheel_odom + IMU  →  odom → base_link TF
       ekf_map:  filtered_odom + GPS pose → map → odom TF
  3. Nav2 bringup           – full navigation stack (controller, planner,
                              recoveries, BT navigator, costmaps, lifecycle).
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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )

    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Run slam_toolbox when true; skip when using a pre-built map.",
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file (used when slam=false).",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    slam_toolbox_params = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")
    localization_params = os.path.join(bringup_dir, "config", "localization.yaml")
    nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    # ------------------------------------------------------------------
    # 1. slam_toolbox  (only when slam=true)
    # ------------------------------------------------------------------
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_toolbox_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 2. robot_localization – odom EKF
    # ------------------------------------------------------------------
    ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odometry/filtered_odom")],
    )

    # 3. robot_localization – map EKF
    ekf_map_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odometry/filtered_map")],
    )

    # ------------------------------------------------------------------
    # 4. Nav2 bringup
    # ------------------------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            # Pass map only when not running slam
            "slam": slam,
            "map": map_yaml,
        }.items(),
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            slam_arg,
            map_yaml_arg,
            slam_toolbox_node,
            ekf_odom_node,
            ekf_map_node,
            nav2_bringup,
        ]
    )
