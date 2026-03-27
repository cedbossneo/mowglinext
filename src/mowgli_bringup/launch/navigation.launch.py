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
        default_value="True",
        description="Run slam_toolbox when True; skip when using a pre-built map.",
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file (used when slam=false).",
    )

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Run dual EKF nodes. Set to False in simulation where Gazebo provides odom TF.",
    )

    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="mapping",
        description="slam_toolbox mode: 'mapping' (first run) or 'localization' (subsequent runs with saved map).",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file_name",
        default_value="",
        description="Full path (without extension) to a saved slam_toolbox .posegraph/.data file for localization mode.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    use_ekf = LaunchConfiguration("use_ekf")
    slam_mode = LaunchConfiguration("slam_mode")
    map_file_name = LaunchConfiguration("map_file_name")

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
            {
                "use_sim_time": use_sim_time,
                "mode": slam_mode,
                "map_file_name": map_file_name,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 2. robot_localization – odom EKF
    # ------------------------------------------------------------------
    ekf_odom_node = Node(
        condition=IfCondition(use_ekf),
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
        condition=IfCondition(use_ekf),
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
    # 4. Nav2 navigation (controllers, planners, behaviors, BT navigator)
    #    We use navigation_launch.py directly instead of bringup_launch.py
    #    because bringup_launch.py also starts localization (AMCL) which
    #    would fight with our slam_toolbox over the map→odom TF.
    # ------------------------------------------------------------------
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
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
            use_ekf_arg,
            slam_mode_arg,
            map_file_arg,
            slam_toolbox_node,
            ekf_odom_node,
            ekf_map_node,
            nav2_navigation,
        ]
    )
