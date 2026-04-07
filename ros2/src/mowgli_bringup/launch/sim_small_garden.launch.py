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
sim_small_garden.launch.py

Small garden (10m x 8m) simulation for testing boundary enforcement
and coverage path adherence. Uses the small_garden world and a tighter
mowing boundary config.

Usage:
  ros2 launch mowgli_bringup sim_small_garden.launch.py
  ros2 launch mowgli_bringup sim_small_garden.launch.py headless:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")
    simulation_dir = get_package_share_directory("mowgli_simulation")
    behavior_dir = get_package_share_directory("mowgli_behavior")
    map_dir = get_package_share_directory("mowgli_map")
    coverage_dir = get_package_share_directory("mowgli_coverage_planner")
    monitoring_dir = get_package_share_directory("mowgli_monitoring")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="Run slam_toolbox when True.",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file.",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo in headless mode (no GUI).",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2.",
    )

    gps_degradation_arg = DeclareLaunchArgument(
        "simulate_gps_degradation",
        default_value="true",
        description="Enable GPS degradation simulation.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    simulate_gps_degradation = LaunchConfiguration("simulate_gps_degradation")

    # ------------------------------------------------------------------
    # Config paths — use small_garden behavior config
    # ------------------------------------------------------------------
    behavior_params = os.path.join(
        behavior_dir, "config", "behavior_tree_small_garden.yaml"
    )
    map_params = os.path.join(map_dir, "config", "map_server_small_garden.yaml")
    nav2_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    coverage_params = os.path.join(coverage_dir, "config", "coverage_planner.yaml")

    # ------------------------------------------------------------------
    # 1. Gazebo simulation — small_garden world
    # ------------------------------------------------------------------
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": "small_garden",
            "headless": headless,
            "use_rviz": use_rviz,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. Navigation stack — SLAM, dual EKF, Nav2
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam": slam,
            "map": map_yaml,
            "use_ekf": "True",
            "slam_mode": "lifelong",
            "map_file_name": "/ros2_ws/maps/small_garden_map",
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2b. Static map→odom TF when SLAM is disabled
    # ------------------------------------------------------------------
    static_map_odom_tf = Node(
        condition=UnlessCondition(slam),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 3. Behavior tree node — small garden config
    # ------------------------------------------------------------------
    behavior_tree_node = Node(
        package="mowgli_behavior",
        executable="behavior_tree_node",
        name="behavior_tree_node",
        output="screen",
        parameters=[
            behavior_params,
            {"use_sim_time": True},
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
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 5. Coverage planner (Fields2Cover v2)
    # ------------------------------------------------------------------
    coverage_planner_node = Node(
        package="mowgli_coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            coverage_params,
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 6. Diagnostics
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[
            monitoring_params,
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 7. Foxglove Bridge
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "use_sim_time": True,
                "send_buffer_limit": 10000000,
                "num_threads": 0,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 8. Obstacle tracker
    # ------------------------------------------------------------------
    obstacle_tracker_params = os.path.join(
        map_dir, "config", "obstacle_tracker.yaml"
    )

    obstacle_tracker_node = Node(
        package="mowgli_map",
        executable="obstacle_tracker_node",
        name="obstacle_tracker",
        output="screen",
        parameters=[
            obstacle_tracker_params,
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 9. NavSat → Pose converter
    # ------------------------------------------------------------------
    navsat_to_pose_node = Node(
        package="mowgli_simulation",
        executable="navsat_to_pose_node",
        name="navsat_to_pose_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "datum_lat": 0.0,
                "datum_lon": 0.0,
                "xy_covariance": 0.001,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 10. GPS degradation simulator
    # ------------------------------------------------------------------
    gps_degradation_node = Node(
        condition=IfCondition(simulate_gps_degradation),
        package="mowgli_simulation",
        executable="gps_degradation_sim_node",
        name="gps_degradation_sim_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "normal_duration_sec": 30.0,
                "degraded_duration_sec": 10.0,
                "degradation_covariance_scale": 100.0,
                "enable_position_drift": True,
                "drift_stddev": 0.5,
                "enabled": True,
            },
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            slam_arg,
            map_arg,
            headless_arg,
            use_rviz_arg,
            gps_degradation_arg,
            # Subsystem includes
            simulation_launch,
            navigation_launch,
            static_map_odom_tf,
            # Individual nodes
            behavior_tree_node,
            map_server_node,
            coverage_planner_node,
            obstacle_tracker_node,
            diagnostics_node,
            foxglove_bridge_node,
            navsat_to_pose_node,
            gps_degradation_node,
        ]
    )
