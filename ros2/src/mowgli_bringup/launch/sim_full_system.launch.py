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
sim_full_system.launch.py

Simulation full system launch for the Mowgli robot mower.

Combines the Gazebo simulation environment with the full navigation and
behavior stack, using simulated time throughout.

Brings up:
  1. mowgli_simulation/launch/simulation.launch.py — Gazebo world + spawned robot
  2. navigation.launch.py                          — robot_localization (dual EKF), Nav2
  3. Behavior tree node                             — mowgli_behavior
  4. Map server                                     — mowgli_map
  5. Coverage server                                — opennav_coverage
  6. Diagnostics                                    — mowgli_monitoring
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
    simulation_dir = get_package_share_directory("mowgli_simulation")
    behavior_dir = get_package_share_directory("mowgli_behavior")
    map_dir = get_package_share_directory("mowgli_map")
    monitoring_dir = get_package_share_directory("mowgli_monitoring")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="garden",
        description="Gazebo world name (garden, empty_garden) or path to SDF.",
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
        description="Enable GPS degradation simulation (periodic float mode).",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Enable LiDAR-dependent nodes (obstacle tracker, Kinematic-ICP). Set to false for GPS-only.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # use_sim_time is always true in simulation — no argument needed.
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    simulate_gps_degradation = LaunchConfiguration("simulate_gps_degradation")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    nav2_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    # ------------------------------------------------------------------
    # 1. Gazebo simulation — world + spawned robot
    # ------------------------------------------------------------------
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": world,
            "headless": headless,
            "use_rviz": use_rviz,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 1b. Topic relays: simulation topics → hardware namespace
    #     The EKF config (localization.yaml) expects /mowgli/hardware/*
    #     topics that come from hardware_bridge on real hardware. In sim,
    #     the Gazebo bridge publishes /wheel_odom and /imu/data directly.
    # ------------------------------------------------------------------
    relay_wheel_odom = Node(
        package="topic_tools",
        executable="relay",
        name="relay_wheel_odom",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/wheel_odom", "/mowgli/hardware/wheel_odom"],
    )

    relay_imu = Node(
        package="topic_tools",
        executable="relay",
        name="relay_imu",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/imu/data", "/mowgli/hardware/imu"],
    )

    # ------------------------------------------------------------------
    # 2. Navigation stack — robot_localization (dual EKF), Nav2
    #    ekf_odom_node publishes odom -> base_footprint; ekf_map_node
    #    publishes map -> odom. Kinematic-ICP drift correction is
    #    real-robot only (requires LiDAR on ARM); sim runs open-loop
    #    GPS-only for now.
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ekf": "True",
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
    # 5. Diagnostics
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
    # 7. Foxglove Bridge — binary WebSocket bridge for Foxglove Studio
    #    Connect via: ws://localhost:8765 (Foxglove WebSocket protocol)
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

    # NOTE: docking_server is launched and lifecycle-managed by Nav2's
    # navigation_launch.py (in the lifecycle_nodes list). Do NOT launch
    # it here — duplicating it exhausts DDS participants and causes
    # lifecycle conflicts.

    # ------------------------------------------------------------------
    # 8. Obstacle tracker — persistent LiDAR obstacle detection
    # ------------------------------------------------------------------
    obstacle_tracker_params = os.path.join(map_dir, "config", "obstacle_tracker.yaml")

    obstacle_tracker_node = Node(
        condition=IfCondition(use_lidar),
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
    # 9. GPS notes
    #    navsat_transform_node takes /gps/fix (NavSatFix) directly from Gazebo.
    #    navsat_to_pose_node is no longer needed.
    #    GPS degradation simulator needs rewriting to intercept NavSatFix
    #    instead of PoseWithCovarianceStamped — disabled for now.
    # TODO: rewrite gps_degradation_sim_node to modify NavSatFix messages
    #       (inflate covariance, inject position drift on /gps/fix).
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # 10. Fake hardware bridge — stub services/topics for simulation
    # ------------------------------------------------------------------
    fake_hardware_bridge_node = Node(
        package="mowgli_simulation",
        executable="fake_hardware_bridge_node",
        name="fake_hardware_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 11. Sim NavSat RTK status promoter
    #     Gazebo emits NavSatFix with STATUS_FIX (0); production code
    #     (navsat_to_absolute_pose_node) requires
    #     STATUS_GBAS_FIX (2) for the GPS path. The ros_gz_bridge now publishes Gazebo's
    #     output on /gps/fix_raw; this relay rewrites status -> GBAS_FIX
    #     and republishes on /gps/fix with a realistic RTK-Fixed
    #     covariance (sigma ~3 mm).
    # ------------------------------------------------------------------
    sim_navsat_rtk_fix_node = Node(
        package="mowgli_simulation",
        executable="sim_navsat_rtk_fix.py",
        name="sim_navsat_rtk_fix",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/gps/fix_raw",
                "output_topic": "/gps/fix",
                # Realistic mowing scenario: 90 s RTK-Fixed (open sky),
                # 30 s RTK-Float (light tree cover), 10 s no-fix (dense
                # canopy / multipath). Set to "" for always-FIXED.
                "quality_pattern": "90,RTK_FIXED;30,RTK_FLOAT;10,NO_FIX",
                "noise_seed": 42,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 11.5 NavSat -> AbsolutePose converter (production node, but full_system
    #      .launch.py launches it directly rather than via navigation.launch.py
    #      so the sim path needs its own copy). Reads /gps/fix and publishes
    #      /gps/pose_cov (PoseWithCovarianceStamped in map frame) which
    #      ekf_map_node fuses as pose0. Without this, no GPS reaches the EKF
    #      in sim and the BT cannot transition out of IDLE.
    #
    #      Datum matches garden.sdf <spherical_coordinates>; if you change
    #      the sim world's lat/lon, change these too.
    # ------------------------------------------------------------------
    sim_localization_params = os.path.join(
        bringup_dir, "config", "robot_localization.yaml"
    )
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            sim_localization_params,
            {
                "use_sim_time": True,
                "datum_lat": 48.137154,
                "datum_lon": 11.576124,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 12. Sim IMU noise injector
    #     Adds gyro/accel bias-random-walk + white noise to Gazebo's
    #     perfect IMU stream (/imu/data_gz from bridge) and republishes
    #     on /imu/data with realistic MEMS noise. Set all *_white_std and
    #     *_walk_std parameters to 0 for a noiseless A/B baseline.
    # ------------------------------------------------------------------
    sim_imu_noise_node = Node(
        package="mowgli_simulation",
        executable="sim_imu_noise.py",
        name="sim_imu_noise",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/imu/data_gz",
                "output_topic": "/imu/data",
                # MPU-9250 / LIS6DSL-class MEMS defaults.
                "gyro_white_std": 0.005,
                "gyro_bias_walk_std": 1.0e-4,
                "gyro_bias_init_std": 1.0e-3,
                "accel_white_std": 0.05,
                "accel_bias_walk_std": 1.0e-3,
                "accel_bias_init_std": 0.05,
                "noise_seed": 42,
            }
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            world_arg,
            headless_arg,
            use_rviz_arg,
            gps_degradation_arg,
            use_lidar_arg,
            # Topic relays (sim → hardware namespace)
            relay_wheel_odom,
            relay_imu,
            # Subsystem includes
            simulation_launch,
            navigation_launch,
            # Individual nodes
            fake_hardware_bridge_node,
            sim_navsat_rtk_fix_node,
            navsat_converter_node,
            sim_imu_noise_node,
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            diagnostics_node,
            foxglove_bridge_node,
        ]
    )
