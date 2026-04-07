"""
sim_full_system.launch.py

Simulation full system launch for the Mowgli robot mower.

Combines the Gazebo simulation environment with the full navigation and
behavior stack, using simulated time throughout.

Brings up:
  1. mowgli_simulation/launch/simulation.launch.py — Gazebo world + spawned robot
  2. navigation.launch.py                          — SLAM, dual EKF, Nav2
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
    TimerAction,
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
        description="Run slam_toolbox when True; skip when using a pre-built map.",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file (used when slam=false).",
    )

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
        description="Enable LiDAR-dependent nodes (SLAM, obstacle tracker). Set to false for GPS-only.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # use_sim_time is always true in simulation — no argument needed.
    # ------------------------------------------------------------------
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
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
    coverage_params = os.path.join(coverage_dir, "config", "coverage_planner.yaml")

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
            "map_file_name": "/ros2_ws/maps/garden_map",
            "use_lidar": use_lidar,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2b. Static map→odom TF (identity) for simulation without SLAM
    #     When SLAM is enabled, slam_toolbox provides the map→odom TF.
    #     When SLAM is disabled, we publish a static identity transform.
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
    # 9. NavSat → Pose converter — bridges Gazebo NavSatFix to the
    #    PoseWithCovarianceStamped expected by the EKF's pose0 input.
    #    On real hardware, gps_pose_converter handles this from AbsolutePose.
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
    # 9. GPS degradation simulator — periodically degrades GPS to float
    #    mode so LiDAR/odometry must compensate. Subscribes to /gps/pose
    #    and republishes on /gps/pose_degraded with inflated covariance.
    #    The EKF is remapped to use /gps/pose_degraded when this is active.
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
        # The node subscribes to /gps/pose and publishes to /gps/pose_sim
        # by default (hardcoded topics). No remapping needed.
    )

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
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            slam_arg,
            map_arg,
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
            static_map_odom_tf,
            # Individual nodes
            fake_hardware_bridge_node,
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
