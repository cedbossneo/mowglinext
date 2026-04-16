# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""
rtabmap.launch.py

Launches RTAB-Map as the SLAM backend in 2D LiDAR mode.

Publishes map→odom TF (replacing slam_toolbox).
FusionCore publishes odom→base_footprint with GPS enabled.
No GPS-SLAM corrector needed — GPS is fused directly in FusionCore.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # RTAB-Map in 2D LiDAR-only mode (no camera).
    # Subscribes to /scan and /fusion/odom, publishes map→odom TF.
    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,

            # Frame configuration
            "frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan": True,
            "subscribe_scan_cloud": False,
            "subscribe_odom_info": False,
            "approx_sync": True,

            # 2D SLAM mode
            "Reg/Strategy": "1",             # ICP registration
            "Reg/Force3DoF": "true",         # 2D mode (no z, roll, pitch)
            "RGBD/ProximityBySpace": "true",
            "RGBD/NeighborLinkRefining": "true",
            # Don't add new node unless robot has moved meaningfully:
            # prevents map graph bloat when stationary on dock.
            "RGBD/AngularUpdate": "0.1",     # ~6 degrees
            "RGBD/LinearUpdate": "0.1",      # 10cm
            "RGBD/OptimizeFromGraphEnd": "false",

            # ICP parameters for 2D LiDAR
            "Icp/VoxelSize": "0.05",
            "Icp/MaxCorrespondenceDistance": "0.15",
            "Icp/PointToPlane": "false",     # 2D scan — no planes
            "Icp/Iterations": "30",

            # Memory / loop closure
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true",
            "Mem/STMSize": "30",
            # Skip identical scans (graph dedup) to limit DB growth
            "Mem/RehearsalSimilarity": "0.6",

            # Optimizer
            "Optimizer/Strategy": "1",       # g2o
            "Optimizer/Iterations": "20",
            "Optimizer/Slam2D": "true",

            # Grid map for costmap
            "Grid/FromDepth": "false",       # Use scan, not depth
            "Grid/Sensor": "0",              # 0=scan, 1=depth (silences warning)
            "Grid/RangeMax": "8.0",
            "Grid/RangeMin": "0.2",          # Filter grass
            "Grid/CellSize": "0.05",
            "Grid/3D": "false",
            "Grid/RayTracing": "true",       # Mark free space behind obstacles

            # Disable octomap (we're 2D — silences "octomap empty" spam)
            "octomap_enabled": False,

            # Database
            "database_path": "/ros2_ws/maps/rtabmap.db",

            # Publish
            "publish_tf": True,              # map→odom TF
            "tf_delay": 0.05,               # 20 Hz TF publish
        }],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/fusion/odom"),
            ("imu", "/imu/data"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_slam_node,
    ])
