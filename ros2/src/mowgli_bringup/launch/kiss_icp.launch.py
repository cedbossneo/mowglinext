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


"""
kiss_icp.launch.py

Starts the LiDAR odometry pipeline:

  1. laserscan_to_pointcloud_node (pointcloud_to_laserscan pkg)
     - /scan (sensor_msgs/LaserScan, LD19 @ 10 Hz, 455 points)
     -> /scan/points (sensor_msgs/PointCloud2 in base_footprint frame)

  2. kiss_icp_node (kiss_icp pkg, upstream)
     - /scan/points -> /kiss_icp/odometry (nav_msgs/Odometry)
     - publish_odom_tf: False   (FusionCore owns odom->base_footprint;
                                 KISS-ICP MUST NOT publish TF here)

An external adapter consumes /kiss_icp/odometry and republishes it as
/encoder2/odom so FusionCore can fuse it as a wheel-odom-like source.
The adapter is NOT started here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    kiss_config = os.path.join(bringup_dir, "config", "kiss_icp.yaml")

    # ------------------------------------------------------------------
    # Arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # 1. LaserScan -> PointCloud2 converter
    #    target_frame=base_footprint so KISS-ICP receives points already
    #    expressed in the robot frame (matches its base_frame parameter).
    # ------------------------------------------------------------------
    scan_to_pc2 = Node(
        package="pointcloud_to_laserscan",
        executable="laserscan_to_pointcloud_node",
        name="laserscan_to_pointcloud",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_footprint",
                "transform_tolerance": 0.1,
            }
        ],
        remappings=[
            ("scan_in", "/scan"),
            ("cloud", "/scan/points"),
        ],
    )

    # ------------------------------------------------------------------
    # 2. KISS-ICP odometry node
    #    - input 'pointcloud_topic' remapped to /scan/points
    #    - output 'kiss/odometry' remapped to /kiss_icp/odometry
    #    - publish_odom_tf: False  (hard contract: FusionCore owns the TF)
    # ------------------------------------------------------------------
    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        parameters=[
            kiss_config,
            {
                "use_sim_time": use_sim_time,
                "base_frame": "base_footprint",
                # lidar_odom_frame is unused when publish_odom_tf=False,
                # but the upstream node still reads it at startup.
                "lidar_odom_frame": "odom_lidar",
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                "publish_debug_clouds": False,
                "position_covariance": 0.1,
                "orientation_covariance": 0.1,
            },
        ],
        remappings=[
            ("pointcloud_topic", "/scan/points"),
            ("kiss/odometry", "/kiss_icp/odometry"),
            ("kiss/frame", "/kiss_icp/frame"),
            ("kiss/keypoints", "/kiss_icp/keypoints"),
            ("kiss/local_map", "/kiss_icp/local_map"),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            scan_to_pc2,
            kiss_icp_node,
        ]
    )
