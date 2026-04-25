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
kinematic_icp.launch.py

Kinematic-ICP on a parallel, fully-decoupled TF tree so there is NO
feedback loop from its output back into its own motion prior.

    Real tree (owned by robot_localization + URDF):
        map -> odom -> base_footprint -> base_link -> lidar_link

    Parallel tree (K-ICP only):
        wheel_odom_raw  ->  base_footprint_wheels  ->  lidar_link_wheels

Data flow:

  1. wheel_odom_tf_node (mowgli_localization)
     - Subscribes /wheel_odom, integrates twist, broadcasts TF
       `wheel_odom_raw -> base_footprint_wheels`. Independent of the
       main EKF tree.

  2. kinematic_icp_scan_frame_relay (mowgli_localization)
     - On startup, waits for the URDF's real `base_footprint -> lidar_link`
       and mirrors it as a static TF `base_footprint_wheels -> lidar_link_wheels`
       so the parallel tree has the sensor extrinsic. Then republishes /scan
       on /scan_kicp with `header.frame_id` rewritten to `lidar_link_wheels`.
     - Both frames live on the parallel tree only, so K-ICP's TF lookups
       never touch the fused state.

  3. kinematic_icp_online_node (kinematic_icp pkg, upstream)
     - lidar_topic = /scan_kicp (relayed), use_2d_lidar = true
     - wheel_odom_frame = wheel_odom_raw (motion prior, raw wheels)
     - base_frame       = base_footprint_wheels (sensor extrinsic origin)
     - publish_odom_tf  = false (ekf_odom_node owns odom -> base_footprint)
     -> /kinematic_icp/lidar_odometry

  4. kinematic_icp_encoder_adapter (mowgli_localization)
     - Finite-differences /kinematic_icp/lidar_odometry pose into body-
       frame twist, publishes /encoder2/odom which ekf_odom_node fuses
       as odom1.

Because K-ICP's prior comes from a wheel-only node and its sensor TF is
a static snapshot, nothing in the fused state ever influences K-ICP's
input. The adapter's encoder2 output feeds ekf_odom_node, but there
is no return path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    kicp_config = os.path.join(bringup_dir, "config", "kinematic_icp.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # 1. Wheel-only TF publisher (raw /wheel_odom integrated on isolated
    #    frames — independent of the main EKF tree).
    # ------------------------------------------------------------------
    wheel_odom_tf = Node(
        package="mowgli_localization",
        executable="wheel_odom_tf_node.py",
        name="wheel_odom_tf_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/wheel_odom",
                "parent_frame": "wheel_odom_raw",
                "child_frame": "base_footprint_wheels",
                # 15 Hz: K-ICP consumes scans at 10 Hz; 1.5× headroom is
                # plenty. Lower rate also matches the 0.3 m/s robot speed —
                # at 15 Hz the robot moves 2 cm per TF tick.
                "rebroadcast_hz": 15.0,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 2. Scan-frame relay: mirrors the URDF sensor extrinsic onto the
    #    parallel tree and republishes /scan with the mirrored frame_id.
    # ------------------------------------------------------------------
    scan_relay = Node(
        package="mowgli_localization",
        executable="kinematic_icp_scan_frame_relay.py",
        name="kinematic_icp_scan_frame_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan",
                "output_topic": "/scan_kicp",
                "input_sensor_frame": "lidar_link",
                "output_sensor_frame": "lidar_link_wheels",
                "real_base_frame": "base_footprint",
                "wheels_base_frame": "base_footprint_wheels",
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. Kinematic-ICP online node (reads the relayed scan + parallel tree).
    # ------------------------------------------------------------------
    kinematic_icp_node = Node(
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="kinematic_icp_online_node",
        namespace="kinematic_icp",
        output="screen",
        parameters=[
            kicp_config,
            {
                "use_sim_time": use_sim_time,
                "lidar_topic": "/scan_kicp",
                "use_2d_lidar": True,
                "wheel_odom_frame": "wheel_odom_raw",
                "base_frame": "base_footprint_wheels",
                # Internal frame only; nothing else consumes it.
                "lidar_odom_frame": "lidar_odom_kicp",
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                # Small timeout so the scan-end TF lookup can wait for the
                # next 50 Hz wheel-TF rebroadcast instead of failing.
                "tf_timeout": 0.1,
            },
        ],
        remappings=[
            ("lidar_odometry", "/kinematic_icp/lidar_odometry"),
        ],
    )

    # ------------------------------------------------------------------
    # 4. Encoder-twist adapter: K-ICP Odometry -> ekf_odom_node odom1.
    # ------------------------------------------------------------------
    kicp_encoder_adapter = Node(
        package="mowgli_localization",
        executable="kinematic_icp_encoder_adapter.py",
        name="kinematic_icp_encoder_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/kinematic_icp/lidar_odometry",
                "output_topic": "/encoder2/odom",
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_odom_tf,
            scan_relay,
            kinematic_icp_node,
            kicp_encoder_adapter,
        ]
    )
