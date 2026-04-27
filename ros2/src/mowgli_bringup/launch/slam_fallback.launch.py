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
slam_fallback.launch.py

slam_toolbox-based RTK fallback localization on a fully-decoupled
parallel TF tree, replacing the previous Kinematic-ICP integration.

    Real EKF tree (owned by robot_localization + URDF):
        map -> odom -> base_footprint -> base_link -> lidar_link

    Parallel tree (slam_toolbox + slam_pose_anchor_node):
        map -> map_slam (slam_pose_anchor_node, EWMA-aligned to GPS)
                  └─> wheel_odom_raw (slam_toolbox publishes
                                       map_slam->wheel_odom_raw,
                                       wheel_odom_tf_node publishes
                                       wheel_odom_raw->base_footprint_wheels)
                          └─> base_footprint_wheels
                                  └─> lidar_link_wheels

Composes four nodes:

  1. wheel_odom_tf_node — integrates raw /wheel_odom into the parallel
     wheel_odom_raw -> base_footprint_wheels TF (motion prior for
     slam_toolbox, identical to the K-ICP-era role).

  2. slam_scan_frame_relay — mirrors the URDF lidar extrinsic onto the
     parallel tree as base_footprint_wheels -> lidar_link_wheels and
     republishes /scan as /scan_slam with the matching frame_id.

  3. slam_toolbox/async_slam_toolbox_node — runs in lifelong mode if a
     persisted map exists at /ros2_ws/maps/slam.posegraph (auto-loaded),
     otherwise mapping mode (builds from scratch). Either way, publishes
     map_slam -> wheel_odom_raw at ~20 Hz.

  4. slam_pose_anchor_node — EWMA-aligns map_slam to the GPS-anchored
     map frame on every RTK-Fixed sample, broadcasts map -> map_slam,
     and republishes the slam pose as /slam/pose_cov (PoseWithCov,
     frame=map) for ekf_map_node to fuse as pose1. Always publishes;
     covariance grows with anchor age so the EKF naturally weights GPS
     above slam when RTK is healthy and slam above GPS during dropouts.

Mode swap at launch time
------------------------
slam_toolbox's `mode: lifelong` requires a deserialisable map at
map_file_name; if the file is absent (very first session), we override
the mode to `mapping` so the node starts with an empty graph. The
companion serialize-on-RECORD_FINISH helper writes the map; subsequent
boots find it and stay in lifelong.

Decoupling guarantees
---------------------
slam_toolbox's motion-prior TF (wheel_odom_raw -> base_footprint_wheels)
is fed by raw wheel odometry only; it never depends on the fused EKF
state, so /slam/pose_cov can never feed back into its own input. This
is the same isolation argument K-ICP used; we just replaced the
LiDAR-odometry node and added the GPS-anchor mechanism.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Path of the persisted slam_toolbox map (sans extension — slam_toolbox
# writes <map_file_name>.posegraph and <map_file_name>.data). Kept in
# /ros2_ws/maps alongside areas.dat, dock_calibration.yaml, and
# mag_calibration.yaml, all of which live on the install_mowgli_maps
# Docker volume so they persist across container rebuilds.
SLAM_MAP_PATH = "/ros2_ws/maps/slam"


def _select_slam_mode() -> str:
    """Return 'lifelong' if a persisted map exists, else 'mapping'.

    slam_toolbox's lifelong mode requires the map file to exist at
    startup; without it we'd crash. The serialize-on-RECORD_FINISH
    helper writes the map after the first area is recorded, so from
    the second session onward this returns 'lifelong'.
    """
    return (
        "lifelong" if os.path.exists(SLAM_MAP_PATH + ".posegraph") else "mapping"
    )


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    slam_config = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # 1. Wheel-only TF publisher — wheel_odom_raw -> base_footprint_wheels.
    #    Reused unchanged from the K-ICP era; same role here as motion
    #    prior for slam_toolbox.
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
                # 30 Hz: matches the cadence slam_toolbox's transform
                # lookups expect at scan-end timestamps. See K-ICP-era
                # rationale in wheel_odom_tf_node.py.
                "rebroadcast_hz": 30.0,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 2. Scan-frame relay: base_footprint_wheels -> lidar_link_wheels
    #    static TF + /scan -> /scan_slam republish with rewritten
    #    frame_id.
    # ------------------------------------------------------------------
    scan_relay = Node(
        package="mowgli_localization",
        executable="slam_scan_frame_relay.py",
        name="slam_scan_frame_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan",
                "output_topic": "/scan_slam",
                "input_sensor_frame": "lidar_link",
                "output_sensor_frame": "lidar_link_wheels",
                "real_base_frame": "base_footprint",
                "wheels_base_frame": "base_footprint_wheels",
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. slam_toolbox async node — runs on the parallel tree.
    # ------------------------------------------------------------------
    slam_mode = _select_slam_mode()
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            slam_config,
            {
                "use_sim_time": use_sim_time,
                # Override the YAML's `mode: lifelong` if no map exists
                # yet — slam_toolbox would otherwise crash trying to
                # deserialise a missing file. After the first
                # serialize-on-RECORD_FINISH, the file appears and the
                # next session's launch picks lifelong.
                "mode": slam_mode,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 4. slam_pose_anchor_node — EWMA-aligns map_slam to map and
    #    republishes /slam/pose_cov for ekf_map_node.
    # ------------------------------------------------------------------
    slam_pose_anchor = Node(
        package="mowgli_localization",
        executable="slam_pose_anchor_node.py",
        name="slam_pose_anchor_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "gps_map_frame": "map",
                "gps_base_frame": "base_footprint",
                "slam_map_frame": "map_slam",
                "slam_base_frame": "base_footprint_wheels",
                # 5 Hz: enough to keep the EKF fed and the anchor
                # smooth without competing with slam_toolbox's TF rate.
                "tick_hz": 5.0,
                "ewma_alpha": 0.05,
                "ekf_cov_threshold": 1e-2,
                "sigma_xy_floor": 0.10,
                "sigma_xy_drift_rate": 0.01,
                "sigma_xy_cap": 5.0,
                "yaw_var_unused": 1.0e3,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 5. slam_map_persist_node — checkpoints slam_toolbox's posegraph to
    #    /ros2_ws/maps/slam.{posegraph,data} on every transition out of
    #    HIGH_LEVEL_STATE_RECORDING. Subsequent boots auto-load this
    #    checkpoint and run lifelong refinement on it.
    # ------------------------------------------------------------------
    slam_map_persist = Node(
        package="mowgli_localization",
        executable="slam_map_persist_node.py",
        name="slam_map_persist_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_path": SLAM_MAP_PATH,
                "settle_delay_sec": 1.0,
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_odom_tf,
            scan_relay,
            slam_node,
            slam_pose_anchor,
            slam_map_persist,
        ]
    )
