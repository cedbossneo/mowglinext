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
navigation.launch.py

Navigation stack launch file for the Mowgli robot mower.

Brings up:
  1. Cartographer           – online SLAM, TF authority for map → odom.
  2. FusionCore             – single UKF (GPS+IMU+wheels), TF authority for
                              odom → base_footprint.
  3. Nav2 bringup           – full navigation stack (controller, planner,
                              recoveries, BT navigator, costmaps, lifecycle).

Architecture:
  map → odom → base_footprint
  Cartographer publishes map→odom from LiDAR scan matching.
  FusionCore publishes odom→base_footprint from GPS+IMU+wheels.
  No GPS-SLAM corrector needed — GPS is fused directly in FusionCore.
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Run FusionCore. Set to False in simulation where Gazebo provides odom TF.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through). Also skips KISS-ICP LiDAR odometry.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ekf = LaunchConfiguration("use_ekf")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    localization_params = os.path.join(bringup_dir, "config", "localization.yaml")
    nav2_params_lidar = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    nav2_params_no_lidar = os.path.join(bringup_dir, "config", "nav2_params_no_lidar.yaml")
    # Select nav2 params based on use_lidar flag (resolved at launch time)
    nav2_params_file = PythonExpression([
        "'", nav2_params_lidar, "' if '",
        use_lidar, "'.lower() in ('true', '1') else '",
        nav2_params_no_lidar, "'",
    ])

    # Compute robot footprint from mowgli_robot.yaml so Nav2 costmaps
    # match the actual chassis shape regardless of mower model.
    robot_config_file = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    footprint_str = ""
    if os.path.isfile(robot_config_file):
        with open(robot_config_file, "r") as f:
            rcfg = yaml.safe_load(f) or {}
        rp = rcfg.get("mowgli", {}).get("ros__parameters", {})
        cl = float(rp.get("chassis_length", 0.54))
        cw = float(rp.get("chassis_width", 0.40))
        ccx = float(rp.get("chassis_center_x", 0.18))
        # Add 5cm margin to chassis footprint for costmap planning clearance
        margin = 0.05
        fp_f = ccx + cl / 2.0 + margin
        fp_r = ccx - cl / 2.0 - margin
        fp_hw = cw / 2.0 + margin
        footprint_str = (
            f"[[{fp_f:.3f}, {fp_hw:.3f}], "
            f"[{fp_f:.3f}, {-fp_hw:.3f}], "
            f"[{fp_r:.3f}, {-fp_hw:.3f}], "
            f"[{fp_r:.3f}, {fp_hw:.3f}]]"
        )

    # Read GPS lever arm from runtime config for FusionCore.
    gps_x = 0.0
    gps_y = 0.0
    gps_z = 0.0
    runtime_robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if os.path.isfile(runtime_robot_config):
        with open(runtime_robot_config, "r") as f:
            rt_cfg = yaml.safe_load(f) or {}
        rt_rp = rt_cfg.get("mowgli", {}).get("ros__parameters", {})
        gps_x = float(rt_rp.get("gps_x", 0.0))
        gps_y = float(rt_rp.get("gps_y", 0.0))
        gps_z = float(rt_rp.get("gps_z", 0.0))

    # Compute BT XML paths from installed package shares (not hardcoded).
    bt_nav_to_pose_xml = os.path.join(
        get_package_share_directory("mowgli_behavior"),
        "trees", "navigate_to_pose.xml",
    )
    bt_nav_through_poses_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees", "navigate_through_poses_w_replanning_and_recovery.xml",
    )

    # Rewrite use_sim_time, footprint, and BT XML paths throughout nav2_params.yaml.
    param_rewrites = {
        "use_sim_time": use_sim_time,
        "default_nav_to_pose_bt_xml": bt_nav_to_pose_xml,
        "default_nav_through_poses_bt_xml": bt_nav_through_poses_xml,
    }
    if footprint_str:
        param_rewrites["footprint"] = footprint_str

    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    # ------------------------------------------------------------------
    # 1. FusionCore — single UKF (GPS + IMU + wheels[+ LiDAR twist later])
    #    Publishes odom → base_footprint TF. GPS-RTK anchored.
    # ------------------------------------------------------------------
    fusioncore_node = LifecycleNode(
        condition=IfCondition(use_ekf),
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore_node",
        namespace="",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
            # GPS lever arm from mowgli_robot.yaml (gps_x/y/z)
            {"gnss.lever_arm_x": gps_x},
            {"gnss.lever_arm_y": gps_y},
            {"gnss.lever_arm_z": gps_z},
        ],
        remappings=[
            ("/odom/wheels", "/wheel_odom"),
            ("/gnss/fix", "/gps/fix"),
        ],
    )

    # Auto-configure and activate the lifecycle node after startup
    fusioncore_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == fusioncore_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    fusioncore_start = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == fusioncore_node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ),
        ],
    )

    # ------------------------------------------------------------------
    # 2. Static map→odom identity
    #    FusionCore's odom is GPS-RTK anchored (σ ~3mm when Fixed), so the
    #    map frame IS the GPS ENU frame. No SLAM correction needed when
    #    GPS is healthy. During GPS degradation, KISS-ICP publishes
    #    corrections to map→odom (see kiss_icp.launch.py, gated on use_lidar).
    # ------------------------------------------------------------------
    static_map_odom_tf = Node(
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 3. KISS-ICP LiDAR odometry (scan -> PointCloud2 -> /kiss_icp/odometry)
    #    Gated entirely on use_lidar. KISS-ICP does NOT publish TF;
    #    an external adapter republishes /kiss_icp/odometry as /encoder2/odom
    #    so FusionCore can fuse it as a wheel-odom-like source.
    # ------------------------------------------------------------------
    kiss_icp_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "kiss_icp.launch.py")
        ),
        condition=IfCondition(use_lidar),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Adapter: /kiss_icp/odometry (pose-only) -> /encoder2/odom (twist-only).
    # Finite-differences KISS-ICP pose into body-frame twist and advertises
    # tight covariance when ICP is healthy / inflated covariance when the
    # pose-covariance proxy indicates a bad match. FusionCore fuses this as
    # a second encoder source. See scripts/kiss_icp_encoder_adapter.py.
    kiss_icp_encoder_adapter = Node(
        condition=IfCondition(use_lidar),
        package="mowgli_localization",
        executable="kiss_icp_encoder_adapter.py",
        name="kiss_icp_encoder_adapter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 4. Nav2 navigation (controllers, planners, behaviors, BT navigator)
    # ------------------------------------------------------------------
    # Gate Nav2 startup on the map→odom TF being available.
    wait_for_tf_script = os.path.join(
        get_package_prefix("mowgli_bringup"),
        "lib", "mowgli_bringup", "wait_for_tf.py"
    )

    wait_for_map_odom_tf = ExecuteProcess(
        cmd=[
            "python3", wait_for_tf_script,
            "--parent", "map",
            "--child", "odom",
            "--timeout", "120",
        ],
        name="wait_for_map_odom_tf",
        output="screen",
    )

    nav2_navigation_group = GroupAction(
        actions=[
            SetParameter("bond_timeout", 10.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_dir, "launch", "nav2_navigation_launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )

    # Launch Nav2 only after the map→odom TF is available
    nav2_after_tf = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_odom_tf,
            on_exit=[nav2_navigation_group],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ekf_arg,
            use_lidar_arg,
            static_map_odom_tf,
            fusioncore_node,
            fusioncore_configure,
            fusioncore_start,
            kiss_icp_group,
            kiss_icp_encoder_adapter,
            wait_for_map_odom_tf,
            nav2_after_tf,
        ]
    )
