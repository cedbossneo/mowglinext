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
  1. Static identity map -> odom (published once; the odom frame IS the
     GPS-ENU frame so map and odom are literally the same).
  2. FusionCore (UKF: GPS + IMU + wheels, optionally blended with Kinematic-ICP
     on /encoder2/odom) owns odom -> base_footprint at 50 Hz.
  3. Kinematic-ICP (optional, gated on use_lidar) runs on a decoupled parallel
     TF tree (wheel_odom_raw -> base_footprint_wheels -> lidar_link_wheels)
     and feeds FusionCore via the encoder2 adapter.
  4. Nav2 bringup — full navigation stack (controllers, planners, recoveries,
     BT navigator, costmaps, lifecycle).

Architecture (REP-105):
  map == odom (static identity) → base_footprint (FusionCore) → base_link → sensors
  There is no SLAM. GPS is fused directly by FusionCore; Kinematic-ICP provides
  LiDAR-derived twist on encoder2 to shore up dead-reckoning during GPS
  degradation.
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
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through). Also skips Kinematic-ICP LiDAR odometry.",
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

    # Read GPS + IMU lever arms and the dock pose from runtime config.
    # Lever arms act as explicit overrides for FusionCore; when zero, it
    # auto-resolves them from TF (base_footprint -> sensor frame).
    # Dock pose is fed into docking_server's home_dock.pose below.
    gps_x = 0.0
    gps_y = 0.0
    gps_z = 0.0
    imu_lever_x = 0.0
    imu_lever_y = 0.0
    imu_lever_z = 0.0
    dock_pose_x = 0.0
    dock_pose_y = 0.0
    dock_pose_yaw = 0.0
    runtime_robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if os.path.isfile(runtime_robot_config):
        with open(runtime_robot_config, "r") as f:
            rt_cfg = yaml.safe_load(f) or {}
        rt_rp = rt_cfg.get("mowgli", {}).get("ros__parameters", {})
        gps_x = float(rt_rp.get("gps_x", 0.0))
        gps_y = float(rt_rp.get("gps_y", 0.0))
        gps_z = float(rt_rp.get("gps_z", 0.0))
        imu_lever_x = float(rt_rp.get("imu_x", 0.0))
        imu_lever_y = float(rt_rp.get("imu_y", 0.0))
        imu_lever_z = float(rt_rp.get("imu_z", 0.0))
        dock_pose_x = float(rt_rp.get("dock_pose_x", 0.0))
        dock_pose_y = float(rt_rp.get("dock_pose_y", 0.0))
        dock_pose_yaw = float(rt_rp.get("dock_pose_yaw", 0.0))

    # Compute BT XML paths from installed package shares (not hardcoded).
    bt_nav_to_pose_xml = os.path.join(
        get_package_share_directory("mowgli_behavior"),
        "trees", "navigate_to_pose.xml",
    )
    bt_nav_through_poses_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees", "navigate_through_poses_w_replanning_and_recovery.xml",
    )

    # opennav_docking declares home_dock.pose as PARAMETER_DOUBLE_ARRAY (see
    # opennav_docking/utils.hpp::parseDockParams). Nav2's RewrittenYaml can
    # only substitute scalar values; passing a stringified list "[x, y, yaw]"
    # ends up as a STRING parameter and the node rejects it with
    # "Dock home_dock has no valid 'pose'".
    #
    # So we preprocess both nav2 yaml files here — load with yaml.safe_load,
    # write the dock pose as a native list, dump to a tmp file — and hand
    # those tmp files to RewrittenYaml as its sources. RewrittenYaml then
    # handles the remaining scalar rewrites (use_sim_time, footprint, BT XML
    # paths) without touching the pose list.
    def _inject_dock_pose(src_path: str) -> str:
        import tempfile
        with open(src_path, "r") as fh:
            doc = yaml.safe_load(fh) or {}
        (doc.setdefault("docking_server", {})
            .setdefault("ros__parameters", {})
            .setdefault("home_dock", {}))["pose"] = [dock_pose_x, dock_pose_y, dock_pose_yaw]
        tmp = tempfile.NamedTemporaryFile(
            mode="w", prefix="mowgli_nav2_", suffix=".yaml", delete=False)
        yaml.safe_dump(doc, tmp, default_flow_style=False, sort_keys=False)
        tmp.close()
        return tmp.name

    nav2_params_lidar = _inject_dock_pose(nav2_params_lidar)
    nav2_params_no_lidar = _inject_dock_pose(nav2_params_no_lidar)
    nav2_params_file = PythonExpression([
        "'", nav2_params_lidar, "' if '",
        use_lidar, "'.lower() in ('true', '1') else '",
        nav2_params_no_lidar, "'",
    ])

    # Rewrite use_sim_time, footprint, and BT XML paths throughout nav2_params.yaml.
    # (home_dock.pose is NOT in this dict — it's injected as a proper YAML
    # list by _inject_dock_pose above; RewrittenYaml can only do scalar
    # substitutions.)
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
            # IMU lever arm from mowgli_robot.yaml (imu_x/y/z). 0 means
            # "let FusionCore auto-resolve from the TF base_footprint ->
            # imu_link translation"; non-zero overrides that.
            {"imu.lever_arm_x": imu_lever_x},
            {"imu.lever_arm_y": imu_lever_y},
            {"imu.lever_arm_z": imu_lever_z},
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
    #    GPS is healthy. During GPS degradation, Kinematic-ICP shores up
    #    dead-reckoning by feeding FusionCore's encoder2 slot (see
    #    kinematic_icp.launch.py, gated on use_lidar).
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
    # 3. Kinematic-ICP LiDAR odometry
    #    Launched by kinematic_icp.launch.py: wheel_odom_tf_node +
    #    kinematic_icp_online_node + kinematic_icp_encoder_adapter.
    #    Gated entirely on use_lidar. Kinematic-ICP does NOT publish the
    #    odom TF; the adapter republishes /kinematic_icp/lidar_odometry as
    #    /encoder2/odom so FusionCore can fuse it as a wheel-odom-like
    #    source, with the kinematic prior enforcing non-holonomic motion.
    # ------------------------------------------------------------------
    kinematic_icp_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "kinematic_icp.launch.py")
        ),
        condition=IfCondition(use_lidar),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
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
            kinematic_icp_group,
            wait_for_map_odom_tf,
            nav2_after_tf,
        ]
    )
