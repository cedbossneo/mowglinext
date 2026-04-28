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
  1. robot_localization dual-EKF — ekf_odom (wheels + gyro → odom→base_footprint
     TF, continuous) and ekf_map (+ GPS via navsat_transform → map→odom
     correction). Sub-cm σ_xy under RTK-Fixed.
  2. Three helper nodes — dock_yaw_to_set_pose (seeds both EKFs with the dock
     heading on the is_charging rising edge), cog_to_imu (GPS COG as a
     continuous absolute-yaw observation with adaptive covariance), and
     mag_yaw_publisher (tilt-compensated LIS3MDL magnetometer yaw, gated on
     /ros2_ws/maps/mag_calibration.yaml existing).
  3. Nav2 bringup — full navigation stack (controllers, planners, recoveries,
     BT navigator, costmaps, lifecycle).

Architecture (REP-105):
  map → (ekf_map) → odom → (ekf_odom) → base_footprint → base_link → sensors
  ekf_map fuses /gps/pose_cov (from navsat_to_absolute_pose_node, datum +
  lever-arm corrected) as pose0. The slam_toolbox RTK fallback that
  previously fed /map_pose has been removed; fusion_graph (planned, see
  docs/HANDOFF_FUSION_GRAPH.md) will replace ekf_map_node end-to-end.
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
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
        description="Run the robot_localization dual EKF. Set to False in simulation where Gazebo provides the odom TF directly.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through).",
    )

    use_fusion_graph_arg = DeclareLaunchArgument(
        "use_fusion_graph",
        default_value="false",
        description="When true, replace ekf_map_node with fusion_graph_node (GTSAM factor-graph localizer). ekf_odom_node keeps publishing odom->base_footprint either way.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ekf = LaunchConfiguration("use_ekf")
    use_lidar = LaunchConfiguration("use_lidar")
    use_fusion_graph = LaunchConfiguration("use_fusion_graph")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    nav2_params_lidar = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    nav2_params_no_lidar = os.path.join(bringup_dir, "config", "nav2_params_no_lidar.yaml")

    # Compute robot footprint from mowgli_robot.yaml so Nav2 costmaps
    # match the actual chassis shape regardless of mower model. Prefer
    # the runtime config (install/, mounted at /ros2_ws/config) which
    # reflects the operator-calibrated chassis values; fall back to the
    # in-package template only when the runtime mount is unavailable
    # (e.g. running outside the production container). Earlier versions
    # of this launch always read the package template, which silently
    # diverged from the URDF (mowgli.launch.py uses the runtime path)
    # and gave Nav2 a footprint that did not match the actual robot.
    runtime_config = "/ros2_ws/config/mowgli_robot.yaml"
    template_config = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    robot_config_file = (
        runtime_config if os.path.isfile(runtime_config) else template_config
    )
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

    # Read dock pose and Nav2 speed knobs from the runtime config. Dock
    # pose feeds docking_server's home_dock.pose below. The WGS84 datum
    # is read by full_system.launch.py and passed to navsat_to_absolute_pose_node
    # directly — not needed here.
    dock_pose_x = 0.0
    dock_pose_y = 0.0
    dock_pose_yaw = 0.0
    # Speeds are operator-facing knobs in mowgli_robot.yaml. Nothing read
    # them before — they were orphan params — so editing them looked like
    # it should do something but didn't. Load here and inject into the
    # Nav2 YAMLs (controller + docking) alongside the dock pose.
    #   transit_speed    → FollowPath.desired_linear_vel + max_speed_xy
    #   mowing_speed     → FollowCoveragePath.max_speed_xy
    #   undock_speed     → BackUp backup_speed attribute in main_tree.xml
    #                      (hardcoded in the XML so not injected here, but
    #                      the comment keeps the three values together
    #                      for discoverability).
    transit_speed = 0.3
    mowing_speed = 0.25
    datum_lat = 0.0
    datum_lon = 0.0
    runtime_robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if os.path.isfile(runtime_robot_config):
        with open(runtime_robot_config, "r") as f:
            rt_cfg = yaml.safe_load(f) or {}
        rt_rp = rt_cfg.get("mowgli", {}).get("ros__parameters", {})
        dock_pose_x = float(rt_rp.get("dock_pose_x", 0.0))
        dock_pose_y = float(rt_rp.get("dock_pose_y", 0.0))
        dock_pose_yaw = float(rt_rp.get("dock_pose_yaw", 0.0))
        transit_speed = float(rt_rp.get("transit_speed", transit_speed))
        mowing_speed = float(rt_rp.get("mowing_speed", mowing_speed))
        datum_lat = float(rt_rp.get("datum_lat", 0.0))
        datum_lon = float(rt_rp.get("datum_lon", 0.0))

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
    def _inject_dock_pose_and_speeds(src_path: str) -> str:
        """Write mowgli_robot.yaml-derived values into the Nav2 params YAML
        and return the temp file path.

        RewrittenYaml only handles scalar substitutions, so we use this
        path for anything that needs the YAML parser (lists, or when we'd
        have to guess at the dotted-path root key). Speed params are
        scalars and could technically go through RewrittenYaml, but
        doing them here keeps all robot-yaml → nav2-yaml wiring in one
        place — easier to find when tuning later.
        """
        import tempfile
        with open(src_path, "r") as fh:
            doc = yaml.safe_load(fh) or {}
        # home_dock.pose must be a YAML list (PARAMETER_DOUBLE_ARRAY).
        (doc.setdefault("docking_server", {})
            .setdefault("ros__parameters", {})
            .setdefault("home_dock", {}))["pose"] = [
                dock_pose_x, dock_pose_y, dock_pose_yaw]

        # FollowPath (transit controller = RPP via RotationShim).
        fp = (doc.setdefault("controller_server", {})
                 .setdefault("ros__parameters", {})
                 .setdefault("FollowPath", {}))
        fp["desired_linear_vel"] = transit_speed

        # FollowCoveragePath (FTC: coverage strip controller). Its speed
        # knob is speed_fast; mowing_speed overrides it.
        fcp = (doc.setdefault("controller_server", {})
                  .setdefault("ros__parameters", {})
                  .setdefault("FollowCoveragePath", {}))
        fcp["speed_fast"] = mowing_speed

        tmp = tempfile.NamedTemporaryFile(
            mode="w", prefix="mowgli_nav2_", suffix=".yaml", delete=False)
        yaml.safe_dump(doc, tmp, default_flow_style=False, sort_keys=False)
        tmp.close()
        return tmp.name

    nav2_params_lidar = _inject_dock_pose_and_speeds(nav2_params_lidar)
    nav2_params_no_lidar = _inject_dock_pose_and_speeds(nav2_params_no_lidar)
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
    # 1. Nav2 navigation (controllers, planners, behaviors, BT navigator)
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
    # Alternative localization backend: robot_localization
    # ------------------------------------------------------------------
    # Three nodes. Active only when localization_backend == "robot_localization".
    # ekf_odom_node         : wheels + IMU gyro → odom → base_footprint TF
    # navsat_transform_node : /gps/fix + /odometry/filtered → /odometry/gps
    # ekf_map_node          : wheels + IMU + /odometry/gps → map → odom TF
    robot_localization_params = os.path.join(
        bringup_dir, "config", "robot_localization.yaml"
    )

    # navsat_transform_node looks up a TF from base_footprint to the frame
    # named in the NavSatFix header. Our URDF calls that frame gps_link,
    # but the ublox_dgnss driver publishes frame_id=gps. Alias gps_link →
    # gps as a static identity so the lookup finds a chain.
    static_gps_link_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_gps_link_to_gps_alias",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "gps_link",
            "--child-frame-id", "gps",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom_node",
        output="screen",
        parameters=[
            robot_localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    # Datum triple [lat, lon, yaw]. yaw stays 0 — our IMU has no
    # magnetometer so yaw can't be anchored to true north at boot;
    # robot yaw will align to GPS track after the first straight
    # motion, and to dock_pose_yaw at dock reset. Datum lat/lon must
    # match what was used to save areas / dock pose, otherwise saved
    # coordinates shift.
    # navsat_transform_node removed 2026-04-26 — its only output (/odometry/gps)
    # had no subscribers in the active fusion path. /gps/pose_cov from the
    # custom navsat_to_absolute_pose_node (which applies the lever-arm
    # correction with map yaw) is what ekf_map actually fuses.
    # /gps/filtered (foxglove visualisation) is replaced by /gps/absolute_pose.

    # ekf_map_node — runs only when use_fusion_graph is False. The
    # factor-graph localizer (see fusion_graph package) replaces it
    # one-for-one (publishes map -> odom + /odometry/filtered_map).
    ekf_map_node = Node(
        condition=UnlessCondition(use_fusion_graph),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map_node",
        output="screen",
        parameters=[
            robot_localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered_map"),
            # robot_localization defaults to a global /set_pose topic shared
            # by both EKFs. Remap ekf_map's subscription to a node-unique
            # name so seeding ekf_map does not also reset ekf_odom.
            ("set_pose", "/ekf_map_node/set_pose"),
        ],
    )

    # fusion_graph_node — GTSAM iSAM2 factor-graph localizer (planned
    # replacement for ekf_map_node). Mutually exclusive with the EKF
    # above. Reads datum + lever-arm from mowgli_robot.yaml inside the
    # fusion_graph launch include.
    fusion_graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fusion_graph"),
                "launch", "fusion_graph.launch.py",
            )
        ),
        condition=IfCondition(use_fusion_graph),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Seeds ekf_map with the dock heading on rising edges of is_charging.
    # Fires once per docking event plus once at boot if the robot is
    # already docked.
    dock_yaw_to_set_pose = Node(
        package="mowgli_localization",
        executable="dock_yaw_to_set_pose.py",
        name="dock_yaw_to_set_pose",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Publishes GPS course-over-ground as a synthetic sensor_msgs/Imu on
    # /imu/cog_heading so ekf_map_node can fuse it as an absolute-yaw
    # observation. Once the session is seeded and the robot is driving
    # forward faster than min_speed_ms with RTK-Fixed, this node corrects
    # gyro drift every /gps/absolute_pose sample.
    cog_to_imu = Node(
        package="mowgli_localization",
        executable="cog_to_imu.py",
        name="cog_to_imu",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "datum_lat": datum_lat,
             "datum_lon": datum_lon},
        ],
    )

    # Publishes tilt-compensated magnetic heading as a synthetic
    # sensor_msgs/Imu on /imu/mag_yaw. Active as soon as mag_calibration.yaml
    # exists (written by /calibrate_imu_yaw_node/calibrate). Unlike
    # cog_to_imu this works with the robot stationary or rotating in place.
    # Only launch if magnetometer calibration exists — without it the node
    # publishes nothing and wastes a process.
    mag_cal_path = "/ros2_ws/maps/mag_calibration.yaml"
    mag_yaw_publisher = Node(
        condition=IfCondition(str(os.path.isfile(mag_cal_path)).lower()),
        package="mowgli_localization",
        executable="mag_yaw_publisher.py",
        name="mag_yaw_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # ekf_odom_node subscribes to /wheel_odom directly (see
    # robot_localization.yaml). ekf_map_node fuses /gps/pose_cov directly
    # as pose0 (published by navsat_to_absolute_pose_node).

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ekf_arg,
            use_lidar_arg,
            use_fusion_graph_arg,
            # robot_localization dual EKF + helpers
            static_gps_link_alias,
            ekf_odom_node,
            ekf_map_node,
            fusion_graph_launch,
            dock_yaw_to_set_pose,
            cog_to_imu,
            mag_yaw_publisher,
            wait_for_map_odom_tf,
            nav2_after_tf,
        ]
    )
