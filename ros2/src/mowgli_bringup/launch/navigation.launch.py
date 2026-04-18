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
  1. slam_toolbox           – online SLAM, TF authority for map → odom.
  2. FusionCore             – single UKF (GPS+IMU+wheels), TF authority for
                              odom → base_footprint.
  3. Nav2 bringup           – full navigation stack (controller, planner,
                              recoveries, BT navigator, costmaps, lifecycle).
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
    LogInfo,
    OpaqueFunction,
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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )

    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="Run slam_toolbox when True; skip when using a pre-built map.",
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Absolute path to a pre-built map yaml file (used when slam=false).",
    )

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Run FusionCore. Set to False in simulation where Gazebo provides odom TF.",
    )

    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="lifelong",
        description="slam_toolbox mode: 'mapping' (first run), 'localization' (read-only), or 'lifelong' (load + keep updating).",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file_name",
        default_value="/ros2_ws/maps/garden_map",
        description="Full path (without extension) to a saved slam_toolbox .posegraph/.data file.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through).",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    use_ekf = LaunchConfiguration("use_ekf")
    slam_mode = LaunchConfiguration("slam_mode")
    map_file_name = LaunchConfiguration("map_file_name")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    slam_toolbox_params_file = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
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

    # Read GPS datum from runtime config (same path as SLAM function).
    # The installed config has defaults (0.0); runtime has per-site values.
    import math as _math
    datum_lat = 0.0
    datum_lon = 0.0
    datum_alt = 0.0
    gps_x = 0.0
    gps_y = 0.0
    gps_z = 0.0
    runtime_robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if os.path.isfile(runtime_robot_config):
        with open(runtime_robot_config, "r") as f:
            rt_cfg = yaml.safe_load(f) or {}
        rt_rp = rt_cfg.get("mowgli", {}).get("ros__parameters", {})
        datum_lat = float(rt_rp.get("datum_lat", 0.0))
        datum_lon = float(rt_rp.get("datum_lon", 0.0))
        # datum_alt intentionally forced to 0 — we're a 2D ground robot on
        # grass, altitude is irrelevant. Using the config value (e.g. 27 m)
        # would just offset the ECEF reference without any benefit, and
        # publish.force_2d zeroes z in the output anyway.
        datum_alt = 0.0
        gps_x = float(rt_rp.get("gps_x", 0.0))
        gps_y = float(rt_rp.get("gps_y", 0.0))
        gps_z = float(rt_rp.get("gps_z", 0.0))

    # Convert datum lat/lon/alt → ECEF (WGS84) for FusionCore's PROJ reference.
    # Without this, FusionCore uses use_first_fix=true and anchors odom to
    # wherever the robot first saw GPS — not the configured datum. Map frame
    # then drifts from the GPS-absolute-pose frame, and the robot appears
    # off-position on the displayed map.
    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563
    WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
    datum_is_fixed = (datum_lat != 0.0 or datum_lon != 0.0)
    ref_x = ref_y = ref_z = 0.0
    if datum_is_fixed:
        lat_rad = _math.radians(datum_lat)
        lon_rad = _math.radians(datum_lon)
        sin_lat = _math.sin(lat_rad)
        cos_lat = _math.cos(lat_rad)
        N = WGS84_A / _math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        ref_x = (N + datum_alt) * cos_lat * _math.cos(lon_rad)
        ref_y = (N + datum_alt) * cos_lat * _math.sin(lon_rad)
        ref_z = (N * (1.0 - WGS84_E2) + datum_alt) * sin_lat

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

    # Build rewritten variants of the slam_toolbox yaml so the launch
    # file can pass the correct mode and map_file_name without touching the
    # config file on disk.
    mapping_slam_params = RewrittenYaml(
        source_file=slam_toolbox_params_file,
        root_key="",
        param_rewrites={
            "mode": "mapping",
            "map_file_name": map_file_name,
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )

    localization_slam_params = RewrittenYaml(
        source_file=slam_toolbox_params_file,
        root_key="",
        param_rewrites={
            "mode": "localization",
            "map_file_name": map_file_name,
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )

    lifelong_slam_params = RewrittenYaml(
        source_file=slam_toolbox_params_file,
        root_key="",
        param_rewrites={
            "mode": "lifelong",
            "map_file_name": map_file_name,
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )

    # ------------------------------------------------------------------
    # 1. slam_toolbox  (only when slam=true)
    #
    #    Uses an OpaqueFunction to check at launch time whether the saved
    #    map file exists.  If the requested mode is 'lifelong' or
    #    'localization' but the .posegraph file is missing, we fall back
    #    to 'mapping' mode so slam_toolbox doesn't crash on first run.
    #
    #    mapping mode     → online_async_launch.py  (builds a new map)
    #    localization mode → localization_launch.py  (read-only pose graph)
    #    lifelong mode    → online_async_launch.py   (loads saved map +
    #                        keeps adding new scans — map improves over time)
    # ------------------------------------------------------------------
    def _launch_slam_toolbox(context):
        resolved_mode = context.launch_configurations["slam_mode"]
        resolved_map = context.launch_configurations["map_file_name"]
        resolved_slam = context.launch_configurations["slam"]
        resolved_sim = context.launch_configurations["use_sim_time"]

        # If slam is disabled, return nothing
        if resolved_slam.lower() not in ("true", "1", "yes"):
            return []

        # Check if the saved posegraph file exists
        posegraph_file = resolved_map + ".posegraph"
        map_exists = os.path.isfile(posegraph_file)

        effective_mode = resolved_mode
        if not map_exists and resolved_mode in ("lifelong", "localization"):
            effective_mode = "mapping"

        actions = []
        if not map_exists and resolved_mode != "mapping":
            actions.append(
                LogInfo(msg=(
                    f"[navigation.launch.py] Map file '{posegraph_file}' not found. "
                    f"Falling back from '{resolved_mode}' to 'mapping' mode."
                ))
            )

        # SLAM start pose: ONLY use dock pose when we can confirm the robot
        # is currently on the dock (charging). hardware_bridge writes the
        # dock pose to /tmp/dock_start_pose.txt the first time charging is
        # detected during its lifetime — if that file exists AND is recent,
        # we trust that we're docked. Otherwise skip the override so
        # slam_toolbox uses the saved map's native origin; GPS will anchor
        # us via the map→slam_map corrector once scans register.
        dock_start_pose = None
        dock_start_pose_file = "/tmp/dock_start_pose.txt"
        if os.path.isfile(dock_start_pose_file):
            try:
                mtime = os.path.getmtime(dock_start_pose_file)
                age = abs(_time.time() - mtime) if ("_time" in dir()) else 0
            except Exception:
                age = 0
            try:
                with open(dock_start_pose_file, "r") as f:
                    parts = f.read().strip().split()
                if len(parts) >= 3:
                    # hardware_bridge writes the yaw in compass convention
                    # (0=N, 90=E, CW). slam_toolbox's map_start_pose needs
                    # ENU (0=E, 90=N, CCW). Convert: enu = pi/2 - compass.
                    compass_yaw = float(parts[2])
                    enu_yaw = (_math.pi / 2.0) - compass_yaw
                    # Normalize to [-pi, pi] for cleanliness.
                    while enu_yaw > _math.pi:
                        enu_yaw -= 2.0 * _math.pi
                    while enu_yaw <= -_math.pi:
                        enu_yaw += 2.0 * _math.pi
                    dock_start_pose = [float(parts[0]), float(parts[1]), enu_yaw]
                    actions.append(
                        LogInfo(msg=(
                            f"[navigation.launch.py] SLAM start pose from dock file "
                            f"[{dock_start_pose[0]:.2f}, {dock_start_pose[1]:.2f}, "
                            f"enu_yaw={enu_yaw:.3f} rad ({_math.degrees(enu_yaw):.1f}°), "
                            f"from compass {compass_yaw:.3f} rad]"
                        ))
                    )
            except Exception as e:
                actions.append(
                    LogInfo(msg=f"[navigation.launch.py] Could not read {dock_start_pose_file}: {e}")
                )
        else:
            actions.append(
                LogInfo(msg=(
                    "[navigation.launch.py] Not on dock (no /tmp/dock_start_pose.txt); "
                    "letting slam_toolbox start from saved map origin (GPS anchors later)."
                ))
            )

        # If we have a dock start pose, create a modified copy of the SLAM
        # config with the dock pose baked in. RewrittenYaml can't handle
        # list-type params, so we modify the YAML directly.  We create
        # per-mode RewrittenYaml variants from this modified file so the
        # dock heading hint is available in ALL modes.
        if dock_start_pose is not None:
            import tempfile
            with open(slam_toolbox_params_file, "r") as f:
                slam_yaml_content = f.read()
            # Replace the default map_start_pose with dock pose
            slam_yaml_content = slam_yaml_content.replace(
                "map_start_pose: [0.0, 0.0, 0.0]",
                f"map_start_pose: [{dock_start_pose[0]}, "
                f"{dock_start_pose[1]}, {dock_start_pose[2]}]"
            )
            # delete=False is required: the file must persist on disk so that
            # RewrittenYaml / the SLAM node can read it after this function
            # returns.  The OS will clean it up on reboot (or container restart).
            dock_slam_file = tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', prefix='slam_dock_',
                delete=False
            )
            dock_slam_file.write(slam_yaml_content)
            dock_slam_file.close()
            dock_source = dock_slam_file.name
        else:
            dock_source = slam_toolbox_params_file

        # Build per-mode params from the (possibly dock-patched) source
        # map_start_at_dock only for mapping (fresh map needs dock as origin).
        # Lifelong/localization load saved posegraph — don't override position.
        dock_mapping_params = RewrittenYaml(
            source_file=dock_source,
            root_key="",
            param_rewrites={
                "mode": "mapping",
                "map_start_at_dock": "true",
                "map_file_name": map_file_name,
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        )
        dock_localization_params = RewrittenYaml(
            source_file=dock_source,
            root_key="",
            param_rewrites={
                "mode": "localization",
                "map_start_at_dock": "false",
                "map_file_name": map_file_name,
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        )
        dock_lifelong_params = RewrittenYaml(
            source_file=dock_source,
            root_key="",
            param_rewrites={
                "mode": "lifelong",
                "map_start_at_dock": "false",
                "map_file_name": map_file_name,
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        )

        # Select the correct launch file and params
        if effective_mode == "localization":
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "localization_launch.py"
            )
            params = dock_localization_params
        elif effective_mode == "mapping":
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "online_async_launch.py"
            )
            params = dock_mapping_params
        else:  # lifelong
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "online_async_launch.py"
            )
            params = dock_lifelong_params

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "use_sim_time": resolved_sim,
                    "slam_params_file": params,
                }.items(),
            )
        )

        return actions

    slam_toolbox_launch = OpaqueFunction(function=_launch_slam_toolbox)

    # ------------------------------------------------------------------
    # 2. FusionCore — single UKF (GPS + IMU + wheels)
    #    Publishes odom → base_footprint TF.
    #    SLAM Toolbox publishes map → odom TF (re-enabled above).
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
            # Anchor FusionCore's odom frame to the configured datum so
            # map frame == GPS ENU frame. When datum is 0, fall back to
            # first-fix for bench tests.
            {"reference.use_first_fix": not datum_is_fixed},
            {"reference.x": ref_x},
            {"reference.y": ref_y},
            {"reference.z": ref_z},
        ],
        remappings=[
            ("/odom/wheels", "/wheel_odom"),
            # GPS intentionally NOT remapped to /gps/fix — FusionCore runs
            # wheel+IMU-only for smooth local dead-reckoning. GPS noise
            # is injected only at the map→slam_map layer via the
            # gps_slam_corrector, and slam_toolbox corrects odom drift
            # locally via scan matching. This decouples the three layers:
            #   GPS → global anchor (gps_slam_corrector)
            #   LiDAR → local correction (slam_toolbox)
            #   wheels+IMU → short-term smoothness (FusionCore)
            # Previously, fusing GPS into FusionCore made odom noisy with
            # Float jitter — slam_toolbox couldn't fight it back because
            # scan-match corrections are weak in open outdoor scenes.
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
    # 3b. GPS-SLAM corrector — publishes map→slam_map TF
    #     Bridges GPS real-world coordinates with SLAM's internal frame.
    #     Only active when SLAM is enabled (no-SLAM mode uses static TF).
    # ------------------------------------------------------------------
    gps_slam_corrector = Node(
        condition=IfCondition(slam),
        package="mowgli_localization",
        executable="gps_slam_corrector_node",
        name="gps_slam_corrector",
        output="screen",
        parameters=[
            {"map_frame": "map"},
            {"slam_frame": "slam_map"},
            {"base_frame": "base_footprint"},
            {"publish_rate": 10.0},
            # Umeyama alignment: one-shot fit from accumulated
            # correspondences, then freeze map→slam_map. Replaces the
            # old low-pass filter that drifted with Float GPS jitter.
            {"min_samples": 20},
            {"min_motion_m": 1.5},
            {"min_fix_status": 1},  # accept Float+ during alignment
            {"sample_period_sec": 0.2},
            {"datum_lat": datum_lat},
            {"datum_lon": datum_lon},
            {"gps_x": gps_x},
            {"gps_y": gps_y},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 4. Nav2 navigation (controllers, planners, behaviors, BT navigator)
    #    We use navigation_launch.py directly instead of bringup_launch.py
    #    because bringup_launch.py also starts localization (AMCL) which
    #    would fight with our slam_toolbox over the map→odom TF.
    # ------------------------------------------------------------------
    # Nav2's navigation_launch.py creates lifecycle_manager_navigation with
    # hardcoded params (ignoring params_file for the lifecycle manager node).
    # Wrap in a GroupAction with SetParameter so bond_timeout is available as
    # a global parameter override — lifecycle_manager will pick it up.
    #
    # ------------------------------------------------------------------
    # 3. Static map→odom fallback (no-SLAM mode)
    # When SLAM is disabled, FusionCore's odom is already GPS-anchored
    # so map≈odom. Publish a static identity TF to satisfy Nav2.
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Gate Nav2 startup on the map→odom TF being available.
    # wait_for_tf.py polls every 0.5 s and exits as soon as the
    # transform is published (by SLAM or by static_map_odom_tf).
    # Installed to lib/mowgli_bringup/ by CMakeLists.txt (install PROGRAMS)
    wait_for_tf_script = os.path.join(
        get_package_prefix("mowgli_bringup"),
        "lib", "mowgli_bringup", "wait_for_tf.py"
    )

    wait_for_slam_tf = ExecuteProcess(
        cmd=[
            "python3", wait_for_tf_script,
            "--parent", "map",
            "--child", "odom",
            "--timeout", "120",
        ],
        name="wait_for_slam_tf",
        output="screen",
    )

    nav2_navigation_group = GroupAction(
        actions=[
            SetParameter("bond_timeout", 10.0),
            # Use our vendored copy of Nav2's navigation_launch.py with
            # route_server removed (not needed for cell-based coverage).
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
            target_action=wait_for_slam_tf,
            on_exit=[nav2_navigation_group],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            slam_arg,
            map_yaml_arg,
            use_ekf_arg,
            slam_mode_arg,
            map_file_arg,
            use_lidar_arg,
            slam_toolbox_launch,
            gps_slam_corrector,
            static_map_odom_tf,
            fusioncore_node,
            fusioncore_configure,
            fusioncore_start,
            wait_for_slam_tf,
            nav2_after_tf,
        ]
    )
