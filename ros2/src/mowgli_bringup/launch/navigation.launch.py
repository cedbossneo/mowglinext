"""
navigation.launch.py

Navigation stack launch file for the Mowgli robot mower.

Brings up:
  1. slam_toolbox           – online SLAM (mapping mode by default).
  2. robot_localization     – dual EKF:
       ekf_odom: wheel_odom + IMU  →  odom → base_link TF
       ekf_map:  filtered_odom + GPS pose → map → odom TF
  3. Nav2 bringup           – full navigation stack (controller, planner,
                              recoveries, BT navigator, costmaps, lifecycle).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
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
        description="Run dual EKF nodes. Set to False in simulation where Gazebo provides odom TF.",
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

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    use_ekf = LaunchConfiguration("use_ekf")
    slam_mode = LaunchConfiguration("slam_mode")
    map_file_name = LaunchConfiguration("map_file_name")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    slam_toolbox_params_file = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    localization_params = os.path.join(bringup_dir, "config", "localization.yaml")
    nav2_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    # Rewrite use_sim_time throughout nav2_params.yaml so that all Nav2 nodes
    # use the correct clock source.  Nav2's navigation_launch.py does NOT
    # inject use_sim_time into the params file automatically.
    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={"use_sim_time": use_sim_time},
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

        # Select the correct launch file and params
        if effective_mode == "localization":
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "localization_launch.py"
            )
            params = localization_slam_params
        elif effective_mode == "mapping":
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "online_async_launch.py"
            )
            params = mapping_slam_params
        else:  # lifelong
            launch_file = os.path.join(
                slam_toolbox_dir, "launch", "online_async_launch.py"
            )
            params = lifelong_slam_params

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

    slam_toolbox_opaque = OpaqueFunction(function=_launch_slam_toolbox)

    # Delay slam_toolbox startup by 10s so Gazebo's /clock bridge is
    # established first.  Without this, use_sim_time lifecycle nodes
    # can't process configure/activate transitions and SLAM never starts.
    slam_toolbox_launch = TimerAction(
        period=10.0,
        actions=[slam_toolbox_opaque],
    )

    # ------------------------------------------------------------------
    # 2. robot_localization – dual EKF
    #    Delayed 15s so the Gazebo clock bridge is established first.
    #    Without this, the EKF nodes hang on "Waiting for clock to start"
    #    and never publish the odom→base_link TF that Nav2 needs.
    # ------------------------------------------------------------------
    ekf_odom_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                condition=IfCondition(use_ekf),
                package="robot_localization",
                executable="ekf_node",
                name="ekf_odom",
                output="screen",
                parameters=[
                    localization_params,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("odometry/filtered", "/mowgli/localization/odom")],
            ),
        ],
    )

    ekf_map_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                condition=IfCondition(use_ekf),
                package="robot_localization",
                executable="ekf_node",
                name="ekf_map",
                output="screen",
                parameters=[
                    localization_params,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("odometry/filtered", "/mowgli/localization/odom_map")],
            ),
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
    # Delay Nav2 startup so SLAM / static TF has time to publish the
    # map → odom transform.  Without this delay, the planner_server's
    # global costmap times out waiting for the map frame and the
    # lifecycle_manager aborts the entire bringup.
    nav2_navigation = TimerAction(
        period=40.0,
        actions=[
            GroupAction(
                actions=[
                    SetParameter("bond_timeout", 10.0),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                nav2_bringup_dir, "launch", "navigation_launch.py"
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_params,
                        }.items(),
                    ),
                ]
            ),
        ],
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
            slam_toolbox_launch,
            ekf_odom_node,
            ekf_map_node,
            nav2_navigation,
        ]
    )
