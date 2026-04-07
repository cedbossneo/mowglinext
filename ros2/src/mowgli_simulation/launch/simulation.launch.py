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
simulation.launch.py

Full simulation launch file for the Mowgli robot mower using Gazebo Ignition Fortress.

Launch sequence:
  1. Declare arguments (world, use_rviz, headless, spawn_x/y/z/yaw).
  2. Launch Gazebo Ignition with the selected world SDF.
  3. Publish /robot_description via robot_state_publisher (from mowgli_bringup URDF).
  4. Spawn the mowgli_mower Gazebo model at the docking station position.
  5. Start ros_gz_bridge with the YAML bridge configuration.
  6. Optionally start RViz2 with the simulation config.

The mower spawns from the standalone Gazebo model (models/mowgli_mower/model.sdf)
so Gazebo handles its own physics/sensors, while robot_state_publisher provides
/robot_description for RViz visualisation and TF from the URDF.
"""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    sim_share = get_package_share_directory("mowgli_simulation")
    bringup_share = get_package_share_directory("mowgli_bringup")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="garden",
        description=(
            "Gazebo world to load. Either 'garden' or 'empty_garden' (selects the "
            "matching SDF from the mowgli_simulation worlds/ directory), or an "
            "absolute path to a custom SDF file."
        ),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with the mowgli_sim.rviz configuration.",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode (no GUI). Useful for CI/testing.",
    )

    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Robot spawn X position (metres).",
    )

    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot spawn Y position (metres).",
    )

    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05",
        description="Robot spawn Z position (metres). Slightly above ground.",
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle (radians).",
    )

    # ------------------------------------------------------------------
    # LaunchConfiguration handles
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # ------------------------------------------------------------------
    # 1. Gazebo Ignition (via OpaqueFunction to resolve world path at runtime)
    # ------------------------------------------------------------------
    def launch_gazebo(context):
        world_name = LaunchConfiguration("world").perform(context)
        is_headless = LaunchConfiguration("headless").perform(context).lower()

        # Resolve world SDF path
        world_sdf = os.path.join(sim_share, "worlds", world_name + ".sdf")
        if not os.path.isfile(world_sdf):
            # Assume it's an absolute path
            world_sdf = world_name

        if is_headless == "true":
            # --headless-rendering tells Gazebo to use EGL for off-screen
            # rendering (Mesa llvmpipe).  Without it, the Sensors system
            # blocks forever trying to initialise an ogre2 context when
            # there is no GPU or display — freezing physics and /clock.
            #
            # On containers without /dev/dri we also need a virtual X
            # display so the Mesa EGL path has something to bind to.
            # Always ensure a working virtual display for ogre2.
            # A stale DISPLAY variable (e.g. ":2" with no X server) will
            # cause the Sensors plugin to block forever, freezing physics.
            if shutil.which("Xvfb"):
                # Check if Xvfb :99 is already running
                xvfb_running = subprocess.run(
                    ["pgrep", "-f", "Xvfb :99"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                ).returncode == 0
                if not xvfb_running:
                    # Clean stale lock
                    lock = "/tmp/.X99-lock"
                    if os.path.exists(lock):
                        os.remove(lock)
                    subprocess.Popen(
                        ["Xvfb", ":99", "-screen", "0", "1024x768x24", "-ac"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    import time
                    time.sleep(0.5)
                os.environ["DISPLAY"] = ":99"

            server_only = " -s --headless-rendering "
        else:
            server_only = " "

        gz_args = f"-r -v3{server_only}{world_sdf}"

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            )
        ]

    gazebo_launch = OpaqueFunction(function=launch_gazebo)

    # ------------------------------------------------------------------
    # 2. robot_state_publisher — publishes /robot_description and static TF
    #    Uses the mowgli URDF from mowgli_bringup for RViz display.
    # ------------------------------------------------------------------
    urdf_xacro = os.path.join(bringup_share, "urdf", "mowgli.urdf.xacro")

    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_xacro]
    )

    # robot_state_publisher uses use_sim_time=False to prevent TF time-jump
    # errors.  Its /tf_static are timeless URDF joint transforms.  Using wall
    # clock avoids the "Detected jump back in time" cascade when Gazebo's
    # /clock first arrives and every TF listener clears its buffer.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
        ],
    )

    # ------------------------------------------------------------------
    # 3. Spawn the mowgli_mower Gazebo model
    #    Uses the standalone SDF model (not the URDF) so Gazebo plugins
    #    (diff-drive, LiDAR, IMU) are fully functional.
    # ------------------------------------------------------------------
    mower_sdf = os.path.join(
        sim_share, "models", "mowgli_mower", "model.sdf"
    )

    spawn_mower_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mowgli",
        output="screen",
        arguments=[
            "-name", "mowgli_mower",
            "-file", mower_sdf,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 4. ros_gz_bridge — topic bridging (YAML parameter file)
    # ------------------------------------------------------------------
    bridge_config = os.path.join(sim_share, "config", "gazebo_bridge.yaml")

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 4b. Static TF: lidar_link → Gazebo sensor frame (identity)
    #     Gazebo Ignition names sensor frames as "model/link/sensor".
    #     The URDF provides base_link→lidar_link. This identity bridge
    #     lets SLAM/Nav2 resolve the Gazebo scan frame back to base_link.
    # ------------------------------------------------------------------
    # Static TF bridges use use_sim_time=False to avoid TF buffer time-jump
    # errors during sim clock initialization.  These are identity transforms
    # that don't depend on time — using wall clock is correct and avoids the
    # "Detected jump back in time" cascade that breaks odom→base_link TF.
    gz_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gz_lidar_frame_bridge",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "lidar_link",
            "--child-frame-id", "mowgli_mower/laser_link/lidar_sensor",
        ],
    )

    gz_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gz_imu_frame_bridge",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "imu_link",
            "--child-frame-id", "mowgli_mower/base_link/imu_sensor",
        ],
    )

    # ------------------------------------------------------------------
    # 5. RViz2 — optional, started after spawner exits
    # ------------------------------------------------------------------
    rviz_config = os.path.join(sim_share, "rviz", "mowgli_sim.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Start RViz only after the mower has been successfully spawned
    rviz_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_mower_node,
            on_exit=[rviz_node],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            world_arg,
            use_rviz_arg,
            headless_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            # Nodes / includes
            gazebo_launch,
            robot_state_publisher_node,
            spawn_mower_node,
            bridge_node,
            gz_lidar_tf,
            gz_imu_tf,
            rviz_after_spawn,
        ]
    )
