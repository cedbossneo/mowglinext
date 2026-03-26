"""
simulation.launch.py

Simulation launch file for the Mowgli robot mower using Gazebo Ignition (Fortress+).

Brings up:
  1. robot_state_publisher  – publishes /robot_description + static TF.
  2. Gazebo Ignition         – empty world simulation.
  3. Robot spawner           – spawns Mowgli into the running Gazebo world.
  4. ros_gz_bridge           – bridges Gazebo topics to ROS2:
       /clock
       /scan          (sensor_msgs/LaserScan)
       /imu/data      (sensor_msgs/Imu)
       /wheel_odom    (nav_msgs/Odometry)
       /cmd_vel       (geometry_msgs/Twist)
  5. joint_state_publisher   – provides wheel joint states for TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world file (name or absolute path).",
    )

    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x", default_value="0.0", description="Robot spawn X position."
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y", default_value="0.0", description="Robot spawn Y position."
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z", default_value="0.1", description="Robot spawn Z position."
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw", default_value="0.0", description="Robot spawn yaw (radians)."
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    use_sim_time = "true"

    # ------------------------------------------------------------------
    # URDF / xacro
    # ------------------------------------------------------------------
    xacro_file = os.path.join(bringup_dir, "urdf", "mowgli.urdf.xacro")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # ------------------------------------------------------------------
    # 1. robot_state_publisher
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 2. Gazebo Ignition
    #    ros_gz_sim package provides the ign_gazebo.launch.py file.
    # ------------------------------------------------------------------
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -v4 ", world]}.items(),
    )

    # ------------------------------------------------------------------
    # 3. Spawn robot into Gazebo
    #    ros_gz_sim provides the create node for model spawning.
    # ------------------------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mowgli",
        output="screen",
        arguments=[
            "-name", "mowgli",
            "-topic", "robot_description",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 4. ros_gz_bridge – topic bridging between Gazebo and ROS2
    # ------------------------------------------------------------------
    bridge_config = (
        # Clock
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock "
        # LiDAR
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan "
        # IMU
        "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU "
        # Wheel odometry (Gazebo → ROS2)
        "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry "
        # cmd_vel (ROS2 → Gazebo)
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        arguments=[bridge_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 5. joint_state_publisher  (publishes wheel joint states)
    #    Launch after the robot is spawned so the URDF is available.
    # ------------------------------------------------------------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start joint_state_publisher only after the spawner exits (success)
    joint_state_publisher_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_publisher_node],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            world_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            robot_state_publisher_node,
            gazebo,
            spawn_robot,
            bridge_node,
            joint_state_publisher_after_spawn,
        ]
    )
