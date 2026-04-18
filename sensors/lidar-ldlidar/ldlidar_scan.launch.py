# Custom launch for Myzhar ldrobot-lidar-ros2 in MowgliNext.
#
# Wraps the lidar component in a ComposableNodeContainer so we can remap
# the internal ~/scan topic to /scan (which all downstream nodes —
# slam_toolbox, Nav2 costmaps — expect). The bundled ldlidar_with_mgr
# launch publishes on /<node_name>/scan, which would force us to remap
# every consumer.
#
# Auto-configures and activates the lifecycle node via
# nav2_lifecycle_manager so the container can just be `docker run`.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    params_file = "/ldlidar.yaml"
    if not os.path.isfile(params_file):
        params_file = os.path.join(
            get_package_share_directory("ldlidar_node"),
            "params",
            "ldlidar.yaml",
        )

    lidar_component = ComposableNode(
        package="ldlidar_component",
        plugin="ldlidar::LdLidarComponent",
        name="ldlidar_node",
        namespace="",
        parameters=[params_file],
        remappings=[
            ("~/scan", "/scan"),
        ],
    )

    container = ComposableNodeContainer(
        name="ldlidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[lidar_component],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="ldlidar_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["ldlidar_node"],
            "bond_timeout": 10.0,
        }],
    )

    return LaunchDescription([container, lifecycle_manager])
