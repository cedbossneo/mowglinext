"""
foxglove_bridge.launch.py

Starts the foxglove_bridge WebSocket server for Foxglove Studio.

Foxglove Bridge provides a high-performance binary WebSocket protocol,
replacing the JSON-based rosbridge_server used on Humble.

Connect Foxglove Studio to: ws://<host>:8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    send_buffer_limit_arg = DeclareLaunchArgument(
        "send_buffer_limit",
        default_value="10000000",
        description="Maximum bytes buffered per client before dropping messages.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    port = LaunchConfiguration("port")
    send_buffer_limit = LaunchConfiguration("send_buffer_limit")

    # ------------------------------------------------------------------
    # foxglove_bridge node
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": port,
                "address": "0.0.0.0",
                "send_buffer_limit": send_buffer_limit,
                "num_threads": 0,
            },
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            port_arg,
            send_buffer_limit_arg,
            foxglove_bridge_node,
        ]
    )
