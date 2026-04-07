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
