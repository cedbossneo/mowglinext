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

"""Unicore UM982 GNSS backend bringup for Mowgli."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    params_file = os.path.join(bringup_dir, "config", "gnss_unicore.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    serial_port_arg = DeclareLaunchArgument(
        "unicore_serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Unicore UM982 GNSS receiver.",
    )
    baudrate_arg = DeclareLaunchArgument(
        "unicore_baudrate",
        default_value="921600",
        description="Baudrate for Unicore UM982 serial communication.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "unicore_frame_id",
        default_value="gnss",
        description="Frame ID reported by the Unicore backend.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("unicore_serial_port")
    baudrate = LaunchConfiguration("unicore_baudrate")
    frame_id = LaunchConfiguration("unicore_frame_id")

    unicore_node = Node(
        package="mowgli_unicore_gnss",
        executable="um982_node",
        name="um982_node",
        output="screen",
        parameters=[
            params_file,
            {
                "port": serial_port,
                "baudrate": baudrate,
                "frame_id": frame_id,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            baudrate_arg,
            frame_id_arg,
            unicore_node,
        ]
    )