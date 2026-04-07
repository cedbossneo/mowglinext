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
Localization launch file.

Launches the full Mowgli localization stack:
  1. wheel_odometry_node      – integrates WheelTick counts into nav_msgs/Odometry
  2. gps_pose_converter_node  – converts AbsolutePose to PoseWithCovarianceStamped
  3. localization_monitor_node – publishes current localization quality/mode
  4. ekf_odom                 – robot_localization EKF (odom → base_link)
  5. ekf_map                  – robot_localization EKF (map  → odom)

EKF parameters are loaded from mowgli_bringup/config/localization.yaml so that
all tuning lives in one place.  The wheel odometry node gets its own small
config file from mowgli_localization/config/wheel_odometry.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    localization_dir = get_package_share_directory('mowgli_localization')
    bringup_dir = get_package_share_directory('mowgli_bringup')

    # ------------------------------------------------------------------
    # Config file paths
    # ------------------------------------------------------------------
    wheel_odom_config = os.path.join(localization_dir, 'config', 'wheel_odometry.yaml')
    localization_yaml = os.path.join(bringup_dir, 'config', 'localization.yaml')

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock when true.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # 1. Wheel odometry node
    # ------------------------------------------------------------------
    wheel_odometry_node = Node(
        package='mowgli_localization',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        output='screen',
        parameters=[
            wheel_odom_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 2. GPS pose converter node
    # ------------------------------------------------------------------
    gps_pose_converter_node = Node(
        package='mowgli_localization',
        executable='gps_pose_converter_node',
        name='gps_pose_converter',
        output='screen',
        parameters=[
            {
                'use_first_fix_as_datum': True,
                'datum_lat': 0.0,
                'datum_lon': 0.0,
                'min_accuracy_threshold': 0.5,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. Localization monitor node
    # ------------------------------------------------------------------
    localization_monitor_node = Node(
        package='mowgli_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        output='screen',
        parameters=[
            {
                'gps_timeout': 2.0,
                'lidar_timeout': 1.0,
                'pose_timeout': 0.5,
                'publish_rate': 10.0,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 4. robot_localization ekf_odom  (odom → base_link)
    # ------------------------------------------------------------------
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[
            localization_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # robot_localization publishes /odometry/filtered by default;
            # remap to a namespaced topic so odom and map EKFs don't clash.
            ('odometry/filtered', 'odometry/filtered_odom'),
        ],
    )

    # ------------------------------------------------------------------
    # 5. robot_localization ekf_map  (map → odom)
    # ------------------------------------------------------------------
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[
            localization_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_map'),
        ],
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_odometry_node,
            gps_pose_converter_node,
            localization_monitor_node,
            ekf_odom_node,
            ekf_map_node,
        ]
    )
