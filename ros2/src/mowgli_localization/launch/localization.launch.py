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

Launches the Mowgli localization stack:
  1. wheel_odometry_node      - integrates WheelTick counts into nav_msgs/Odometry
  2. localization_monitor_node - publishes current localization quality/mode
  3. FusionCore               - single UKF (GPS + IMU + wheels) -> odom -> base_footprint TF

FusionCore parameters are loaded from mowgli_bringup/config/localization.yaml so that
all tuning lives in one place.  The wheel odometry node gets its own small
config file from mowgli_localization/config/wheel_odometry.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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
    # 2. Localization monitor node
    # ------------------------------------------------------------------
    localization_monitor_node = Node(
        package='mowgli_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        output='screen',
        parameters=[
            {
                'gps_timeout': 2.0,
                'pose_timeout': 0.5,
                'publish_rate': 10.0,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 3. FusionCore — single UKF (GPS + IMU + wheels)
    #    Publishes odom → base_footprint TF + /fusion/odom
    # ------------------------------------------------------------------
    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore_node',
        namespace='',
        output='screen',
        parameters=[
            localization_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/odom/wheels', '/wheel_odom'),
            ('/gnss/fix', '/gps/fix'),
        ],
    )

    # Auto-configure and activate the lifecycle node
    fusioncore_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state='configuring',
            goal_state='inactive',
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
    # Launch description
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_odometry_node,
            localization_monitor_node,
            fusioncore_node,
            fusioncore_configure,
            fusioncore_start,
        ]
    )
