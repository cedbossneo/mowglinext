#!/bin/bash
# =============================================================================
# ros2_entrypoint.sh
#
# Container entrypoint. Sources ROS2 Kilted and the workspace overlay before
# exec'ing whatever command was passed as CMD (or via `docker run <cmd>`).
#
# NOTE: Make this script executable on the host before building:
#   chmod +x scripts/ros2_entrypoint.sh
# =============================================================================
set -e

# Source ROS2 Jazzy first (underlay) so its paths are available for
# packages not yet released for kilted (e.g. rtabmap_slam). Then source
# kilted on top so it wins for ROS_DISTRO and conflicting packages.
if [ -d /opt/ros/jazzy ] && [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck source=/opt/ros/jazzy/setup.bash
    source /opt/ros/jazzy/setup.bash
fi

# Source ROS2 base installation (kilted is primary distro)
# shellcheck source=/opt/ros/kilted/setup.bash
source /opt/ros/kilted/setup.bash

# Source the workspace overlay if it has been built (not present in dev before
# first colcon build, but always present in runtime/simulation images).
if [ -f /ros2_ws/install/setup.bash ]; then
    # shellcheck source=/ros2_ws/install/setup.bash
    source /ros2_ws/install/setup.bash
fi

exec "$@"
