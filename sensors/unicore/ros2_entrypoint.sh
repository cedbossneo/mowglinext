#!/bin/bash
set -e

# Source ROS2 base environment
set +u
source /opt/ros/kilted/setup.bash
# Autonomous Unicore workspace install tree
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
set -u

exec "$@"
