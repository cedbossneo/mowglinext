#!/bin/bash
set -e

# Source ROS2 (setup.bash uses unset variables internally)
set +u
source /opt/ros/jazzy/setup.bash
set -u

exec "$@"
