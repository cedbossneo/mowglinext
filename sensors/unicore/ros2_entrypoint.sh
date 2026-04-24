#!/bin/bash
set -e

# Source ROS2 base environment
set +u
source /opt/ros/kilted/setup.bash
# mowgli_unicore_gnss built from source and installed to /opt/mowgli_unicore_gnss
if [ -f /opt/mowgli_unicore_gnss/setup.bash ]; then
  source /opt/mowgli_unicore_gnss/setup.bash
fi
set -u

exec "$@"
