#!/bin/bash
echo "Starting OpenMower GPS"
source /opt/open_mower_ros/src/open_mower/config/mower_config.sh
roslaunch /opt/open_mower_ros/src/open_mower/launch/include/_gps.launch
