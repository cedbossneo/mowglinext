#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/open_mower_ros/devel/setup.bash

exec roslaunch --screen foxglove_bridge foxglove_bridge.launch