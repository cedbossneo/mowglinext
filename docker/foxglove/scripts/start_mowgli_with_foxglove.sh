#!/bin/bash
set -e

echo "Starting OpenMower + Foxglove"

source /opt/ros/noetic/setup.bash
source /opt/open_mower_ros/devel/setup.bash
source /config/mower_config.sh

export MOWER="${OM_MOWER:-CUSTOM}"
export ESC_TYPE="${OM_MOWER_ESC_TYPE:-xesc_mini}"
export HARDWARE_PLATFORM="${OM_HARDWARE_VERSION:-1}"
export PARAMS_PATH="${OM_PARAMS_PATH:-/config}"
export OM_LEGACY_CONFIG_MODE="True"

cleanup() {
  echo "Stopping processes..."
  kill "$MOWGLI_PID" "$FOXGLOVE_PID" 2>/dev/null || true
  wait
}

trap cleanup SIGINT SIGTERM

roslaunch --wait open_mower open_mower.launch &
MOWGLI_PID=$!

sleep 5

roslaunch --screen foxglove_bridge foxglove_bridge.launch &
FOXGLOVE_PID=$!

wait