#!/bin/bash
# sim-stop.sh — Gracefully stop all ROS2/Gazebo processes and clean DDS state.
#
# 1. Send SIGINT to ros2 launch processes (triggers graceful shutdown of all nodes)
# 2. Wait up to 5s for children to exit
# 3. SIGKILL any stragglers
# 4. Clean Cyclone DDS shared memory and Gazebo transport state
set -e
trap "" INT  # Ignore SIGINT so we don't die from the signals we send

# Find ros2 launch python processes (not this script)
PIDS=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')

if [ -n "$PIDS" ]; then
  echo "Stopping ros2 launch (SIGINT)..."
  kill -INT $PIDS 2>/dev/null || true

  for i in 1 2 3 4 5; do
    sleep 1
    REMAINING=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')
    [ -z "$REMAINING" ] && break
  done

  REMAINING=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')
  if [ -n "$REMAINING" ]; then
    echo "Force killing stragglers (SIGKILL)..."
    kill -9 $REMAINING 2>/dev/null || true
    sleep 1
  fi
fi

# Kill any remaining Gazebo processes (they survive ros2 launch SIGINT)
GZ_PIDS=$(ps -eo pid,args | grep '[g]z sim' | awk '{print $1}')
if [ -n "$GZ_PIDS" ]; then
  echo "Killing Gazebo processes..."
  kill -9 $GZ_PIDS 2>/dev/null || true
  sleep 1
fi

# Clean DDS shared memory and Gazebo transport state
rm -rf /dev/shm/cyclone* /dev/shm/dds* /dev/shm/iox* /tmp/gz-* /tmp/ign-*
echo "All clean."
