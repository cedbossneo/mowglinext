#!/bin/bash
# =============================================================================
# UM982 GNSS driver startup
#
# Launches: um982_node
# Config read from /opt/mowgli_unicore_gnss/share/mowgli_unicore_gnss/config/um982.yaml
# Serial device default: /dev/gps (configurable via params)
# =============================================================================
set -euo pipefail

echo "[start_gps.sh] Launching Unicore UM982 GNSS driver..."

# Use the launch file to start the driver
ros2 launch mowgli_unicore_gnss um982_launch.py
