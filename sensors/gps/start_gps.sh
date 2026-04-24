#!/bin/bash
# =============================================================================
# GPS startup — ublox_dgnss driver + NTRIP client.
#
# ublox_dgnss finds the F9P by USB VID:PID, so no /dev path is needed.
# Reads NTRIP credentials from /config/mowgli_robot.yaml (bind-mounted).
#
# Launches:
#   1. ublox_dgnss_node              — F9P driver (libusb)
#   2. ublox_nav_sat_fix_hp_node     — UBX HP → NavSatFix on /fix → /gps/fix
#   3. ntrip_client_node             — NTRIP → RTCM → driver (via libusb)
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

parse_yaml() {
  grep -E "^\s+${1}:" "$CONFIG" | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
}

NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"

set +u
source /opt/ros/kilted/setup.bash
source /opt/ublox_dgnss/setup.bash
set -u

echo "[start_gps.sh] Starting ublox_dgnss (libusb — no device path needed)"

# ublox_dgnss driver
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
  --params-file /ublox_dgnss.yaml &
GPS_PID=$!

# UBX HP → NavSatFix — remap /fix → /gps/fix to keep downstream consumers
# (navsat_transform_node, navsat_to_absolute_pose, GUI) on the current topic name.
ros2 run ublox_nav_sat_fix_hp_node ublox_nav_sat_fix_hp --ros-args \
  --params-file /ublox_dgnss.yaml \
  -r /fix:=/gps/fix &
HP_PID=$!

if [ "$NTRIP_ENABLED" = "true" ]; then
  echo "[start_gps.sh] NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
  # Wait for driver to open the USB device before pushing RTCM.
  sleep 3
  ros2 run ntrip_client_node ntrip_client_node --ros-args \
    --params-file /ublox_dgnss.yaml \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" &
  NTRIP_PID=$!
fi

wait -n || true
kill "$GPS_PID" "$HP_PID" 2>/dev/null || true
[ -n "${NTRIP_PID:-}" ] && kill "$NTRIP_PID" 2>/dev/null || true
wait
