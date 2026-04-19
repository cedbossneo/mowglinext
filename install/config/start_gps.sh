#!/bin/bash
# =============================================================================
# GPS startup script — reads config from /config/mowgli_robot.yaml
#
# Launches:
#   1. ublox_gps_node       — u-blox driver on the configured serial port
#   2. ntrip_client         — RTK corrections from NTRIP caster → /rtcm topic
#   3. rtcm_serial_bridge   — forwards /rtcm messages to GPS serial port
# =============================================================================
set -euo pipefail
set -x
CONFIG="/config/mowgli_robot.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

# Parse YAML values using grep+sed (no python dependency).
# Looks for "key: value" under the mowgli: ros__parameters: block.
parse_yaml() {
  grep -E "^\s+${1}:" "$CONFIG" | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
}

GPS_PORT=$(parse_yaml gps_port)
GPS_BAUD=$(parse_yaml gps_baudrate)
GPS_PROTOCOL=$(parse_yaml gps_protocol)
NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)

# Defaults
GPS_PORT="${GPS_PORT:-/dev/gps}"
GPS_BAUD="${GPS_BAUD:-460800}"
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"

echo "[start_gps.sh] GPS port: $GPS_PORT @ ${GPS_BAUD} baud"
echo "[start_gps.sh] NTRIP enabled: $NTRIP_ENABLED"

set +u
source /opt/ros/kilted/setup.bash
set -u


# Do not start ublox_gps_node for now.
# The receiver is already configured, and ublox_gps tries to push UART config
# at startup, which breaks this setup.
echo "[start_gps.sh] ublox_gps_node disabled for now (no GPS reconfiguration)"
GPS_PID=""
#if [ "$GPS_PROTOCOL" == "NMEA" ]; then
## Launch nmea_gps in background
#  ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
#    -p port:="${GPS_PORT}" \
#    -p baud:=${GPS_BAUD} \
#    -p frame_id:=gps_link \
#    -r /fix:=/gps/fix &
#  GPS_PID=$!
#else
#
## Generate runtime params override — only the device path.
## uart1.baudrate is intentionally omitted: the GPS is USB-connected (/dev/ttyACMx)
## so physical UART1 baud rate config is irrelevant and causes CFG-PRT failures.
## The stock launch file hardcodes c94_m8p_rover.yaml and ignores arguments,
## so we use ros2 run with two --params-file layers instead.
#
#cat > /tmp/ublox_override.yaml <<EOF
#ublox_gps_node:
#  ros__parameters:
#    device: "${GPS_PORT}"
#    baudrate: ${GPS_BAUD}
#EOF
#echo "[start_gps.sh] Generated /tmp/ublox_override.yaml:"
#cat /tmp/ublox_override.yaml
## Launch ublox_gps in background
#ros2 run ublox_gps ublox_gps_node --ros-args \
#  --params-file /f9p_rover.yaml \
#  --params-file /tmp/ublox_override.yaml \
#  -r /fix:=/gps/fix \
#  -r /fix_velocity:=/gps/fix_velocity &
#GPS_PID=$!
#fi

# Launch NTRIP client if enabled
if [ "$NTRIP_ENABLED" = "true" ]; then
  echo "[start_gps.sh] Starting NTRIP: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
# Wait for ublox to initialise before sending corrections
  sleep 5
  ros2 run ntrip_client ntrip_ros.py \
    --ros-args \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" \
    -p "authenticate:=true" \
    -p "rtcm_message_package:=rtcm_msgs" &
  NTRIP_PID=$!

# Bridge: forward /rtcm topic → GPS serial port
  sleep 2
  echo "[start_gps.sh] Starting RTCM→serial bridge on ${GPS_PORT}"
  python3 /rtcm_serial_bridge.py \
    --ros-args \
    -p "device:=${GPS_PORT}" \
    -p "baudrate:=${GPS_BAUD}" &
  BRIDGE_PID=$!
fi
echo "[start_gps.sh] GPS_PID=${GPS_PID:-none} NTRIP_PID=${NTRIP_PID:-none} BRIDGE_PID=${BRIDGE_PID:-none}"
# Wait for any child to exit, then stop everything
wait -n || true
#kill "$GPS_PID" 2>/dev/null || true
#[ -n "${GPS_PID:-}" ] && kill "$GPS_PID" 2>/dev/null || true
#[ -n "${NTRIP_PID:-}" ] && kill "$NTRIP_PID" 2>/dev/null || true
#[ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null || true
wait
