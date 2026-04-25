#!/bin/bash
# =============================================================================
# NMEA GPS startup. Reads config from /config/mowgli_robot.yaml.
#
# Launches:
#   1. nmea_serial_bridge.py    /dev/gps → /nmea_sentence (RELIABLE)
#   2. nmea_topic_driver        /nmea_sentence → /gps/fix (NavSatFix)
#   3. str2str (NTRIP)          NTRIP caster → RTCM3 → /dev/gps (write side)
#   + inotify watcher reloads str2str when ntrip_* keys in the YAML change.
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"
[ -f "$CONFIG" ] || { echo "[gps-nmea] $CONFIG missing — bind-mount install/config/mowgli to /config"; exit 1; }

parse_yaml() {
  grep -E "^\s+${1}:" "$CONFIG" 2>/dev/null | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
}

GPS_PORT=$(parse_yaml gps_port)
GPS_BAUD=$(parse_yaml gps_baudrate)
GPS_FRAME=$(parse_yaml gps_frame_id)
NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASS=$(parse_yaml ntrip_password)
NTRIP_MOUNT=$(parse_yaml ntrip_mountpoint)

GPS_PORT="${GPS_PORT:-/dev/gps}"
GPS_BAUD="${GPS_BAUD:-115200}"
GPS_FRAME="${GPS_FRAME:-gps_link}"
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"
NTRIP_PORT="${NTRIP_PORT:-2101}"

echo "[gps-nmea] port=$GPS_PORT baud=$GPS_BAUD frame=$GPS_FRAME ntrip=$NTRIP_ENABLED"

set +u; source /opt/ros/kilted/setup.bash; set -u

# 1. Raw NMEA reader → /nmea_sentence (RELIABLE inside the script).
python3 /nmea_serial_bridge.py --ros-args \
  -p port:="${GPS_PORT}" \
  -p baud:="${GPS_BAUD}" \
  -p frame_id:="${GPS_FRAME}" &
BRIDGE_PID=$!

# 2. nmea_navsat_driver topic mode: /nmea_sentence → /gps/fix.
#    Trust the driver's default publisher QoS (RELIABLE in current upstream).
#    If FusionCore stops receiving fixes after upstream changes, switch to a
#    bespoke NavSatFix publisher inside nmea_serial_bridge.py.
ros2 run nmea_navsat_driver nmea_topic_driver --ros-args \
  -p frame_id:="${GPS_FRAME}" \
  -p time_ref_source:=gps \
  -p useRMC:=false \
  -p use_GNSS_time:=false \
  -r /fix:=/gps/fix \
  -r /vel:=/gps/vel \
  -r /time_reference:=/gps/time_reference &
DRIVER_PID=$!

# 3. NTRIP via str2str — pulls RTCM3 from the caster, writes it to the GPS
#    serial device. Multi-opener safe on Linux CDC.
STR2STR_PID=0
start_str2str() {
  [ "$NTRIP_ENABLED" = "true" ] || return
  [ -n "${NTRIP_HOST:-}" ] && [ -n "${NTRIP_MOUNT:-}" ] || {
    echo "[gps-nmea] NTRIP enabled but host/mountpoint missing"; return; }
  echo "[gps-nmea] NTRIP -> ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNT} -> ${GPS_PORT}"
  str2str \
    -in "ntrip://${NTRIP_USER}:${NTRIP_PASS}@${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNT}" \
    -out "file://${GPS_PORT}" &
  STR2STR_PID=$!
}

stop_str2str() {
  [ "${STR2STR_PID}" -gt 0 ] && kill -0 "${STR2STR_PID}" 2>/dev/null || return 0
  kill -TERM "${STR2STR_PID}" 2>/dev/null || true
  for _ in 1 2 3 4 5; do kill -0 "${STR2STR_PID}" 2>/dev/null || break; sleep 0.5; done
  kill -KILL "${STR2STR_PID}" 2>/dev/null || true
  wait "${STR2STR_PID}" 2>/dev/null || true
  STR2STR_PID=0
}

cleanup() {
  stop_str2str
  kill "${DRIVER_PID}" "${BRIDGE_PID}" 2>/dev/null || true
  exit 0
}
trap cleanup TERM INT

# Give the bridge a beat to open the serial device before NTRIP starts writing.
sleep 3
start_str2str

# Hot-reload NTRIP on config changes (so users editing the YAML via the GUI
# don't need to restart the container).
if [ "$NTRIP_ENABLED" = "true" ] && command -v inotifywait >/dev/null 2>&1; then
  (
    while true; do
      inotifywait -q -e close_write,move_self,delete_self "$CONFIG" || true
      echo "[gps-nmea] config changed — reloading NTRIP"
      NTRIP_HOST=$(parse_yaml ntrip_host)
      NTRIP_PORT=$(parse_yaml ntrip_port)
      NTRIP_USER=$(parse_yaml ntrip_user)
      NTRIP_PASS=$(parse_yaml ntrip_password)
      NTRIP_MOUNT=$(parse_yaml ntrip_mountpoint)
      NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
      stop_str2str
      sleep 0.5
      start_str2str
    done
  ) &
fi

wait -n || true
cleanup
