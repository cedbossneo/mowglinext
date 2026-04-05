#interactive_config
#write_config
#gps_port: "${GPS_PORT:-/dev/gps}"
#gps_baudrate: ${GPS_BAUD:-460800}
#!/usr/bin/env bash

# ── Configuration globale ───────────────────────────────────────────────────

REPO_URL="https://github.com/Mowglifrenchtouch/mowglinext.git"
REPO_BRANCH="main"
REPO_DIR="${MOWGLI_HOME:-$HOME/mowglinext}"
DOCKER_SUBDIR="docker"
INSTALL_DIR="${REPO_DIR}/${DOCKER_SUBDIR}"
UDEV_RULES_FILE="/etc/udev/rules.d/50-mowgli.rules"

MOWGLI_ROS2_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-ros2/mowgli-ros2:main"
GPS_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-docker/gps:v3"
LIDAR_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-docker/lidar:v3"
GUI_IMAGE_DEFAULT="ghcr.io/cedbossneo/openmower-gui:v3"

CHECK_ONLY=false
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=true

# Track issues for the final summary
ISSUES=()

add_issue() {
  ISSUES+=("$1")
}

interactive_config() {
  step "5/6  Mower configuration"

  local yaml_file="$INSTALL_DIR/config/mowgli/mowgli_robot.yaml"
  mkdir -p "$INSTALL_DIR/config/mowgli"
  mkdir -p "$INSTALL_DIR/config/om"
  mkdir -p "$INSTALL_DIR/config/mqtt"
  mkdir -p "$INSTALL_DIR/config/db"

  # Mosquitto
  if [ ! -f "$INSTALL_DIR/config/mqtt/mosquitto.conf" ]; then
    cat > "$INSTALL_DIR/config/mqtt/mosquitto.conf" <<'EOF'
log_type error
log_type warning
log_type information
listener 1883
allow_anonymous true
listener 9001
protocol websockets
allow_anonymous true
EOF
    info "Created mosquitto.conf"
  fi

  # If config already exists, ask whether to reconfigure
  SKIP_WRITE_CONFIG=false
  if [ -f "$yaml_file" ]; then
    info "mowgli_robot.yaml already exists"
    if ! confirm "Do you want to reconfigure it?"; then
      SKIP_WRITE_CONFIG=true
      return
    fi
  fi

  echo ""
  echo -e "${BOLD}Let's configure your mower. You can change these later in:${NC}"
  echo -e "  ${DIM}$yaml_file${NC}"
  echo ""

  # GPS datum
  echo -e "${CYAN}GPS Datum${NC} — map origin coordinates (should be near your dock)"
  echo ""
  echo -e "  ${BOLD}1)${NC} Auto-detect from GPS after startup (mower must be on the dock)"
  echo -e "  ${BOLD}2)${NC} Enter coordinates manually"
  echo -e "  ${BOLD}3)${NC} Skip (configure later)"
  echo ""
  local datum_lat="0.0" datum_lon="0.0"
  prompt "  Choose" "1"
  local datum_choice="$REPLY"

  case "$datum_choice" in
    2)
      echo -e "  ${DIM}Find coordinates on Google Maps: right-click dock > copy coordinates${NC}"
      prompt "  Latitude?" "0.0"
      datum_lat="$REPLY"
      prompt "  Longitude?" "0.0"
      datum_lon="$REPLY"
      if [[ "$datum_lat" == "0.0" || "$datum_lon" == "0.0" ]]; then
        warn "Datum is 0.0 — GPS localisation won't work"
        add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
      fi
      ;;
    1)
      info "Datum will be auto-detected from GPS after startup"
      ;;
    *)
      warn "Datum skipped — you must set it before mowing"
      add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
      ;;
  esac

  # NTRIP
  echo ""
  echo -e "${CYAN}NTRIP RTK${NC} — correction stream for centimetre-level GPS accuracy"
  echo -e "${DIM}Free in France: caster.centipede.fr (user: centipede / pass: centipede)${NC}"
  echo -e "${DIM}Find your nearest base station at https://centipede.fr${NC}"
  local ntrip_enabled="false"
  local ntrip_host="" ntrip_port="2101" ntrip_user="" ntrip_password="" ntrip_mountpoint=""

  if confirm "  Enable NTRIP corrections?"; then
    ntrip_enabled="true"
    echo ""
    echo -e "  ${DIM}Enter NTRIP parameters (press Enter to accept defaults):${NC}"
    prompt "    Host?" "caster.centipede.fr"
    ntrip_host="$REPLY"
    prompt "    Port?" "2101"
    ntrip_port="$REPLY"
    prompt "    User?" "centipede"
    ntrip_user="$REPLY"
    prompt "    Password?" "centipede"
    ntrip_password="$REPLY"
    prompt "    Mountpoint (nearest base station)?" ""
    ntrip_mountpoint="$REPLY"

    if [[ -z "$ntrip_mountpoint" ]]; then
      warn "No mountpoint set — NTRIP won't connect without one"
      add_issue "Set ntrip_mountpoint in $yaml_file to your nearest base station"
    fi
  fi

  # LiDAR position and orientation
  echo ""
  echo -e "${CYAN}LiDAR mounting${NC} — position relative to base_link centre (metres)"
  echo -e "${DIM}x=forward, y=left, z=up. yaw in radians (3.14159 = 180 degrees)${NC}"
  prompt "  LiDAR x (forward)?" "0.20"
  local lidar_x="$REPLY"
  prompt "  LiDAR y (left)?" "0.0"
  local lidar_y="$REPLY"
  prompt "  LiDAR z (height)?" "0.22"
  local lidar_z="$REPLY"
  prompt "  LiDAR yaw (rotation)?" "0.0"
  local lidar_yaw="$REPLY"

  # Store config vars for write_config and auto_detect
  CONFIG_DATUM_LAT="$datum_lat"
  CONFIG_DATUM_LON="$datum_lon"
  CONFIG_NTRIP_ENABLED="$ntrip_enabled"
  CONFIG_NTRIP_HOST="$ntrip_host"
  CONFIG_NTRIP_PORT="$ntrip_port"
  CONFIG_NTRIP_USER="$ntrip_user"
  CONFIG_NTRIP_PASSWORD="$ntrip_password"
  CONFIG_NTRIP_MOUNTPOINT="$ntrip_mountpoint"
  CONFIG_LIDAR_X="$lidar_x"
  CONFIG_LIDAR_Y="$lidar_y"
  CONFIG_LIDAR_Z="$lidar_z"
  CONFIG_LIDAR_YAW="$lidar_yaw"
  CONFIG_DOCK_X="0.0"
  CONFIG_DOCK_Y="0.0"
  CONFIG_DOCK_YAW="0.0"
}

write_config() {
  local yaml_file="$INSTALL_DIR/config/mowgli/mowgli_robot.yaml"

  : "${GPS_PROTOCOL:=UBX}"
  : "${GPS_PORT:=/dev/gps}"
  : "${GPS_BAUD:=460800}"

  cat > "$yaml_file" <<EOF
# Mowgli ROS2 — Site-specific configuration
# Full reference: docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/mowgli_robot.yaml

mowgli:
  ros__parameters:
    datum_lat: $CONFIG_DATUM_LAT
    datum_lon: $CONFIG_DATUM_LON

    gps_port: "$GPS_PORT"
    gps_baudrate: $GPS_BAUD

    ntrip_enabled: $CONFIG_NTRIP_ENABLED
    ntrip_host: "$CONFIG_NTRIP_HOST"
    ntrip_port: $CONFIG_NTRIP_PORT
    ntrip_user: "$CONFIG_NTRIP_USER"
    ntrip_password: "$CONFIG_NTRIP_PASSWORD"
    ntrip_mountpoint: "$CONFIG_NTRIP_MOUNTPOINT"

    # LiDAR mounting (relative to base_link, metres)
    lidar_x: $CONFIG_LIDAR_X
    lidar_y: $CONFIG_LIDAR_Y
    lidar_z: $CONFIG_LIDAR_Z
    lidar_yaw: $CONFIG_LIDAR_YAW

    dock_pose_x: $CONFIG_DOCK_X
    dock_pose_y: $CONFIG_DOCK_Y
    dock_pose_yaw: $CONFIG_DOCK_YAW

navsat_to_absolute_pose:
  ros__parameters:
    datum_lat: $CONFIG_DATUM_LAT
    datum_lon: $CONFIG_DATUM_LON
EOF

  info "Wrote $yaml_file"

  cat > "$INSTALL_DIR/config/om/mower_config.sh" <<EOF
export OM_DATUM_LAT=$CONFIG_DATUM_LAT
export OM_DATUM_LONG=$CONFIG_DATUM_LON
export OM_GPS_PROTOCOL=$GPS_PROTOCOL
export OM_GPS_PORT=$GPS_PORT
export OM_GPS_BAUDRATE=$GPS_BAUD
export OM_USE_NTRIP=$( [[ "$CONFIG_NTRIP_ENABLED" == "true" ]] && echo "True" || echo "False" )
export OM_NTRIP_HOSTNAME=$CONFIG_NTRIP_HOST
export OM_NTRIP_PORT=$CONFIG_NTRIP_PORT
export OM_NTRIP_USER=$CONFIG_NTRIP_USER
export OM_NTRIP_PASSWORD=$CONFIG_NTRIP_PASSWORD
export OM_NTRIP_ENDPOINT=$CONFIG_NTRIP_MOUNTPOINT
export OM_TOOL_WIDTH=0.13
export OM_ENABLE_MOWER=true
export OM_AUTOMATIC_MODE=0
export OM_BATTERY_FULL_VOLTAGE=28.5
export OM_BATTERY_EMPTY_VOLTAGE=24.0
export OM_BATTERY_CRITICAL_VOLTAGE=23.0
EOF

  info "Wrote mower_config.sh"
}

auto_detect_position() {
  step "Auto-detect: GPS datum & dock position"

  if [[ "$CONFIG_DATUM_LAT" != "0.0" && "$CONFIG_DATUM_LAT" != "0" ]]; then
    info "Datum already set ($CONFIG_DATUM_LAT, $CONFIG_DATUM_LON) — skipping auto-detect"
    return
  fi

  if ! docker inspect -f '{{.State.Status}}' mowgli-gps 2>/dev/null | grep -q running; then
    warn "GPS container not running — cannot auto-detect"
    add_issue "Set datum_lat and datum_lon manually in config/mowgli/mowgli_robot.yaml"
    return
  fi

  echo -e "${DIM}Waiting for GPS fix (up to 60s)...${NC}"

  local fix_data="" lat="" lon="" attempt=0
  while [[ $attempt -lt 12 ]]; do
    fix_data=$(docker exec mowgli-gps bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /ublox_gps_node/fix --once 2>/dev/null" 2>/dev/null || echo "")
    lat=$(echo "$fix_data" | grep "latitude:" | awk '{print $2}')
    lon=$(echo "$fix_data" | grep "longitude:" | awk '{print $2}')

    if [[ -n "$lat" && "$lat" != "0.0" ]]; then
      break
    fi

    ((attempt++))
    sleep 5
  done

  if [[ -z "$lat" || "$lat" == "0.0" ]]; then
    warn "Could not get a GPS fix — set datum manually"
    add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
    return
  fi

  info "GPS position: $lat, $lon"

  local is_charging="false"
  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local status_data
    status_data=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /status --once 2>/dev/null" 2>/dev/null || echo "")
    is_charging=$(echo "$status_data" | grep "is_charging:" | awk '{print $2}')
  fi

  CONFIG_DATUM_LAT="$lat"
  CONFIG_DATUM_LON="$lon"
  info "Datum auto-set to GPS position: $lat, $lon"

  if [[ "$is_charging" == "true" ]]; then
    CONFIG_DOCK_X="0.0"
    CONFIG_DOCK_Y="0.0"
    CONFIG_DOCK_YAW="0.0"
    info "Mower is charging — dock position set to map origin (0, 0)"
    echo -e "       ${DIM}The datum IS your dock, so dock_pose = (0, 0, 0)${NC}"
  else
    warn "Mower is not charging — dock position left at (0, 0)"
    echo -e "       ${DIM}To set dock position later: drive to dock, then read /gps/pose${NC}"
    add_issue "Set dock_pose_x/y/yaw in config/mowgli/mowgli_robot.yaml (drive mower to dock, read the pose)"
  fi

  write_config
  info "Config updated with auto-detected position"

  echo -e "${DIM}Restarting containers with new config...${NC}"
  cd "$INSTALL_DIR"
  docker compose restart gps mowgli 2>&1 | tail -3
  sleep 10
}

run_mower_configuration_step() {
  SKIP_WRITE_CONFIG=false
  interactive_config
  if ! $SKIP_WRITE_CONFIG; then
    write_config
  fi
}

