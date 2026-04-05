#!/usr/bin/env bash

check_devices() {
  step "Check: Hardware devices"

  : "${GPS_PORT:=/dev/gps}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_TYPE:=unknown}"
  : "${LIDAR_ENABLED:=true}"

  local devices=(
    "/dev/mowgli:Mowgli STM32 board"
    "${GPS_PORT}:GPS receiver"
  )

  if [[ "${LIDAR_ENABLED}" == "true" && "${LIDAR_TYPE}" != "none" ]]; then
    devices+=("${LIDAR_PORT}:LiDAR device")
  fi

  for dev_info in "${devices[@]}"; do
    local dev="${dev_info%%:*}"
    local name="${dev_info#*:}"

    if [ -e "$dev" ]; then
      info "$name ($dev)"
    else
      fail "$name ($dev) — not found"

      case "$dev" in
        /dev/mowgli)
          add_issue "Mowgli board not detected. Flash the Mowgli firmware to the STM32 board and connect it via USB."
          echo -e "       ${DIM}Firmware: https://github.com/cedbossneo/Mowgli${NC}"
          echo -e "       ${DIM}Flash with: STM32CubeProgrammer or st-flash${NC}"
          ;;
        *)
          if [[ "$dev" == "$GPS_PORT" ]]; then
            add_issue "GPS not detected. Check your GPS connection and udev rules."
            echo -e "       ${DIM}Expected port: ${GPS_PORT}${NC}"
            echo -e "       ${DIM}Check: ls -l ${GPS_PORT}${NC}"
            echo -e "       ${DIM}Then verify udev rules: cat /etc/udev/rules.d/50-mowgli.rules${NC}"
          elif [[ "$dev" == "$LIDAR_PORT" ]]; then
            add_issue "LiDAR device not detected. Check the selected UART/USB port and LIDAR_PORT in .env."
            echo -e "       ${DIM}Expected port: ${LIDAR_PORT}${NC}"
            echo -e "       ${DIM}LiDAR type: ${LIDAR_TYPE}${NC}"
            echo -e "       ${DIM}If using a different port, edit LIDAR_PORT in .env${NC}"
          fi
          ;;
      esac
    fi
  done
}

check_containers() {
  step "Check: Containers"

  cd "$INSTALL_DIR" 2>/dev/null || cd "$(dirname "$(realpath "$0")")"

  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_TYPE:=unknown}"

  local services=(mowgli gps gui mosquitto)

  if [[ "${LIDAR_ENABLED}" == "true" && "${LIDAR_TYPE}" != "none" ]]; then
    services+=(lidar)
  fi

  for svc in "${services[@]}"; do
    local container
    case "$svc" in
      mowgli)    container="mowgli-ros2" ;;
      gps)       container="mowgli-gps" ;;
      lidar)     container="mowgli-lidar" ;;
      gui)       container="openmower-gui" ;;
      mosquitto) container="mowgli-mqtt" ;;
    esac

    local status
    status=$(docker inspect -f '{{.State.Status}}' "$container" 2>/dev/null || echo "missing")

    if [[ "$status" == "running" ]]; then
      local uptime
      uptime=$(docker inspect -f '{{.State.StartedAt}}' "$container" 2>/dev/null | cut -dT -f2 | cut -d. -f1)
      info "$svc ($container) — running since $uptime"
    else
      fail "$svc ($container) — $status"
      if [[ "$status" == "missing" ]]; then
        add_issue "Container $container not found. Run: docker compose up -d"
      else
        add_issue "Container $container is $status. Check logs: docker compose logs $svc --tail 30"
      fi
    fi
  done

  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local dead_nodes
    dead_nodes=$(docker compose logs mowgli --tail 200 2>&1 \
      | grep -oP "process has died.*cmd '([^']+)'" \
      | grep -oP "(?<=cmd ')[^']+" \
      | xargs -I{} basename {} 2>/dev/null \
      | sort -u || true)

    if [[ -n "$dead_nodes" ]]; then
      warn "Crashed nodes inside mowgli-ros2:"
      echo "$dead_nodes" | while read -r node; do
        echo -e "       ${RED}$node${NC}"
      done
      add_issue "Some ROS nodes crashed inside mowgli-ros2. Check: docker compose logs mowgli | grep 'process has died'"
    fi
  fi
}

check_firmware() {
  step "Check: Mowgli firmware"

  if ! docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 not running — skipping firmware check"
    return
  fi

  local status_data
  status_data=$(docker exec mowgli-ros2 bash -c \
    "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /status --once 2>/dev/null" \
    2>/dev/null || echo "")

  if [[ -z "$status_data" ]]; then
    fail "No data on /status — hardware bridge cannot communicate with Mowgli board"
    add_issue "Mowgli firmware not responding. Ensure the STM32 is flashed with Mowgli firmware and /dev/mowgli is accessible."
    echo -e "       ${DIM}Flash firmware: https://github.com/cedbossneo/Mowgli${NC}"
    echo -e "       ${DIM}Check serial: docker compose logs mowgli | grep hardware_bridge${NC}"
  else
    local mower_status
    mower_status=$(echo "$status_data" | grep "mower_status:" | awk '{print $2}')

    if [[ "$mower_status" == "255" ]]; then
      warn "Firmware reporting mower_status=255 (not initialised)"
      add_issue "Mowgli board is connected but reporting uninitialised state. Press the power button on the mower or check the firmware."
    else
      info "Firmware responding — mower_status=$mower_status"
    fi

    local charging esc_power
    charging=$(echo "$status_data" | grep "is_charging:" | awk '{print $2}')
    esc_power=$(echo "$status_data" | grep "esc_power:" | awk '{print $2}')
    echo -e "       ${DIM}Charging: $charging | ESC power: $esc_power${NC}"
  fi
}

check_gps() {
  step "Check: GPS"

  if ! docker inspect -f '{{.State.Status}}' mowgli-gps 2>/dev/null | grep -q running; then
    warn "mowgli-gps not running — skipping GPS check"
    return
  fi

  local fix_data
  fix_data=$(docker exec mowgli-gps bash -c \
    "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /ublox_gps_node/fix --once 2>/dev/null" \
    2>/dev/null || echo "")

  if [[ -z "$fix_data" ]]; then
    fail "No GPS fix data on /ublox_gps_node/fix"
    add_issue "GPS not publishing. Check: docker compose logs gps --tail 30"
    return
  fi

  local lat lon status_val cov
  lat=$(echo "$fix_data" | grep "latitude:" | awk '{print $2}')
  lon=$(echo "$fix_data" | grep "longitude:" | awk '{print $2}')
  status_val=$(echo "$fix_data" | grep -A1 "status:" | grep "status:" | tail -1 | awk '{print $2}')
  cov=$(echo "$fix_data" | grep -m1 "^- " | awk '{print $2}')

  local accuracy
  accuracy=$(echo "$cov" | awk '{printf "%.2f", sqrt($1)}')

  local acc_num
  acc_num=$(echo "$accuracy" | awk '{printf "%d", $1 * 100}')

  if [[ "$status_val" == "2" ]] || [[ "$acc_num" -le 5 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FIXED — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "1" ]] || [[ "$acc_num" -le 20 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FLOAT — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "0" ]]; then
    warn "GPS: Standard fix — ${accuracy}m accuracy (no RTK corrections)"
  else
    fail "GPS: No fix (status=$status_val)"
  fi

  local ntrip_logs
  ntrip_logs=$(docker compose logs gps --tail 50 2>&1)

  if echo "$ntrip_logs" | grep -q "Connected to http"; then
    local ntrip_url
    ntrip_url=$(echo "$ntrip_logs" | grep -oP "Connected to \K[^ ]+" | tail -1)
    info "NTRIP connected: $ntrip_url"
  elif echo "$ntrip_logs" | grep -q "Unable to connect"; then
    fail "NTRIP connection failed"
    add_issue "NTRIP cannot connect. Check ntrip_host and ntrip_mountpoint in config/mowgli/mowgli_robot.yaml"
  elif echo "$ntrip_logs" | grep -q "Network is unreachable"; then
    fail "NTRIP: network unreachable"
    add_issue "No internet connection for NTRIP. Check your network configuration."
  fi

  if echo "$ntrip_logs" | grep -q "Forwarded.*RTCM messages"; then
    local rtcm_count
    rtcm_count=$(echo "$ntrip_logs" | grep -oP "Forwarded \K\d+" | tail -1)
    info "RTCM bridge: $rtcm_count corrections forwarded to GPS"
  elif echo "$ntrip_logs" | grep -q "NTRIP enabled: true"; then
    warn "RTCM bridge not forwarding yet — RTK may take a few minutes to converge"
  fi

  local datum_lat datum_lon
  datum_lat=$(grep "datum_lat:" "$INSTALL_DIR/config/mowgli/mowgli_robot.yaml" 2>/dev/null | head -1 | awk '{print $2}')
  datum_lon=$(grep "datum_lon:" "$INSTALL_DIR/config/mowgli/mowgli_robot.yaml" 2>/dev/null | head -1 | awk '{print $2}')

  if [[ "$datum_lat" == "0.0" || "$datum_lon" == "0.0" || -z "$datum_lat" ]]; then
    fail "GPS datum is 0.0/0.0 — robot position will be wrong"
    add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml to your docking station coordinates"
  else
    info "Datum: $datum_lat, $datum_lon"
  fi
}

check_lidar() {
  step "Check: LiDAR"

  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_TYPE:=unknown}"
  : "${LIDAR_BAUD:=?}"

  if [[ "${LIDAR_ENABLED}" != "true" || "${LIDAR_TYPE}" == "none" ]]; then
    info "LiDAR disabled — skipping LiDAR checks"
    return
  fi

  info "LiDAR config: type=${LIDAR_TYPE} port=${LIDAR_PORT} baud=${LIDAR_BAUD}"

  if ! docker inspect -f '{{.State.Status}}' mowgli-lidar 2>/dev/null | grep -q running; then
    warn "mowgli-lidar not running — skipping LiDAR check"
    return
  fi

  local scan_check
  scan_check=$(docker exec mowgli-ros2 bash -c \
    "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /scan 2>/dev/null" \
    2>/dev/null || echo "")

  local pub_count
  pub_count=$(echo "$scan_check" | grep "Publisher count:" | awk '{print $3}')

  if [[ "$pub_count" -ge 1 ]] 2>/dev/null; then
    info "LiDAR publishing on /scan ($pub_count publisher)"
  else
    fail "No publisher on /scan — LiDAR data not reaching ROS"
    add_issue "LiDAR not publishing. Check: docker compose logs lidar --tail 20"
    echo -e "       ${DIM}Expected serial port: ${LIDAR_PORT}${NC}"
  fi
}

check_slam() {
  step "Check: SLAM & Navigation"

  if ! docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 not running — skipping SLAM check"
    return
  fi

  local slam_state
  slam_state=$(docker exec mowgli-ros2 bash -c \
    "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /slam_toolbox/pose 2>/dev/null" \
    2>/dev/null || echo "")

  if echo "$slam_state" | grep -q "Publisher count: [1-9]"; then
    info "SLAM toolbox active"
  else
    warn "SLAM toolbox not publishing poses"
    add_issue "SLAM not active. It needs LiDAR data on /scan to start mapping."
  fi

  local map_exists
  map_exists=$(docker exec mowgli-ros2 bash -c \
    "ls /ros2_ws/maps/garden_map.posegraph 2>/dev/null && echo yes || echo no" \
    2>/dev/null || echo "no")

  if [[ "$map_exists" == *"yes"* ]]; then
    info "Saved map found (lifelong/localisation mode available)"
  else
    warn "No saved map — SLAM running in mapping mode"
    echo -e "       ${DIM}Drive the mower around to create a map, then dock to save it${NC}"
  fi
}

check_rangefinders() {
  step "Check: Rangefinders"

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    if [ -e "${TFLUNA_FRONT_PORT:-/dev/tfluna_front}" ]; then
      info "TF-Luna front detected (${TFLUNA_FRONT_PORT})"
    else
      fail "TF-Luna front not detected (${TFLUNA_FRONT_PORT})"
      add_issue "TF-Luna front not detected. Check selected UART and TFLUNA_FRONT_PORT in .env."
    fi
  fi

  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    if [ -e "${TFLUNA_EDGE_PORT:-/dev/tfluna_edge}" ]; then
      info "TF-Luna edge detected (${TFLUNA_EDGE_PORT})"
    else
      fail "TF-Luna edge not detected (${TFLUNA_EDGE_PORT})"
      add_issue "TF-Luna edge not detected. Check selected UART and TFLUNA_EDGE_PORT in .env."
    fi
  fi
}

check_gui() {
  step "Check: GUI & connectivity"

  if ! docker inspect -f '{{.State.Status}}' openmower-gui 2>/dev/null | grep -q running; then
    fail "openmower-gui not running"
    add_issue "GUI container not running. Run: docker compose up -d gui"
    return
  fi

  local ip
  ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")

  if curl -sf -o /dev/null --connect-timeout 3 "http://$ip:80" 2>/dev/null || \
     curl -sf -o /dev/null --connect-timeout 3 "http://localhost:80" 2>/dev/null; then
    info "GUI accessible at http://$ip"
  else
    warn "GUI might be starting up — try http://$ip in your browser"
  fi

  local rb_info
  rb_info=$(docker exec mowgli-ros2 bash -c \
    "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null" \
    2>/dev/null | grep rosbridge || echo "")

  if [[ -n "$rb_info" ]]; then
    info "Rosbridge WebSocket active (ws://$ip:9090)"
  else
    warn "Rosbridge node not found — GUI may not receive live data"
  fi

  echo ""
  echo -e "  ${BOLD}Access points:${NC}"
  echo -e "    GUI:       ${CYAN}http://$ip${NC}"
  echo -e "    Foxglove:  ${CYAN}ws://$ip:8765${NC}"
  echo -e "    Rosbridge: ${CYAN}ws://$ip:9090${NC}"
  echo -e "    MQTT:      ${CYAN}$ip:1883${NC}"
}