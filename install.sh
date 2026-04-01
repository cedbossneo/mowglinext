#!/usr/bin/env bash
# =============================================================================
# Mowgli ROS2 v3 — Interactive Install / Upgrade / Diagnose Script
#
# Usage:
#   curl -sSL https://raw.githubusercontent.com/cedbossneo/mowgli-docker/v3/install.sh | bash
#   # or locally:
#   ./install.sh            # full install + diagnostics
#   ./install.sh --check    # diagnostics only (skip install)
# =============================================================================

set -euo pipefail

# ── Colours & helpers ───────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

info()  { echo -e "  ${GREEN}OK${NC}  $*"; }
warn()  { echo -e "  ${YELLOW}!!${NC}  $*"; }
fail()  { echo -e "  ${RED}FAIL${NC}  $*"; }
error() { echo -e "${RED}[x]${NC} $*" >&2; }
step()  { echo -e "\n${CYAN}${BOLD}── $* ──${NC}"; }
ask()   { echo -en "${BOLD}$1${NC} "; }

# Prompt with default value.  Usage: result=$(prompt "Question?" "default")
prompt() {
  local answer
  ask "$1 [${2:-}]:"
  read -r answer
  echo "${answer:-$2}"
}

# Yes/no prompt.  Usage: if confirm "Continue?"; then ...
confirm() {
  local answer
  ask "$1 [Y/n]:"
  read -r answer
  [[ "${answer,,}" != "n" ]]
}

command_exists() { command -v "$1" &>/dev/null; }

# ── Configuration ───────────────────────────────────────────────────────────
REPO_URL="https://github.com/cedbossneo/mowgli-docker.git"
REPO_BRANCH="v3"
INSTALL_DIR="${MOWGLI_HOME:-$HOME/mowgli-docker}"
UDEV_RULES_FILE="/etc/udev/rules.d/50-mowgli.rules"

MOWGLI_ROS2_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-ros2/mowgli-ros2:main"
GPS_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-docker/gps:v3"
LIDAR_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-docker/lidar:v3"
GUI_IMAGE_DEFAULT="ghcr.io/cedbossneo/openmower-gui:v3"

CHECK_ONLY=false
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=true

# Track issues for the final summary
ISSUES=()
add_issue() { ISSUES+=("$1"); }

# ── Sudo helper ─────────────────────────────────────────────────────────────
require_root_for() {
  if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
  else
    SUDO=""
  fi
}

# ═══════════════════════════════════════════════════════════════════════════
# INSTALL STEPS (skipped with --check)
# ═══════════════════════════════════════════════════════════════════════════

install_docker() {
  step "1/6  Docker"

  if command_exists docker; then
    info "Docker $(docker --version 2>/dev/null | grep -oP '[\d.]+'| head -1)"
  else
    info "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    info "Docker installed"
  fi

  if docker compose version &>/dev/null; then
    info "Docker Compose $(docker compose version --short 2>/dev/null)"
  else
    error "Docker Compose v2 not found. Install docker-compose-plugin."
    exit 1
  fi

  if ! groups "$USER" | grep -qw docker 2>/dev/null; then
    require_root_for "docker group"
    $SUDO usermod -aG docker "$USER"
    warn "Added $USER to docker group — log out/in for it to take effect"
  fi
}

install_udev_rules() {
  step "2/6  udev rules"
  require_root_for "udev rules"

  local rules
  rules=$(cat <<'RULES'
# Mowgli ROS2 v3 — device symlinks
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"
RULES
)

  if [ -f "$UDEV_RULES_FILE" ] && [ "$(cat "$UDEV_RULES_FILE")" = "$rules" ]; then
    info "udev rules up to date"
  else
    echo "$rules" | $SUDO tee "$UDEV_RULES_FILE" > /dev/null
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    info "udev rules installed"
  fi
}

setup_directory() {
  step "3/6  Repository"

  if [ -d "$INSTALL_DIR/.git" ]; then
    info "Updating $INSTALL_DIR"
    git -C "$INSTALL_DIR" fetch origin
    git -C "$INSTALL_DIR" reset --hard "origin/$REPO_BRANCH"
  elif [ -d "$INSTALL_DIR" ] && [ -f "$INSTALL_DIR/docker-compose.yaml" ]; then
    info "Existing installation at $INSTALL_DIR (not a git repo, skipping)"
  else
    info "Cloning mowgli-docker to $INSTALL_DIR"
    git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" "$INSTALL_DIR"
  fi
}

setup_env() {
  step "4/6  Environment (.env)"

  local env_file="$INSTALL_DIR/.env"

  if [ -f "$env_file" ]; then
    info "Existing .env found — checking for missing keys"
    local needs_update=false

    for pair in \
      "MOWGLI_ROS2_IMAGE=$MOWGLI_ROS2_IMAGE_DEFAULT" \
      "GPS_IMAGE=$GPS_IMAGE_DEFAULT" \
      "LIDAR_IMAGE=$LIDAR_IMAGE_DEFAULT" \
      "GUI_IMAGE=$GUI_IMAGE_DEFAULT" \
      "ROS_DOMAIN_ID=0" \
      "LIDAR_PORT=/dev/ttyS1" \
      "LIDAR_BAUD=230400" \
    ; do
      local key="${pair%%=*}"
      if ! grep -q "^${key}=" "$env_file"; then
        echo "$pair" >> "$env_file"
        needs_update=true
      fi
    done

    if $needs_update; then
      warn "Added missing keys to .env"
    else
      info ".env is complete"
    fi
  else
    cat > "$env_file" <<EOF
ROS_DOMAIN_ID=0
MOWER_IP=10.0.0.161
LIDAR_PORT=/dev/ttyS1
LIDAR_BAUD=230400
MOWGLI_ROS2_IMAGE=$MOWGLI_ROS2_IMAGE_DEFAULT
GPS_IMAGE=$GPS_IMAGE_DEFAULT
LIDAR_IMAGE=$LIDAR_IMAGE_DEFAULT
GUI_IMAGE=$GUI_IMAGE_DEFAULT
EOF
    info "Created $env_file"
  fi
}

# ── Interactive configuration ───────────────────────────────────────────────
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
  if [ -f "$yaml_file" ]; then
    info "mowgli_robot.yaml already exists"
    if ! confirm "Do you want to reconfigure it?"; then
      return
    fi
  fi

  echo ""
  echo -e "${BOLD}Let's configure your mower. You can change these later in:${NC}"
  echo -e "  ${DIM}$yaml_file${NC}"
  echo ""

  # GPS datum — will be overridden by auto-detect if available
  echo -e "${CYAN}GPS Datum${NC} — coordinates of your docking station (WGS84)"
  echo -e "${DIM}Find them on Google Maps: right-click your dock location > copy coordinates${NC}"
  echo -e "${DIM}Leave as 0.0 to auto-detect from GPS after startup (mower must be on dock)${NC}"
  local datum_lat datum_lon
  datum_lat=$(prompt "  Latitude?" "0.0")
  datum_lon=$(prompt "  Longitude?" "0.0")

  # NTRIP
  echo ""
  echo -e "${CYAN}NTRIP RTK${NC} — correction stream for centimetre-level GPS accuracy"
  echo -e "${DIM}Free in France: caster.centipede.fr (user: centipede / pass: centipede)${NC}"
  local ntrip_enabled="false"
  local ntrip_host="" ntrip_port="2101" ntrip_user="" ntrip_password="" ntrip_mountpoint=""

  if confirm "  Enable NTRIP corrections?"; then
    ntrip_enabled="true"
    ntrip_host=$(prompt "  NTRIP host?" "caster.centipede.fr")
    ntrip_port=$(prompt "  NTRIP port?" "2101")
    ntrip_user=$(prompt "  NTRIP user?" "centipede")
    ntrip_password=$(prompt "  NTRIP password?" "centipede")
    ntrip_mountpoint=$(prompt "  Mountpoint (base station)?" "")
    if [[ -z "$ntrip_mountpoint" ]]; then
      warn "No mountpoint set — NTRIP won't connect"
      add_issue "Set ntrip_mountpoint in $yaml_file to your nearest base station"
    fi
  fi

  # LiDAR position and orientation
  echo ""
  echo -e "${CYAN}LiDAR mounting${NC} — position relative to base_link centre (metres)"
  echo -e "${DIM}x=forward, y=left, z=up. yaw in radians (3.14159 = 180 degrees)${NC}"
  local lidar_x lidar_y lidar_z lidar_yaw
  lidar_x=$(prompt "  LiDAR x (forward)?" "0.20")
  lidar_y=$(prompt "  LiDAR y (left)?" "0.0")
  lidar_z=$(prompt "  LiDAR z (height)?" "0.22")
  lidar_yaw=$(prompt "  LiDAR yaw (rotation)?" "0.0")

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

# Write config files using CONFIG_* variables
write_config() {
  local yaml_file="$INSTALL_DIR/config/mowgli/mowgli_robot.yaml"

  cat > "$yaml_file" <<EOF
# Mowgli ROS2 — Site-specific configuration
# Full reference: docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/mowgli_robot.yaml

mowgli:
  ros__parameters:
    datum_lat: $CONFIG_DATUM_LAT
    datum_lon: $CONFIG_DATUM_LON

    gps_port: "/dev/gps"
    gps_baudrate: 921600

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
export OM_GPS_PROTOCOL=UBX
export OM_GPS_PORT=/dev/gps
export OM_GPS_BAUDRATE=921600
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

# ── Auto-detect datum and dock from live GPS + charging status ──────────────
auto_detect_position() {
  step "Auto-detect: GPS datum & dock position"

  # Only auto-detect if datum was left at 0.0
  if [[ "$CONFIG_DATUM_LAT" != "0.0" && "$CONFIG_DATUM_LAT" != "0" ]]; then
    info "Datum already set ($CONFIG_DATUM_LAT, $CONFIG_DATUM_LON) — skipping auto-detect"
    return
  fi

  # Check if GPS container is running
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

  # Check if mower is charging (= on the dock)
  local is_charging="false"
  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local status_data
    status_data=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /status --once 2>/dev/null" 2>/dev/null || echo "")
    is_charging=$(echo "$status_data" | grep "is_charging:" | awk '{print $2}')
  fi

  # Set datum to current GPS position
  CONFIG_DATUM_LAT="$lat"
  CONFIG_DATUM_LON="$lon"
  info "Datum auto-set to GPS position: $lat, $lon"

  # If charging, also set dock position (0,0 relative to datum = the dock itself)
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

  # Rewrite config files with updated values
  write_config
  info "Config updated with auto-detected position"

  # Restart containers to pick up new config
  echo -e "${DIM}Restarting containers with new config...${NC}"
  cd "$INSTALL_DIR"
  docker compose restart gps mowgli 2>&1 | tail -3
  sleep 10
}

pull_and_start() {
  step "6/6  Pull & start"

  cd "$INSTALL_DIR"
  info "Pulling latest images..."
  docker compose pull 2>&1 | tail -5
  info "Starting the stack..."
  docker compose up -d 2>&1 | tail -5
  info "Stack started — waiting 20s for containers to initialise..."
  sleep 20
}

# ═══════════════════════════════════════════════════════════════════════════
# E2E HEALTH CHECKS
# ═══════════════════════════════════════════════════════════════════════════

check_devices() {
  step "Check: Hardware devices"

  for dev_info in "/dev/mowgli:Mowgli STM32 board" "/dev/gps:GPS receiver (u-blox F9P)" "/dev/ttyS1:LiDAR serial port"; do
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
        /dev/gps)
          add_issue "GPS not detected. Connect the u-blox F9P (or compatible) via USB."
          echo -e "       ${DIM}If connected, check: lsusb | grep -i u-blox${NC}"
          echo -e "       ${DIM}Then verify udev rules: cat /etc/udev/rules.d/50-mowgli.rules${NC}"
          ;;
        /dev/ttyS1)
          add_issue "LiDAR serial port not found. Check if your board has /dev/ttyS1 (hardware UART)."
          echo -e "       ${DIM}If using a different port, edit LIDAR_PORT in .env${NC}"
          ;;
      esac
    fi
  done
}

check_containers() {
  step "Check: Containers"

  cd "$INSTALL_DIR" 2>/dev/null || cd "$(dirname "$(realpath "$0")")"

  local all_ok=true
  for svc in mowgli gps lidar gui mosquitto; do
    local container
    case $svc in
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
      all_ok=false
      if [[ "$status" == "missing" ]]; then
        add_issue "Container $container not found. Run: docker compose up -d"
      else
        add_issue "Container $container is $status. Check logs: docker compose logs $svc --tail 30"
      fi
    fi
  done

  # Check for crashed ROS nodes inside mowgli
  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local dead_nodes
    dead_nodes=$(docker compose logs mowgli --tail 200 2>&1 | grep -oP "process has died.*cmd '([^']+)'" | grep -oP "(?<=cmd ')[^']+" | xargs -I{} basename {} 2>/dev/null | sort -u || true)
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

  # Check if hardware_bridge is publishing /status
  local status_data
  status_data=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /status --once 2>/dev/null" 2>/dev/null || echo "")

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

    # Show key status fields
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

  # Check GPS fix
  local fix_data
  fix_data=$(docker exec mowgli-gps bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /ublox_gps_node/fix --once 2>/dev/null" 2>/dev/null || echo "")

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

  # NavSatStatus: -1=no fix, 0=fix, 1=SBAS(RTK float), 2=GBAS(RTK fixed)
  # Some ublox driver versions don't map carrSoln correctly, so also check accuracy
  local acc_num
  acc_num=$(echo "$accuracy" | awk '{printf "%d", $1 * 100}')  # cm

  if [[ "$status_val" == "2" ]] || [[ "$acc_num" -le 5 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FIXED — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "1" ]] || [[ "$acc_num" -le 20 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FLOAT — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "0" ]]; then
    warn "GPS: Standard fix — ${accuracy}m accuracy (no RTK corrections)"
  else
    fail "GPS: No fix (status=$status_val)"
  fi

  # Check NTRIP
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

  # Check datum
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

  if ! docker inspect -f '{{.State.Status}}' mowgli-lidar 2>/dev/null | grep -q running; then
    warn "mowgli-lidar not running — skipping LiDAR check"
    return
  fi

  local scan_check
  scan_check=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /scan 2>/dev/null" 2>/dev/null || echo "")

  local pub_count
  pub_count=$(echo "$scan_check" | grep "Publisher count:" | awk '{print $3}')

  if [[ "$pub_count" -ge 1 ]] 2>/dev/null; then
    info "LiDAR publishing on /scan ($pub_count publisher)"
  else
    fail "No publisher on /scan — LiDAR data not reaching ROS"
    add_issue "LiDAR not publishing. Check: docker compose logs lidar --tail 20"
    echo -e "       ${DIM}Verify serial port: ls -l \$(grep LIDAR_PORT .env | cut -d= -f2)${NC}"
  fi
}

check_slam() {
  step "Check: SLAM & Navigation"

  if ! docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 not running — skipping SLAM check"
    return
  fi

  # Check SLAM toolbox
  local slam_state
  slam_state=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /slam_toolbox/pose 2>/dev/null" 2>/dev/null || echo "")

  if echo "$slam_state" | grep -q "Publisher count: [1-9]"; then
    info "SLAM toolbox active"
  else
    warn "SLAM toolbox not publishing poses"
    add_issue "SLAM not active. It needs LiDAR data on /scan to start mapping."
  fi

  # Check if map exists
  local map_exists
  map_exists=$(docker exec mowgli-ros2 bash -c "ls /ros2_ws/maps/garden_map.posegraph 2>/dev/null && echo yes || echo no" 2>/dev/null || echo "no")

  if [[ "$map_exists" == *"yes"* ]]; then
    info "Saved map found (lifelong/localisation mode available)"
  else
    warn "No saved map — SLAM running in mapping mode"
    echo -e "       ${DIM}Drive the mower around to create a map, then dock to save it${NC}"
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

  # Check if GUI port is listening
  if curl -sf -o /dev/null --connect-timeout 3 "http://$ip:80" 2>/dev/null || \
     curl -sf -o /dev/null --connect-timeout 3 "http://localhost:80" 2>/dev/null; then
    info "GUI accessible at http://$ip"
  else
    warn "GUI might be starting up — try http://$ip in your browser"
  fi

  # Check rosbridge
  local rb_info
  rb_info=$(docker exec mowgli-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null" 2>/dev/null | grep rosbridge || echo "")
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

# ═══════════════════════════════════════════════════════════════════════════
# FINAL SUMMARY
# ═══════════════════════════════════════════════════════════════════════════

print_summary() {
  echo ""
  echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"

  if [[ ${#ISSUES[@]} -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}  All checks passed! Your mower is ready.${NC}"
    echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"
    echo ""
    echo -e "  ${BOLD}Next steps:${NC}"
    echo -e "  1. Open the GUI in your browser"
    echo -e "  2. Record your garden boundary (Areas tab)"
    echo -e "  3. Set dock position (drive mower to dock, save pose)"
    echo -e "  4. Start mowing!"
  else
    echo -e "${YELLOW}${BOLD}  ${#ISSUES[@]} issue(s) to resolve:${NC}"
    echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"
    echo ""
    local i=1
    for issue in "${ISSUES[@]}"; do
      echo -e "  ${YELLOW}${i}.${NC} $issue"
      ((i++))
    done
    echo ""
    echo -e "  ${DIM}Fix these issues, then re-run:${NC}"
    echo -e "  ${BOLD}./install.sh --check${NC}"
  fi
  echo ""
}

# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

main() {
  echo ""
  echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════╗${NC}"
  echo -e "${GREEN}${BOLD}║       Mowgli ROS2 v3 — Setup & Diagnose     ║${NC}"
  echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════╝${NC}"
  echo ""

  if ! $CHECK_ONLY; then
    install_docker
    install_udev_rules
    setup_directory
    setup_env
    interactive_config
    write_config
    pull_and_start
    auto_detect_position
  else
    INSTALL_DIR="${MOWGLI_HOME:-$HOME/mowgli-docker}"
    if [ ! -f "$INSTALL_DIR/docker-compose.yaml" ]; then
      error "No installation found at $INSTALL_DIR — run without --check first"
      exit 1
    fi
    cd "$INSTALL_DIR"
    echo -e "${DIM}Running diagnostics on $INSTALL_DIR${NC}"
  fi

  echo ""
  echo -e "${CYAN}${BOLD}══ System Health Check ══${NC}"

  check_devices
  check_containers
  check_firmware
  check_gps
  check_lidar
  check_slam
  check_gui

  print_summary
}

main "$@"
