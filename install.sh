#!/usr/bin/env bash
# =============================================================================
# Mowgli ROS2 v3 — Install / Upgrade Script
#
# Usage:
#   curl -sSL https://raw.githubusercontent.com/cedbossneo/mowgli-ros2/main/mowgli-docker/install.sh | bash
#   # or locally:
#   ./install.sh
#
# What it does:
#   1. Installs Docker + Docker Compose (if missing)
#   2. Adds current user to the docker group
#   3. Installs udev rules for Mowgli, GPS, LiDAR
#   4. Clones or updates the mowgli-docker directory
#   5. Creates .env from template (preserving existing values on upgrade)
#   6. Creates default config files if missing
#   7. Pulls latest images
#   8. Starts the stack
# =============================================================================

set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No colour

info()  { echo -e "${GREEN}[+]${NC} $*"; }
warn()  { echo -e "${YELLOW}[!]${NC} $*"; }
error() { echo -e "${RED}[x]${NC} $*" >&2; }
step()  { echo -e "\n${CYAN}=== $* ===${NC}"; }

# ── Configuration ────────────────────────────────────────────────────────────
REPO_URL="https://github.com/cedbossneo/mowgli-ros2.git"
REPO_BRANCH="v3"
INSTALL_DIR="${MOWGLI_HOME:-$HOME/mowgli-docker}"
UDEV_RULES_FILE="/etc/udev/rules.d/50-mowgli.rules"

MOWGLI_ROS2_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowgli-ros2/mowgli-ros2:main"
GUI_IMAGE_DEFAULT="ghcr.io/cedbossneo/openmower-gui:v3"

# ── Helpers ──────────────────────────────────────────────────────────────────

command_exists() { command -v "$1" &>/dev/null; }

require_root_for() {
  if [ "$(id -u)" -ne 0 ]; then
    warn "$1 requires root privileges"
    SUDO="sudo"
  else
    SUDO=""
  fi
}

# ── Step 1: Docker ──────────────────────────────────────────────────────────
install_docker() {
  step "Docker"

  if command_exists docker; then
    local ver
    ver=$(docker --version 2>/dev/null | head -1)
    info "Docker already installed: $ver"
  else
    info "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    info "Docker installed"
  fi

  # Ensure docker compose v2 is available
  if docker compose version &>/dev/null; then
    local compose_ver
    compose_ver=$(docker compose version --short 2>/dev/null)
    info "Docker Compose: $compose_ver"
  else
    error "Docker Compose v2 not found. Please install docker-compose-plugin."
    exit 1
  fi

  # Add user to docker group if needed
  if ! groups "$USER" | grep -qw docker 2>/dev/null; then
    info "Adding $USER to the docker group..."
    require_root_for "Adding user to docker group"
    $SUDO usermod -aG docker "$USER"
    warn "You may need to log out and back in for group changes to take effect."
  fi
}

# ── Step 2: udev rules ─────────────────────────────────────────────────────
install_udev_rules() {
  step "udev rules"

  require_root_for "Installing udev rules"

  local rules
  rules=$(cat <<'RULES'
# Mowgli ROS2 v3 — device symlinks
# Installed by mowgli-docker/install.sh

# Mowgli STM32 board
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# GPS: simpleRTK2B (u-blox F9P)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
# GPS: RTK1010Board (ESP USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"

# LiDAR: LD19 connected via hardware UART /dev/ttyS1 (230400 baud)
# No udev rule needed — ttyS1 is a fixed kernel device
RULES
)

  if [ -f "$UDEV_RULES_FILE" ]; then
    local existing
    existing=$(cat "$UDEV_RULES_FILE")
    if [ "$existing" = "$rules" ]; then
      info "udev rules already up to date"
      return
    fi
    warn "Updating existing udev rules at $UDEV_RULES_FILE"
  fi

  echo "$rules" | $SUDO tee "$UDEV_RULES_FILE" > /dev/null
  $SUDO udevadm control --reload-rules
  $SUDO udevadm trigger
  info "udev rules installed and reloaded"
}

# ── Step 3: Clone / update repo ────────────────────────────────────────────
setup_directory() {
  step "Mowgli Docker directory"

  if [ -d "$INSTALL_DIR/.git" ]; then
    info "Updating existing installation at $INSTALL_DIR"
    git -C "$INSTALL_DIR" fetch origin
    git -C "$INSTALL_DIR" reset --hard "origin/$REPO_BRANCH"
    info "Updated to latest"
  elif [ -d "$INSTALL_DIR" ] && [ -f "$INSTALL_DIR/docker-compose.yaml" ]; then
    info "Existing installation found at $INSTALL_DIR (not a git repo, skipping update)"
  else
    info "Cloning mowgli-docker to $INSTALL_DIR"
    git clone --branch "$REPO_BRANCH" --depth 1 --filter=blob:none \
      --sparse "$REPO_URL" "$INSTALL_DIR.tmp"
    cd "$INSTALL_DIR.tmp"
    git sparse-checkout set mowgli-docker
    # Move contents up from mowgli-docker/ subfolder
    mv mowgli-docker/* mowgli-docker/.* "$INSTALL_DIR" 2>/dev/null || true
    cd /
    rm -rf "$INSTALL_DIR.tmp"
    info "Cloned to $INSTALL_DIR"
  fi
}

# ── Step 4: .env ────────────────────────────────────────────────────────────
setup_env() {
  step "Environment (.env)"

  local env_file="$INSTALL_DIR/.env"

  if [ -f "$env_file" ]; then
    info "Existing .env found — preserving your settings"

    # Ensure all required keys exist (add missing ones from defaults)
    local needs_update=false

    grep -q "^MOWGLI_ROS2_IMAGE=" "$env_file" || {
      echo "MOWGLI_ROS2_IMAGE=$MOWGLI_ROS2_IMAGE_DEFAULT" >> "$env_file"
      needs_update=true
    }
    grep -q "^GUI_IMAGE=" "$env_file" || {
      echo "GUI_IMAGE=$GUI_IMAGE_DEFAULT" >> "$env_file"
      needs_update=true
    }
    grep -q "^ROS_DOMAIN_ID=" "$env_file" || {
      echo "ROS_DOMAIN_ID=0" >> "$env_file"
      needs_update=true
    }
    grep -q "^LIDAR_PORT=" "$env_file" || {
      echo "LIDAR_PORT=/dev/ttyS1" >> "$env_file"
      needs_update=true
    }
    grep -q "^LIDAR_BAUD=" "$env_file" || {
      echo "LIDAR_BAUD=230400" >> "$env_file"
      needs_update=true
    }

    if $needs_update; then
      warn "Added missing keys to .env"
    else
      info ".env is complete"
    fi
  else
    info "Creating .env from template"
    cat > "$env_file" <<EOF
# Mowgli ROS2 v3 — Environment configuration
# Edit this file, then run: docker compose up -d

# ROS2 Domain ID — all containers must share the same value for DDS discovery
ROS_DOMAIN_ID=0

# IP of the mower Pi (only needed for ser2net mode)
MOWER_IP=10.0.0.161

# LiDAR serial port and baud rate (LD19 on hardware UART)
LIDAR_PORT=/dev/ttyS1
LIDAR_BAUD=230400

# Docker images
MOWGLI_ROS2_IMAGE=$MOWGLI_ROS2_IMAGE_DEFAULT
GUI_IMAGE=$GUI_IMAGE_DEFAULT
EOF
    info "Created $env_file — edit it to set your MOWER_IP"
  fi
}

# ── Step 5: Config files ───────────────────────────────────────────────────
setup_config() {
  step "Configuration files"

  # Ensure config directories exist
  mkdir -p "$INSTALL_DIR/config/mowgli"
  mkdir -p "$INSTALL_DIR/config/om"
  mkdir -p "$INSTALL_DIR/config/mqtt"
  mkdir -p "$INSTALL_DIR/config/db"

  # Mosquitto config
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
    info "Created default mosquitto.conf"
  else
    info "mosquitto.conf already exists"
  fi

  # mowgli_robot.yaml
  if [ ! -f "$INSTALL_DIR/config/mowgli/mowgli_robot.yaml" ]; then
    warn "No mowgli_robot.yaml found — you MUST configure GPS datum and dock pose"
    warn "Edit: $INSTALL_DIR/config/mowgli/mowgli_robot.yaml"
    # The file ships with the repo, so if we cloned it should be there.
    # If running standalone install, create a minimal one.
    if [ ! -f "$INSTALL_DIR/config/mowgli/mowgli_robot.yaml" ]; then
      cat > "$INSTALL_DIR/config/mowgli/mowgli_robot.yaml" <<'EOF'
# Mowgli ROS2 — Site-specific configuration
# MUST be edited for your installation!
# Full reference: docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/mowgli_robot.yaml

mowgli:
  ros__parameters:
    # GPS datum — set to coordinates near your docking station
    datum_lat: 0.0
    datum_lon: 0.0

    # NTRIP RTK correction
    ntrip_enabled: false
    ntrip_host: ""
    ntrip_port: 2101
    ntrip_user: ""
    ntrip_password: ""
    ntrip_mountpoint: ""

    # Dock position in map frame (read from /gps/pose after manual positioning)
    dock_pose_x: 0.0
    dock_pose_y: 0.0
    dock_pose_yaw: 0.0
EOF
    fi
  else
    info "mowgli_robot.yaml already exists"
  fi

  # mower_config.sh (GUI)
  if [ ! -f "$INSTALL_DIR/config/om/mower_config.sh" ]; then
    warn "No mower_config.sh found — GUI settings will use defaults"
    warn "Edit: $INSTALL_DIR/config/om/mower_config.sh"
    if [ ! -f "$INSTALL_DIR/config/om/mower_config.sh" ]; then
      cat > "$INSTALL_DIR/config/om/mower_config.sh" <<'EOF'
# Mowgli ROS2 v3 — GUI configuration
# Edit this file to match your mower hardware and location.

export OM_MOWER="YardForce500"
export OM_MOWER_ESC_TYPE="xesc_mini"
export OM_MOWER_GAMEPAD="xbox360"

# GPS — MUST be set for your location
export OM_USE_RELATIVE_POSITION=False
export OM_DATUM_LAT=0.0
export OM_DATUM_LONG=0.0
export OM_GPS_PROTOCOL=UBX
export OM_GPS_PORT=/dev/gps
export OM_GPS_BAUDRATE=921600

# NTRIP RTK
export OM_USE_NTRIP=False
export OM_NTRIP_HOSTNAME=""
export OM_NTRIP_PORT=2101
export OM_NTRIP_USER=""
export OM_NTRIP_PASSWORD=""
export OM_NTRIP_ENDPOINT=""

# Mowing
export OM_TOOL_WIDTH=0.13
export OM_ENABLE_MOWER=true
export OM_AUTOMATIC_MODE=0

# Battery (25.2V 7S Li-ion)
export OM_BATTERY_FULL_VOLTAGE=28.5
export OM_BATTERY_EMPTY_VOLTAGE=24.0
export OM_BATTERY_CRITICAL_VOLTAGE=23.0
EOF
    fi
  else
    info "mower_config.sh already exists"
  fi
}

# ── Step 6: Pull & start ───────────────────────────────────────────────────
pull_and_start() {
  step "Pull images & start"

  cd "$INSTALL_DIR"

  info "Pulling latest images..."
  docker compose pull

  info "Starting the stack..."
  docker compose up -d

  echo ""
  info "Mowgli ROS2 v3 is running!"
  echo ""
  echo "  Foxglove Studio:  ws://$(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):8765"
  echo "  Rosbridge:        ws://$(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):9090"
  echo "  MQTT:             $(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):1883"
  echo ""
  echo "  Logs:             docker compose logs -f mowgli"
  echo "  Stop:             docker compose down"
  echo "  Config:           $INSTALL_DIR/config/"
  echo ""

  # Check if devices exist
  local missing_devices=false
  for dev in /dev/mowgli /dev/gps /dev/ttyS1; do
    if [ ! -e "$dev" ]; then
      warn "Device $dev not found — plug in the hardware and check udev rules"
      missing_devices=true
    fi
  done
  if $missing_devices; then
    echo ""
    warn "Some devices are missing. The mowgli container may restart until they appear."
    warn "Check: ls -l /dev/mowgli /dev/gps /dev/ttyS1"
  fi
}

# ── Main ────────────────────────────────────────────────────────────────────

main() {
  echo ""
  echo -e "${GREEN}╔══════════════════════════════════════════╗${NC}"
  echo -e "${GREEN}║     Mowgli ROS2 v3 — Installer          ║${NC}"
  echo -e "${GREEN}╚══════════════════════════════════════════╝${NC}"
  echo ""

  install_docker
  install_udev_rules
  setup_directory
  setup_env
  setup_config
  pull_and_start
}

main "$@"
