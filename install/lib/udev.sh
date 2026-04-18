#!/usr/bin/env bash
build_static_udev_rules() {
  cat <<'EOF'
# =========================================================
# Mowgli II - static udev rules
# =========================================================

# Mowgli STM32 board — match by product string to avoid confusion
# with other STM32 CDC devices (0483:5740 is generic STM32 VCP).
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# Known GPS USB devices
# NOTE: 0483:5740 (STM32 VCP) removed — it conflicts with the Mowgli board
# which uses the same vendor/product ID. If your GPS uses an STM32-based
# USB adapter, add a rule matching its specific product string instead.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01aa", SYMLINK+="gps", MODE="0666"
EOF
}

build_dynamic_udev_rules() {
  echo "# ========================================================="
  echo "# Mowgli II - dynamic rules from current hardware selection"
  echo "# ========================================================="

  # MAVROS selected by /dev/serial/by-id
  if [ "${HARDWARE_BACKEND:-mowgli}" = "mavros" ] && [ -n "${MAVROS_BY_ID:-}" ] && [ -e "${MAVROS_BY_ID}" ]; then
    local mavros_kernel
    mavros_kernel="$(basename "$(readlink -f "$MAVROS_BY_ID")")"
    echo "KERNEL==\"${mavros_kernel}\", SYMLINK+=\"mavros\", MODE=\"0666\""
  fi

  # GPS principal
  if [ "${GPS_CONNECTION:-usb}" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$GPS_UART_DEVICE")\", SYMLINK+=\"gps\", MODE=\"0666\""
  fi

  # GPS debug (miniUART only)
  if [ "${GPS_DEBUG_ENABLED:-false}" = "true" ] && [ -n "${GPS_DEBUG_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$GPS_DEBUG_UART_DEVICE")\", SYMLINK+=\"gps_debug\", MODE=\"0666\""
  fi

  # LiDAR
  if [ "${LIDAR_ENABLED:-true}" = "true" ] && [ "${LIDAR_CONNECTION:-uart}" = "uart" ] && [ -n "${LIDAR_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$LIDAR_UART_DEVICE")\", SYMLINK+=\"lidar\", MODE=\"0666\""
  fi

  # TF-Luna front
  if [ "${TFLUNA_FRONT_ENABLED:-false}" = "true" ] && [ -n "${TFLUNA_FRONT_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$TFLUNA_FRONT_UART_DEVICE")\", SYMLINK+=\"tfluna_front\", MODE=\"0666\""
  fi

  # TF-Luna edge
  if [ "${TFLUNA_EDGE_ENABLED:-false}" = "true" ] && [ -n "${TFLUNA_EDGE_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$TFLUNA_EDGE_UART_DEVICE")\", SYMLINK+=\"tfluna_edge\", MODE=\"0666\""
  fi
}

install_udev_rules() {
  step "Installing udev rules"
  require_root_for "udev rules"

  local rules_file="$UDEV_RULES_FILE"
  local tmpfile
  tmpfile="$(mktemp)"

  {
    build_static_udev_rules
    echo ""
    build_dynamic_udev_rules
  } > "$tmpfile"

  local changed=false

  if [ ! -f "$rules_file" ]; then
    changed=true
  elif ! cmp -s "$tmpfile" "$rules_file"; then
    changed=true
  fi

  if $changed; then
    $SUDO cp "$tmpfile" "$rules_file"
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    info "udev rules installed/updated"
  else
    info "udev rules already up to date"
  fi

  rm -f "$tmpfile"

  # Verify symlinks were created — UART devices may not exist until reboot
  local needs_reboot=false

  if [ "${GPS_CONNECTION:-usb}" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    if [ ! -e "$GPS_UART_DEVICE" ]; then
      warn "GPS UART device $GPS_UART_DEVICE does not exist yet (UART overlay needs reboot)"
      needs_reboot=true
    elif [ ! -e "${GPS_PORT:-/dev/gps}" ]; then
      warn "GPS symlink ${GPS_PORT:-/dev/gps} not created — check udev rules"
    else
      info "GPS symlink: ${GPS_PORT:-/dev/gps} -> $(readlink -f "${GPS_PORT:-/dev/gps}")"
    fi
  fi

  if [ "${LIDAR_ENABLED:-true}" = "true" ] && [ "${LIDAR_CONNECTION:-}" = "uart" ] && [ -n "${LIDAR_UART_DEVICE:-}" ]; then
    if [ ! -e "$LIDAR_UART_DEVICE" ]; then
      warn "LiDAR UART device $LIDAR_UART_DEVICE does not exist yet (UART overlay needs reboot)"
      needs_reboot=true
    elif [ ! -e "${LIDAR_PORT:-/dev/lidar}" ]; then
      warn "LiDAR symlink ${LIDAR_PORT:-/dev/lidar} not created — check udev rules"
    else
      info "LiDAR symlink: ${LIDAR_PORT:-/dev/lidar} -> $(readlink -f "${LIDAR_PORT:-/dev/lidar}")"
    fi
  fi

  if $needs_reboot; then
    warn "Some UART devices require a reboot to become available"
    add_issue "Reboot required for UART devices. Run: sudo reboot"
  fi
}
