#!/usr/bin/env bash

find_serial_by_id() {
  local by_id_dir="${SERIAL_BY_ID_DIR:-/dev/serial/by-id}"
  local pattern
  local match

  [ -d "$by_id_dir" ] || return 1

  for pattern in "$@"; do
    match="$(find "$by_id_dir" -maxdepth 1 -type l -iname "$pattern" | sort | head -n1)"
    if [ -n "$match" ]; then
      printf '%s\n' "$match"
      return 0
    fi
  done

  return 1
}

emit_by_id_udev_rule() {
  local by_id_path="$1"
  local symlink="$2"
  local kernel

  [ -L "$by_id_path" ] || return 1
  kernel="$(basename "$(readlink -f "$by_id_path")")"
  [ -n "$kernel" ] || return 1

  echo "# $symlink from $by_id_path"
  echo "KERNEL==\"${kernel}\", SYMLINK+=\"${symlink}\", MODE=\"0666\""
}

build_static_udev_rules() {
  cat <<'EOF'
# =========================================================
# Mowgli II - static udev rules
# =========================================================

# Mowgli STM32 board — match by product string to avoid confusion
# with other STM32 CDC devices (0483:5740 is generic STM32 VCP).
#SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# Known GPS USB devices
# NOTE: 0483:5740 (STM32 VCP) removed — it conflicts with the Mowgli board
# which uses the same vendor/product ID. If your GPS uses an STM32-based
# USB adapter, add a rule matching its specific product string instead.
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01aa", SYMLINK+="gps", MODE="0666"
EOF
}

build_dynamic_udev_rules() {
  local by_id_path

  echo "# ========================================================="
  echo "# Mowgli II - dynamic rules from current hardware selection"
  echo "# ========================================================="

  if by_id_path="$(find_serial_by_id "*Mowgli*")"; then
    emit_by_id_udev_rule "$by_id_path" "mowgli"
  fi

  # MAVROS uses the explicitly selected device. Prefer by-id when selected,
  # but keep the ttyACM/ttyUSB fallback for compatibility with manual choices.
  if [ "${HARDWARE_BACKEND:-mowgli}" = "mavros" ] && [ -n "${MAVROS_BY_ID:-}" ] && [ -e "${MAVROS_BY_ID}" ]; then
    if [ -L "$MAVROS_BY_ID" ]; then
      emit_by_id_udev_rule "$MAVROS_BY_ID" "mavros"
    else
      echo "KERNEL==\"$(basename "$MAVROS_BY_ID")\", SYMLINK+=\"mavros\", MODE=\"0666\""
    fi
  fi

  # GPS principal
  if [ "${GPS_CONNECTION:-usb}" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$GPS_UART_DEVICE")\", SYMLINK+=\"gps\", MODE=\"0666\""
  elif [ -n "${GPS_BY_ID:-}" ] && [ -e "${GPS_BY_ID}" ]; then
    emit_by_id_udev_rule "$GPS_BY_ID" "gps"
  elif [ "${HARDWARE_BACKEND:-mowgli}" != "mavros" ]; then
    case "${GNSS_BACKEND:-gps}" in
      ublox)
        by_id_path="$(find_serial_by_id "*u-blox*" "*ublox*" || true)"
        ;;
      unicore)
        by_id_path=""
        ;;
      gps)
        by_id_path="$(find_serial_by_id "*u-blox*" "*ublox*" || true)"
        ;;
      *)
        by_id_path=""
        ;;
    esac

    if [ -n "$by_id_path" ]; then
      emit_by_id_udev_rule "$by_id_path" "gps"
    fi
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
  elif [ -n "${GPS_BY_ID:-}" ]; then
    if [ ! -e "${GPS_BY_ID}" ]; then
      warn "GPS by-id device ${GPS_BY_ID} does not exist"
    elif [ ! -e "${GPS_PORT:-/dev/gps}" ]; then
      warn "GPS symlink ${GPS_PORT:-/dev/gps} not created from GPS_BY_ID — check udev rules"
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
