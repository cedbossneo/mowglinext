#!/usr/bin/env bash

detect_rpi_model() {
  if [ -r /proc/device-tree/model ]; then
    tr -d '\0' < /proc/device-tree/model
    return 0
  fi
  echo "Unknown platform"
}

get_boot_config_file() {
  local config_file="/boot/firmware/config.txt"
  [ -f "$config_file" ] || config_file="/boot/config.txt"
  echo "$config_file"
}

append_config_line_if_missing() {
  local line="$1"
  local config_file
  config_file="$(get_boot_config_file)"

  require_root_for "boot config"

  if ! grep -q "^${line}$" "$config_file" 2>/dev/null; then
    echo "$line" | $SUDO tee -a "$config_file" > /dev/null
    info "Enabled ${line}"
  else
    info "${line} already enabled"
  fi
}

remove_config_line_if_present() {
  local line="$1"
  local config_file
  config_file="$(get_boot_config_file)"

  require_root_for "boot config"

  if grep -q "^${line}$" "$config_file" 2>/dev/null; then
    $SUDO sed -i "\|^${line}$|d" "$config_file"
    info "Removed ${line}"
  fi
}

disable_bluetooth_for_uart() {
  local config_file
  config_file="$(get_boot_config_file)"

  require_root_for "disable bluetooth"

  append_config_line_if_missing "dtoverlay=disable-bt"

  # Évite les conflits si une autre ancienne config traîne
  remove_config_line_if_present "dtoverlay=miniuart-bt"

  if command -v systemctl >/dev/null 2>&1; then
    $SUDO systemctl disable hciuart >/dev/null 2>&1 || true
    $SUDO systemctl stop hciuart >/dev/null 2>&1 || true
    info "Disabled hciuart service"
  fi
}

enable_all_platform_uarts() {
  step "UART platform setup"

  local model
  model="$(detect_rpi_model)"
  info "Detected platform: ${model}"

  append_config_line_if_missing "enable_uart=1"

  # Active tous les UART overlays utiles
  append_config_line_if_missing "dtoverlay=uart1"
  append_config_line_if_missing "dtoverlay=uart2"
  append_config_line_if_missing "dtoverlay=uart3"
  append_config_line_if_missing "dtoverlay=uart4"
  append_config_line_if_missing "dtoverlay=uart5"

  # Bluetooth toujours désactivé dans Mowgli II
  disable_bluetooth_for_uart

  info "All UART overlays requested and Bluetooth disabled"
}