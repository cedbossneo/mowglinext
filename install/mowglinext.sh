#!/usr/bin/env bash
# =============================================================================
# Mowgli ROS2 v3 — Interactive Install / Upgrade / Diagnose Script
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_LIB_DIR="${SCRIPT_DIR}/lib"

source "${INSTALL_LIB_DIR}/common.sh"
source "${INSTALL_LIB_DIR}/i18n.sh"
source "${INSTALL_LIB_DIR}/config.sh"
source "${INSTALL_LIB_DIR}/banner.sh"
source "${INSTALL_LIB_DIR}/progress.sh"
source "${INSTALL_LIB_DIR}/motd.sh"
source "${INSTALL_LIB_DIR}/system.sh"
source "${INSTALL_LIB_DIR}/docker.sh"
source "${INSTALL_LIB_DIR}/udev.sh"
source "${INSTALL_LIB_DIR}/deploy.sh"
source "${INSTALL_LIB_DIR}/env.sh"
source "${INSTALL_LIB_DIR}/gps.sh"
source "${INSTALL_LIB_DIR}/lidar.sh"
source "${INSTALL_LIB_DIR}/range.sh"
source "${INSTALL_LIB_DIR}/uart.sh"
source "${INSTALL_LIB_DIR}/rc_local.sh"
source "${INSTALL_LIB_DIR}/checks.sh"
source "${INSTALL_LIB_DIR}/compose.sh"
source "${INSTALL_LIB_DIR}/tools.sh"

load_preset() {
  local preset_file="${SCRIPT_DIR}/.preset"
  if [ -f "$preset_file" ]; then
    info "Loading hardware preset from web composer"
    # Source the preset file to set environment variables
    # shellcheck disable=SC1090
    source "$preset_file"
    PRESET_LOADED=true
  else
    PRESET_LOADED=false
  fi
}

main() {
  show_banner
  load_locale
  init_install_logs

  if ! $CHECK_ONLY; then
    local TOTAL_STEPS=13

    # Language selection, load previous env, then load preset
    select_language

    # Load existing .env for defaults on re-run (preset/CLI flags override)
    if [ -f "$INSTALL_DIR/.env" ]; then
      set -a
      # shellcheck disable=SC1091
      source "$INSTALL_DIR/.env"
      set +a
      info "Loaded previous configuration from .env"
    fi

    load_preset

    progress_run_interactive 1 "$TOTAL_STEPS" "Updating system" \
      run_system_update

    progress_run 2 "$TOTAL_STEPS" "Installing Docker" \
      'install_docker'

    progress_run 3 "$TOTAL_STEPS" "Enabling UARTs" \
      'enable_all_platform_uarts && generate_rc_local'

    progress_run_interactive 4 "$TOTAL_STEPS" "Configuring GPS" \
      run_gps_configuration_step

    progress_run_interactive 5 "$TOTAL_STEPS" "Configuring LiDAR" \
      run_lidar_configuration_step

    progress_run_interactive 6 "$TOTAL_STEPS" "Configuring rangefinders" \
      run_range_configuration_step

    progress_run 7 "$TOTAL_STEPS" "Installing udev rules" \
      'install_udev_rules'

    progress_run_interactive 8 "$TOTAL_STEPS" "Preparing repository" \
      setup_directory

    progress_run 9 "$TOTAL_STEPS" "Writing environment" \
      'setup_env'

    progress_run_interactive 10 "$TOTAL_STEPS" "Configuring mower" \
      run_mower_configuration_step

    progress_run_interactive 11 "$TOTAL_STEPS" "Installing optional tools" \
      install_optional_tools

    progress_run 12 "$TOTAL_STEPS" "Installing MOTD" \
      'install_motd'

    progress_run_live 13 "$TOTAL_STEPS" "Starting containers" \
      run_startup_step_live

  else
    if [ ! -f "$INSTALL_DIR/compose/docker-compose.base.yml" ]; then
      error "No installation found at $INSTALL_DIR — run without --check first"
      return 1
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
  check_rangefinders
  check_slam
  check_gui

  print_summary
}

parse_args "$@"
main