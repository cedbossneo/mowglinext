#!/usr/bin/env bash

configure_lidar() {
  step "LiDAR configuration"

  # Reset generated rule
  LIDAR_UART_RULE=""

  # If preset values exist (from web composer), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${LIDAR_TYPE:-}" ]]; then
    : "${LIDAR_PORT:=/dev/lidar}"

    info "LiDAR pre-configured by web composer (skipping prompts)"
  else
    # Defaults based on PCB / GUI-ready
    : "${LIDAR_ENABLED:=true}"
    : "${LIDAR_TYPE:=ldlidar}"
    : "${LIDAR_MODEL:=LDLiDAR_LD19}"
    : "${LIDAR_CONNECTION:=uart}"
    : "${LIDAR_PORT:=/dev/lidar}"
    : "${LIDAR_UART_DEVICE:=/dev/ttyAMA5}"
    : "${LIDAR_BAUD:=230400}"

    echo ""
    echo "Type de LiDAR :"
    echo "  1) Aucun"
    echo "  2) RPLidar Slamtec (A1/A2/A3)"
    echo "  3) LDLiDAR (LD06 / LD14 / LD19)"
    echo "  4) STL27L"
    prompt "Choix" "3"
    local lidar_choice="$REPLY"

    case "$lidar_choice" in
      1)
        LIDAR_ENABLED="false"
        LIDAR_TYPE="none"
        LIDAR_MODEL=""
        LIDAR_CONNECTION=""
        LIDAR_UART_DEVICE=""
        LIDAR_UART_RULE=""
        ;;
      2)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="rplidar"
        LIDAR_MODEL="RPLIDAR_A1"
        LIDAR_BAUD="115200"
        ;;
      3)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="ldlidar"
        LIDAR_MODEL="LDLiDAR_LD19"
        LIDAR_BAUD="230400"
        ;;
      4)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="stl27l"
        LIDAR_MODEL="STL27L"
        LIDAR_BAUD="230400"
        ;;
      *)
        error "Choix LiDAR invalide"
        return 1
        ;;
    esac

    if [ "$LIDAR_ENABLED" = "true" ]; then
      echo ""
      echo "Connexion LiDAR :"
      echo "  1) USB"
      echo "  2) UART (PCB default: ttyAMA5)"
      prompt "Choix" "2"
      local conn_choice="$REPLY"

      case "$conn_choice" in
        1)
          LIDAR_CONNECTION="usb"
          LIDAR_UART_DEVICE=""
          LIDAR_UART_RULE=""
          ;;
        2)
          LIDAR_CONNECTION="uart"
          LIDAR_UART_DEVICE="/dev/ttyAMA5"
          LIDAR_UART_RULE="KERNEL==\"ttyAMA5\", SYMLINK+=\"lidar\", MODE=\"0666\""
          ;;
        *)
          error "Choix connexion LiDAR invalide"
          return 1
          ;;
      esac
    fi
  fi

  # Generate udev rule if UART connection
  if [[ "${LIDAR_CONNECTION:-}" == "uart" && -n "${LIDAR_UART_DEVICE:-}" ]]; then
    LIDAR_UART_RULE="KERNEL==\"ttyAMA5\", SYMLINK+=\"lidar\", MODE=\"0666\""
  fi

  echo ""
  info "LiDAR : enabled=${LIDAR_ENABLED:-false} type=${LIDAR_TYPE:-none} model=${LIDAR_MODEL:-none} connection=${LIDAR_CONNECTION:-none} port=$LIDAR_PORT uart=${LIDAR_UART_DEVICE:-none} baud=${LIDAR_BAUD:-0}"
}

run_lidar_configuration_step() {
  configure_lidar
}