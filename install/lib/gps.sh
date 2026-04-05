#!/usr/bin/env bash

configure_gps() {
  step "GPS configuration"

  # Reset generated rules
  GPS_UART_RULE=""
  GPS_DEBUG_UART_RULE=""

  # If preset values exist (from web composer), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${GPS_CONNECTION:-}" && -n "${GPS_PROTOCOL:-}" ]]; then
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
    : "${GPS_DEBUG_BAUD:=115200}"

    info "GPS pre-configured by web composer (skipping prompts)"
  else
    # Defaults based on PCB / GUI-ready
    : "${GPS_PROTOCOL:=UBX}"
    : "${GPS_CONNECTION:=uart}"
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
    : "${GPS_BAUD:=460800}"

    # Debug only on miniUART
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
    : "${GPS_DEBUG_BAUD:=115200}"

    echo ""
    echo "Connexion GPS :"
    echo "  1) USB"
    echo "  2) UART (PCB default: ttyAMA4)"
    prompt "Choix" "2"
    local conn_choice="$REPLY"

    case "$conn_choice" in
      1)
        GPS_CONNECTION="usb"
        GPS_UART_DEVICE=""
        ;;
      2)
        GPS_CONNECTION="uart"
        GPS_UART_DEVICE="/dev/ttyAMA4"
        ;;
      *)
        error "Choix connexion GPS invalide"
        return 1
        ;;
    esac

    echo ""
    echo "Protocole GPS :"
    echo "  1) UBX"
    echo "  2) NMEA"
    prompt "Choix" "1"
    local proto_choice="$REPLY"

    case "$proto_choice" in
      1)
        GPS_PROTOCOL="UBX"
        GPS_BAUD="460800"
        ;;
      2)
        GPS_PROTOCOL="NMEA"
        GPS_BAUD="115200"
        ;;
      *)
        error "Choix protocole invalide"
        return 1
        ;;
    esac

    echo ""
    if confirm "Activer le port GPS debug (miniUART / gps_debug) ?"; then
      GPS_DEBUG_ENABLED="true"
      GPS_DEBUG_UART_DEVICE="/dev/ttyS0"
    else
      GPS_DEBUG_ENABLED="false"
      GPS_DEBUG_UART_DEVICE=""
    fi
  fi

  # Main GPS rule only if UART is selected
  if [ "$GPS_CONNECTION" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    GPS_UART_RULE="KERNEL==\"ttyAMA4\", SYMLINK+=\"gps\", MODE=\"0666\""
  fi

  # Debug GPS rule only if enabled, always on miniUART
  if [ "${GPS_DEBUG_ENABLED:-false}" = "true" ]; then
    GPS_DEBUG_UART_RULE="KERNEL==\"ttyS0\", SYMLINK+=\"gps_debug\", MODE=\"0666\""
  fi

  echo ""
  info "GPS principal : connection=$GPS_CONNECTION protocol=$GPS_PROTOCOL port=$GPS_PORT uart=${GPS_UART_DEVICE:-none} baud=$GPS_BAUD"
  info "GPS debug     : enabled=$GPS_DEBUG_ENABLED port=$GPS_DEBUG_PORT uart=${GPS_DEBUG_UART_DEVICE:-none} baud=$GPS_DEBUG_BAUD"
}

run_gps_configuration_step() {
  configure_gps
}