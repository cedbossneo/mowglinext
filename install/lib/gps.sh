#!/usr/bin/env bash

configure_gps() {
  step "GPS configuration"

  # Reset generated rules
  GPS_UART_RULE=""
  GPS_DEBUG_UART_RULE=""

  # If preset values exist (from web composer or CLI), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${GPS_CONNECTION:-}" && -n "${GPS_PROTOCOL:-}" ]]; then
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_BAUD:=115200}"

    info "GPS pre-configured (skipping prompts)"

    # For UART connections, always let user confirm/change the port
    if [[ "${GPS_CONNECTION}" == "uart" ]]; then
      pick_uart_port "${GPS_UART_DEVICE:-/dev/ttyAMA4}"
      GPS_UART_DEVICE="$REPLY"
    fi
    if [[ "${GPS_DEBUG_ENABLED}" == "true" ]]; then
      pick_uart_port "${GPS_DEBUG_UART_DEVICE:-/dev/ttyS0}"
      GPS_DEBUG_UART_DEVICE="$REPLY"
    fi
  else
    # Defaults: NMEA primary on this fork (UM980 etc.); UBX still selectable
    # but pulls the ublox image from upstream cedbossneo/mowglinext.
    : "${GPS_PROTOCOL:=NMEA}"
    : "${GPS_CONNECTION:=uart}"
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
    : "${GPS_BAUD:=115200}"

    # Debug only on miniUART
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
    : "${GPS_DEBUG_BAUD:=115200}"

    echo ""
    echo "$MSG_GPS_CONNECTION"
    echo "  1) USB"
    echo "  2) UART"
    prompt "$MSG_CHOICE" "2"
    local conn_choice="$REPLY"

    case "$conn_choice" in
      1)
        GPS_CONNECTION="usb"
        GPS_UART_DEVICE=""
        ;;
      2)
        GPS_CONNECTION="uart"
        pick_uart_port "/dev/ttyAMA4"
        GPS_UART_DEVICE="$REPLY"
        ;;
      *)
        error "$MSG_GPS_INVALID_CONNECTION"
        return 1
        ;;
    esac

    echo ""
    echo "$MSG_GPS_PROTOCOL"
    echo "  1) NMEA   (UM980, ArduSimple, Septentrio, generic NMEA RTK) [default]"
    echo "  2) UBX    (u-blox ZED-F9P — image pulled from upstream)"
    prompt "$MSG_CHOICE" "1"
    local proto_choice="$REPLY"

    case "$proto_choice" in
      1)
        GPS_PROTOCOL="NMEA"
        GPS_BAUD="115200"
        ;;
      2)
        GPS_PROTOCOL="UBX"
        GPS_BAUD="460800"
        warn "UBX selected — pulling ublox image from upstream (cedbossneo/mowglinext)."
        warn "  This fork's CI does not build the ublox image. Override GPS_IMAGE in"
        warn "  .env if you want a different source."
        ;;
      *)
        error "$MSG_GPS_INVALID_PROTOCOL"
        return 1
        ;;
    esac

    echo ""
    if confirm "$MSG_GPS_DEBUG_CONFIRM"; then
      GPS_DEBUG_ENABLED="true"
      pick_uart_port "/dev/ttyS0"
      GPS_DEBUG_UART_DEVICE="$REPLY"
    else
      GPS_DEBUG_ENABLED="false"
      GPS_DEBUG_UART_DEVICE=""
    fi
  fi

  # Main GPS rule only if UART is selected
  if [ "$GPS_CONNECTION" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    local gps_kernel
    gps_kernel="$(basename "$GPS_UART_DEVICE")"
    GPS_UART_RULE="KERNEL==\"${gps_kernel}\", SYMLINK+=\"gps\", MODE=\"0666\""
  fi

  # Debug GPS rule only if enabled
  if [ "${GPS_DEBUG_ENABLED:-false}" = "true" ] && [ -n "${GPS_DEBUG_UART_DEVICE:-}" ]; then
    local gps_debug_kernel
    gps_debug_kernel="$(basename "$GPS_DEBUG_UART_DEVICE")"
    GPS_DEBUG_UART_RULE="KERNEL==\"${gps_debug_kernel}\", SYMLINK+=\"gps_debug\", MODE=\"0666\""
  fi

  echo ""
  info "$MSG_GPS_MAIN : connection=$GPS_CONNECTION protocol=$GPS_PROTOCOL port=$GPS_PORT uart=${GPS_UART_DEVICE:-none} baud=$GPS_BAUD"
  info "GPS debug     : enabled=$GPS_DEBUG_ENABLED port=$GPS_DEBUG_PORT uart=${GPS_DEBUG_UART_DEVICE:-none} baud=$GPS_DEBUG_BAUD"
}

run_gps_configuration_step() {
  configure_gps
}
