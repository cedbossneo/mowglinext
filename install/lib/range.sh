#!/usr/bin/env bash

configure_rangefinders() {
  step "Rangefinders configuration"

  # Reset generated rules
  TFLUNA_FRONT_UART_RULE=""
  TFLUNA_EDGE_UART_RULE=""

  # If preset values exist (from web composer), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${TFLUNA_FRONT_ENABLED:-}" ]]; then
    : "${TFLUNA_FRONT_PORT:=/dev/tfluna_front}"
    : "${TFLUNA_FRONT_UART_DEVICE:=/dev/ttyAMA3}"
    : "${TFLUNA_FRONT_BAUD:=115200}"
    : "${TFLUNA_EDGE_ENABLED:=false}"
    : "${TFLUNA_EDGE_PORT:=/dev/tfluna_edge}"
    : "${TFLUNA_EDGE_UART_DEVICE:=/dev/ttyAMA2}"
    : "${TFLUNA_EDGE_BAUD:=115200}"

    info "Rangefinders pre-configured by web composer (skipping prompts)"
  else
    # Defaults based on PCB / GUI-ready
    : "${TFLUNA_FRONT_ENABLED:=false}"
    : "${TFLUNA_FRONT_PORT:=/dev/tfluna_front}"
    : "${TFLUNA_FRONT_UART_DEVICE:=/dev/ttyAMA3}"
    : "${TFLUNA_FRONT_BAUD:=115200}"

    : "${TFLUNA_EDGE_ENABLED:=false}"
    : "${TFLUNA_EDGE_PORT:=/dev/tfluna_edge}"
    : "${TFLUNA_EDGE_UART_DEVICE:=/dev/ttyAMA2}"
    : "${TFLUNA_EDGE_BAUD:=115200}"

    echo ""
    echo "Configuration des capteurs TF-Luna :"
    echo "  1) Aucun"
    echo "  2) Front uniquement"
    echo "  3) Edge uniquement"
    echo "  4) Front + edge"
    prompt "Choix" "1"
    local range_choice="$REPLY"

    case "$range_choice" in
      1)
        TFLUNA_FRONT_ENABLED="false"
        TFLUNA_EDGE_ENABLED="false"
        ;;
      2)
        TFLUNA_FRONT_ENABLED="true"
        TFLUNA_EDGE_ENABLED="false"
        ;;
      3)
        TFLUNA_FRONT_ENABLED="false"
        TFLUNA_EDGE_ENABLED="true"
        ;;
      4)
        TFLUNA_FRONT_ENABLED="true"
        TFLUNA_EDGE_ENABLED="true"
        ;;
      *)
        error "Choix TF-Luna invalide"
        return 1
        ;;
    esac
  fi

  # Generate udev rules based on enabled sensors
  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    TFLUNA_FRONT_UART_RULE='KERNEL=="ttyAMA3", SYMLINK+="tfluna_front", MODE="0666"'
  fi
  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    TFLUNA_EDGE_UART_RULE='KERNEL=="ttyAMA2", SYMLINK+="tfluna_edge", MODE="0666"'
  fi

  echo ""
  info "TF-Luna front : enabled=$TFLUNA_FRONT_ENABLED port=$TFLUNA_FRONT_PORT uart=${TFLUNA_FRONT_UART_DEVICE:-none} baud=$TFLUNA_FRONT_BAUD"
  info "TF-Luna edge  : enabled=$TFLUNA_EDGE_ENABLED port=$TFLUNA_EDGE_PORT uart=${TFLUNA_EDGE_UART_DEVICE:-none} baud=$TFLUNA_EDGE_BAUD"
}

run_range_configuration_step() {
  configure_rangefinders
}