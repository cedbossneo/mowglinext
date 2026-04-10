#!/usr/bin/env bash
select_hardware_backend() {
  echo ""
  echo "Select hardware backend:"
  echo "  [1] Mowgli STM32 board"
  echo "  [2] Pixhawk via MAVROS"
  echo ""
  printf "Choice [1-2]: "
  read -r choice

  case "$choice" in
    1)
      export HARDWARE_BACKEND="mowgli"
      export MAVROS_ENABLED="false"
      export MAVROS_BY_ID=""
      info "Selected backend: Mowgli STM32 board"
      ;;
    2)
      export HARDWARE_BACKEND="mavros"
      export MAVROS_ENABLED="true"
      info "Selected backend: Pixhawk via MAVROS"
      detect_mavros_by_id
      ;;
    *)
      error "Invalid choice"
      return 1
      ;;
  esac
}
detect_mavros_by_id() {
  local byid_dir="/dev/serial/by-id"
  local candidates=()

  if [ ! -d "$byid_dir" ]; then
    warn "Directory $byid_dir not found"
    return 1
  fi

  while IFS= read -r path; do
    candidates+=("$path")
  done < <(find "$byid_dir" -maxdepth 1 -type l \( -iname '*Pixhawk*' -o -iname '*Holybro*' \) | sort)

  if [ ${#candidates[@]} -eq 0 ]; then
    warn "No Pixhawk device found in /dev/serial/by-id"
    return 1
  fi

  info "Detected Pixhawk candidate device(s):"
  local i=1
  for c in "${candidates[@]}"; do
    echo "  [$i] $c -> $(readlink -f "$c")"
    i=$((i + 1))
  done

  local choice=""
  if [ ${#candidates[@]} -eq 1 ]; then
    choice="1"
  else
    printf "Select MAVROS port [1-%d]: " "${#candidates[@]}"
    read -r choice
  fi

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidates[@]}" ]; then
    error "Invalid MAVROS device selection"
    return 1
  fi

  export MAVROS_BY_ID="${candidates[$((choice - 1))]}"
  info "Selected MAVROS by-id: ${MAVROS_BY_ID}"
}