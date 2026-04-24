#!/usr/bin/env bash
# Ensure config files mounted as bind-mount *files* exist before compose runs.
# Docker creates a directory when the host path is missing, which breaks the
# container with "not a directory" errors.

: "${REPO_DIR:?REPO_DIR is not set}"
: "${INSTALL_DIR:?INSTALL_DIR is not set}"

DOCKER_DIR="${DOCKER_DIR:-$REPO_DIR/docker}"
COMPOSE_SRC_DIR="${COMPOSE_SRC_DIR:-$INSTALL_DIR/compose}"
FINAL_COMPOSE_FILE="${FINAL_COMPOSE_FILE:-$DOCKER_DIR/docker-compose.yaml}"
FINAL_ENV_FILE="${FINAL_ENV_FILE:-$DOCKER_DIR/.env}"

ensure_default_configs() {
  local defaults="$REPO_DIR/docker/config"

  if [ ! -d "$defaults" ]; then
    warn "Defaults config directory missing: $defaults"
    return 1
  fi

  mkdir -p "$DOCKER_DIR/config/mqtt"
  mkdir -p "$DOCKER_DIR/config/mowgli"
  mkdir -p "$DOCKER_DIR/config/om"
  mkdir -p "$DOCKER_DIR/config/db"

  if [ ! -f "$DOCKER_DIR/config/mqtt/mosquitto.conf" ]; then
    cp "$defaults/mqtt/mosquitto.conf" "$DOCKER_DIR/config/mqtt/mosquitto.conf"
    info "Created default mosquitto.conf"
  fi

  if [ ! -f "$DOCKER_DIR/config/cyclonedds.xml" ]; then
    cp "$defaults/cyclonedds.xml" "$DOCKER_DIR/config/cyclonedds.xml"
    info "Created default cyclonedds.xml"
  fi
}


build_compose_stack() {
  COMPOSE_FILES=()

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.base.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gui.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mqtt.yml")

  # In Mowgli mode, select one direct GNSS stack.
  # In MAVROS mode, GPS is handled via Pixhawk/MAVROS + NTRIP sidecar,
  # so direct GNSS compose fragments must not be included.
  if [[ "${HARDWARE_BACKEND:-mowgli}" != "mavros" ]]; then
    case "${GNSS_BACKEND:-gps}" in
      gps)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps.yml")
        ;;
      ublox)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.ublox.yaml")
        ;;
      unicore)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.unicore.yaml")
        ;;
      *)
        error "Unknown GNSS_BACKEND: ${GNSS_BACKEND:-unset} (expected: gps, ublox, unicore)"
        return 1
        ;;
    esac
  fi

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.watchtower.yml")

  # Foxglove bridge is controlled via the ENABLE_FOXGLOVE env var passed
  # to the ROS2 container (see docker-compose.base.yml).  No separate
  # compose file is needed — the launch file starts/skips the node.
  if [[ "${LIDAR_ENABLED:-true}" == "true" && "${LIDAR_TYPE:-none}" != "none" ]]; then
    case "${LIDAR_TYPE:-}" in
      rplidar)
        # RPLiDAR A-series: supported, but A1 only is the tested combo today.
        # A2 / A3 / S-series images are built but untested — please report back.
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-rplidar.yml")
        ;;
      ldlidar)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-ldlidar.yml")
        ;;
      stl27l)
        warn "LIDAR_TYPE=stl27l is EXPERIMENTAL — driver is a stub, motion tests not run yet."
        COMPOSE_FILES+=("compose/docker-compose.lidar-stl27l.yml")
        ;;
      *)
        warn "Unknown LIDAR_TYPE: ${LIDAR_TYPE:-unset}"
        ;;
    esac
  fi

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    warn "TF-Luna (front) integration is EXPERIMENTAL and not wired into collision_monitor yet."
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-front.yml")
  fi

  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    warn "TF-Luna (edge) integration is EXPERIMENTAL and not wired into collision_monitor yet."
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-edge.yml")
  fi

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    warn "MAVROS bridge is COMING SOON — enabling this starts the service but nothing in Mowgli consumes it."
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mavros.yml")
  fi

  if [[ "${ENABLE_VESC:-false}" == "true" ]]; then
    warn "VESC motor-controller support is COMING SOON — not integrated with the STM32-based hardware_bridge flow yet."
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.vesc.yml")
  fi

  info "Compose fragments sélectionnés :"
  for f in "${COMPOSE_FILES[@]}"; do
    echo "  - $f"
  done
}

# Extract service definitions from a compose template file.
# Outputs everything between the "services:" line and the next top-level key
# (or EOF), preserving indentation. Skips x-ros2-env anchors since the merged
# file defines its own single anchor.

write_compose_merged() {
# Generate a single self-contained docker-compose.yaml by merging all
# selected compose templates. Users get one readable file instead of
# needing to understand Docker Compose include/project mechanics.
  mkdir -p "$DOCKER_DIR"

  local compose_args=()
  local f

  for f in "${COMPOSE_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
      echo "Fragment manquant : $f" >&2
      return 1
    fi
    compose_args+=("-f" "$f")
  done

  (
    cd "$REPO_DIR" || exit 1
    docker compose \
      --project-directory "$REPO_DIR" \
      --env-file "$FINAL_ENV_FILE" \
      "${compose_args[@]}" \
      config > "$FINAL_COMPOSE_FILE"
  )

  info "Fichier généré : $FINAL_COMPOSE_FILE"
}

run_compose_stack() {
  ensure_default_configs
  build_compose_stack
  write_compose_merged

  info "Compose final : $FINAL_COMPOSE_FILE"
  info "Env file : $FINAL_ENV_FILE"

  info "Pulling selected images..."
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" pull
  echo ""
  info "Starting stack..."
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" up -d
  echo ""
  info "Current containers:"
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" ps
}
