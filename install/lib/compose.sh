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
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.watchtower.yml")
  if [[ "${LIDAR_ENABLED:-true}" == "true" && "${LIDAR_TYPE:-none}" != "none" ]]; then
    case "${LIDAR_TYPE:-}" in
      rplidar)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-rplidar.yml")
        ;;
      ldlidar)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-ldlidar.yml")
        ;;
      stl27l)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-stl27l.yml")
        ;;
      *)
        warn "Unknown LIDAR_TYPE: ${LIDAR_TYPE:-unset}"
        ;;
    esac
  fi

  [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-front.yml")

  [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-edge.yml")

  [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mavros.yml")

  [[ "${ENABLE_VESC:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.vesc.yml")

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

  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" pull
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" up -d
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" ps
}