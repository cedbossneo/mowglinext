#!/usr/bin/env bash

build_compose_stack() {
  COMPOSE_FILES=()

  COMPOSE_FILES+=("compose/docker-compose.base.yml")
  COMPOSE_FILES+=("compose/docker-compose.gui.yml")
  COMPOSE_FILES+=("compose/docker-compose.mqtt.yml")
  COMPOSE_FILES+=("compose/docker-compose.gps.yml")

  [[ "${ENABLE_FOXGLOVE:-true}" == "true" ]] && \
    COMPOSE_FILES+=("compose/docker-compose.foxglove.yml")

  if [[ "${LIDAR_ENABLED:-true}" == "true" && "${LIDAR_TYPE:-none}" != "none" ]]; then
    case "${LIDAR_TYPE:-}" in
      rplidar)
        COMPOSE_FILES+=("compose/docker-compose.lidar-rplidar.yml")
        ;;
      ldlidar)
        COMPOSE_FILES+=("compose/docker-compose.lidar-ldlidar.yml")
        ;;
      stl27l)
        COMPOSE_FILES+=("compose/docker-compose.lidar-stl27l.yml")
        ;;
      *)
        warn "Unknown LIDAR_TYPE: ${LIDAR_TYPE:-unset}"
        ;;
    esac
  fi

  [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("compose/docker-compose.tfluna-front.yml")

  [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("compose/docker-compose.tfluna-edge.yml")

  [[ "${ENABLE_MAVROS:-false}" == "true" ]] && \
    COMPOSE_FILES+=("compose/docker-compose.mavros.yml")

  [[ "${ENABLE_VESC:-false}" == "true" ]] && \
    COMPOSE_FILES+=("compose/docker-compose.vesc.yml")

  info "Compose stack:"
  for f in "${COMPOSE_FILES[@]}"; do
    echo "  - $f"
  done
}

run_compose_stack() {
  local args=()
  local f

  cd "$INSTALL_DIR" || return 1

  echo ""
  info "Selected compose files:"
  for f in "${COMPOSE_FILES[@]}"; do
    echo "  - $f"
    args+=(-f "$f")
  done
  echo ""

  info "Using env file: $INSTALL_DIR/.env"

  info "Pulling selected images..."
  docker compose --env-file "$INSTALL_DIR/.env" "${args[@]}" pull

  echo ""
  info "Starting selected stack..."
  docker compose --env-file "$INSTALL_DIR/.env" "${args[@]}" up -d

  echo ""
  info "Current containers:"
  docker compose --env-file "$INSTALL_DIR/.env" "${args[@]}" ps
}