#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Build + push multi-arch images to GHCR
# ============================================================

# ---- Config ----
OWNER="mowglifrenchtouch"
REPO="mowglinext"
REGISTRY="ghcr.io/${OWNER}/${REPO}"

# Tag final poussé sur GHCR
TAG="mavros"

# Images à builder
IMAGES=(
  "ros2"
  "gui"
  "gps"
  "lidar-ldlidar"
  "lidar-rplidar"
  "lidar-stl27l"
  "mavros"
)

# Multi-arch
PLATFORMS="linux/amd64,linux/arm64"

# Buildx builder name
BUILDER_NAME="mowgli-builder"

# ---- Helpers ----
info() {
  echo -e "\n[INFO] $*"
}

error() {
  echo -e "\n[ERROR] $*" >&2
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    error "Command not found: $1"
    exit 1
  }
}

ensure_repo_root() {
  if [[ ! -d .git ]]; then
    error "Run this script from the repository root."
    exit 1
  fi
}

ensure_builder() {
  if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
    info "Creating buildx builder: ${BUILDER_NAME}"
    docker buildx create --name "${BUILDER_NAME}" --use
  else
    info "Using existing buildx builder: ${BUILDER_NAME}"
    docker buildx use "${BUILDER_NAME}"
  fi

  docker buildx inspect --bootstrap >/dev/null
}

ensure_fusioncore() {
  if [[ ! -f ros2/src/fusioncore/fusioncore_core/package.xml ]]; then
    info "Initializing FusionCore submodule"
    git submodule update --init ros2/src/fusioncore
  fi
}

build_and_push() {
  local name="$1"
  local context="$2"
  local dockerfile="$3"
  local image="$4"
  local target="${5:-}"

  info "Building ${name}"
  echo "       image: ${image}:${TAG}"
  echo "       context: ${context}"
  echo "       dockerfile: ${dockerfile}"
  [[ -n "${target}" ]] && echo "       target: ${target}"

  if [[ -n "${target}" ]]; then
    docker buildx build \
      --platform "${PLATFORMS}" \
      -f "${dockerfile}" \
      --target "${target}" \
      -t "${image}:${TAG}" \
      --push \
      "${context}"
  else
    docker buildx build \
      --platform "${PLATFORMS}" \
      -f "${dockerfile}" \
      -t "${image}:${TAG}" \
      --push \
      "${context}"
  fi
}

# ---- Main ----
require_cmd git
require_cmd docker
ensure_repo_root
ensure_builder
ensure_fusioncore

for img in "${IMAGES[@]}"; do
  case "${img}" in
    ros2)
      build_and_push \
        "ros2" \
        "./ros2" \
        "./ros2/Dockerfile" \
        "${REGISTRY}/mowgli-ros2" \
        "runtime"
      ;;
    gui)
      build_and_push \
        "gui" \
        "./gui" \
        "./gui/Dockerfile" \
        "${REGISTRY}/mowglinext-gui"
      ;;
    gps)
      build_and_push \
        "gps" \
        "./sensors/gps" \
        "./sensors/gps/Dockerfile" \
        "${REGISTRY}/gps"
      ;;
    lidar-ldlidar)
      build_and_push \
        "lidar-ldlidar" \
        "./sensors/lidar-ldlidar" \
        "./sensors/lidar-ldlidar/Dockerfile" \
        "${REGISTRY}/lidar-ldlidar" \
        "runtime"
      ;;
    lidar-rplidar)
      build_and_push \
        "lidar-rplidar" \
        "./sensors/lidar-rplidar" \
        "./sensors/lidar-rplidar/Dockerfile" \
        "${REGISTRY}/lidar-rplidar"
      ;;
    lidar-stl27l)
      build_and_push \
        "lidar-stl27l" \
        "./sensors/lidar-stl27l" \
        "./sensors/lidar-stl27l/Dockerfile" \
        "${REGISTRY}/lidar-stl27l" \
        "runtime"
      ;;
    mavros)
      build_and_push \
        "mavros" \
        "./sensors/mavros" \
        "./sensors/mavros/Dockerfile" \
        "${REGISTRY}/mavros"
      ;;
    *)
      error "Unknown image: ${img}"
      exit 1
      ;;
  esac
done

info "All images pushed successfully with tag: ${TAG}"