#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Build + push multi-arch images to GHCR
# ============================================================

# ---- Config ----
OWNER="${OWNER:-mowglifrenchtouch}"
REPO="${REPO:-mowglinext}"
REGISTRY="ghcr.io/${OWNER}/${REPO}"

# Tag principal
TAG="${TAG:-mavros}"

# Tags additionnels optionnels
# Exemple:
# 
EXTRA_TAGS="nightly nightly-$(date +%Y%m%d)"
EXTRA_TAGS="${EXTRA_TAGS:-}"

# Build metadata
BUILD_DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
GIT_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"

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
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"

# Buildx builder name
BUILDER_NAME="${BUILDER_NAME:-mowgli-builder}"

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

make_tag_args() {
  local image="$1"
  local args=("-t" "${image}:${TAG}")

  if [[ -n "${EXTRA_TAGS}" ]]; then
    for extra in ${EXTRA_TAGS}; do
      args+=("-t" "${image}:${extra}")
    done
  fi

  printf '%s\n' "${args[@]}"
}

build_and_push() {
  local name="$1"
  local context="$2"
  local dockerfile="$3"
  local image="$4"
  local target="${5:-}"

  info "Building ${name}"
  echo "       image: ${image}:${TAG}"
  [[ -n "${EXTRA_TAGS}" ]] && echo "       extra tags: ${EXTRA_TAGS}"
  echo "       context: ${context}"
  echo "       dockerfile: ${dockerfile}"
  [[ -n "${target}" ]] && echo "       target: ${target}"
  echo "       git branch: ${GIT_BRANCH}"
  echo "       git sha: ${GIT_SHA}"

  mapfile -t tag_args < <(make_tag_args "${image}")

  if [[ -n "${target}" ]]; then
    docker buildx build \
      --platform "${PLATFORMS}" \
      -f "${dockerfile}" \
      --target "${target}" \
      "${tag_args[@]}" \
      --label org.opencontainers.image.created="${BUILD_DATE}" \
      --label org.opencontainers.image.revision="${GIT_SHA}" \
      --label org.opencontainers.image.source="https://github.com/${OWNER}/${REPO}" \
      --push \
      "${context}"
  else
    docker buildx build \
      --platform "${PLATFORMS}" \
      -f "${dockerfile}" \
      "${tag_args[@]}" \
      --label org.opencontainers.image.created="${BUILD_DATE}" \
      --label org.opencontainers.image.revision="${GIT_SHA}" \
      --label org.opencontainers.image.source="https://github.com/${OWNER}/${REPO}" \
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

info "All images pushed successfully"
info "Primary tag: ${TAG}"
[[ -n "${EXTRA_TAGS}" ]] && info "Extra tags: ${EXTRA_TAGS}"