#!/usr/bin/env bash

install_docker() {
  step "1/6  Docker"

  if command_exists docker; then
    info "Docker $(docker --version 2>/dev/null | grep -oP '[\d.]+' | head -1)"
  else
    info "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    info "Docker installed"
  fi

  if docker compose version &>/dev/null; then
    info "Docker Compose $(docker compose version --short 2>/dev/null)"
  else
    error "Docker Compose v2 not found. Install docker-compose-plugin."
    exit 1
  fi

  if ! groups "$USER" | grep -qw docker 2>/dev/null; then
    require_root_for "docker group"
    $SUDO usermod -aG docker "$USER"
    warn "Added $USER to docker group — log out/in for it to take effect"
  fi
}

# Run a command with the docker group active.
#
# On a fresh install, `usermod -aG docker` (above) won't take effect in the
# current shell — the new group membership is only picked up at next login.
# Running `docker ...` directly then fails with "permission denied while
# trying to connect to the docker API". We work around this by trampolining
# through `sg docker -c …` when the socket is unreachable but the user is
# already in the group. Re-runs (where the user has logged in again) take
# the fast path and call the command directly. See issue #27.
docker_run() {
  if docker info >/dev/null 2>&1; then
    "$@"
  elif groups "$USER" 2>/dev/null | grep -qw docker; then
    sg docker -c "$(printf '%q ' "$@")"
  else
    "$@"
  fi
}