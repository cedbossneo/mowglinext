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