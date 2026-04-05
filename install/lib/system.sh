#!/usr/bin/env bash

get_os_codename() {
  . /etc/os-release
  echo "${VERSION_CODENAME:-unknown}"
}

get_os_pretty_name() {
  . /etc/os-release
  echo "${PRETTY_NAME:-unknown}"
}

check_apt_sources_warning() {
  local current_codename
  current_codename="$(get_os_codename)"

  if grep -RhiqE 'trixie|testing|stable' /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null; then
    warn "Potentially unstable APT sources detected on ${current_codename}"
    warn "Check your APT sources before upgrading."
  fi
}

run_system_update() {
  step "System update"

  require_root_for "apt update"

  local current_pretty
  current_pretty="$(get_os_pretty_name)"
  local current_codename
  current_codename="$(get_os_codename)"

  info "Detected system: ${current_pretty}"

  check_apt_sources_warning

  info "Running apt update..."
  $SUDO apt update

  if confirm "Do you want to pin APT to the current release (${current_codename})?"; then
    $SUDO mkdir -p /etc/apt/apt.conf.d
    echo "APT::Default-Release \"${current_codename}\";" | $SUDO tee /etc/apt/apt.conf.d/99defaultrelease > /dev/null
    info "APT pinned to ${current_codename}"
  else
    info "No APT release pin applied"
  fi

  if confirm "Do you want to run apt upgrade -y now?"; then
    info "Running apt upgrade..."
    $SUDO apt upgrade -y
    info "System upgraded"
  else
    info "APT upgrade skipped"
  fi
}

get_upgradable_count() {
  if ! command -v apt >/dev/null 2>&1; then
    echo "n/a"
    return
  fi

  apt list --upgradable 2>/dev/null | awk 'NR>1' | wc -l | tr -d ' '
}

is_release_pinned() {
  if [ -f /etc/apt/apt.conf.d/99defaultrelease ]; then
    echo "yes"
  else
    echo "no"
  fi
}