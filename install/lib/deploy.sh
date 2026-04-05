#!/usr/bin/env bash
setup_directory() {
  step "Preparing repository"

  if [ -d "$REPO_DIR/.git" ]; then
    info "Updating existing git repository in $REPO_DIR"
    git -C "$REPO_DIR" fetch origin
    git -C "$REPO_DIR" reset --hard "origin/$REPO_BRANCH"
    return
  fi

  if [ -d "$REPO_DIR" ]; then
    warn "Directory $REPO_DIR already exists but is not a git repository"

    if confirm "Do you want to backup and replace it?"; then
      local backup_dir="${REPO_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
      mv "$REPO_DIR" "$backup_dir"
      info "Backup created at $backup_dir"
    else
      error "Cannot continue without a clean repository directory"
      return 1
    fi
  fi

  info "Cloning repository into $REPO_DIR"
  git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" "$REPO_DIR"

  if [ ! -d "$INSTALL_DIR" ]; then
    error "Docker directory not found after clone: $INSTALL_DIR"
    return 1
  fi
}

run_startup_step_live() {
  build_compose_stack
  run_compose_stack

  if ! $SKIP_WRITE_CONFIG; then
    auto_detect_position
  fi
}