#!/usr/bin/env bash

setup_directory() {
  step "Preparing repository"

  if [ -d "$REPO_DIR/.git" ]; then
    info "Updating existing git repository in $REPO_DIR"

    if ! git -C "$REPO_DIR" fetch origin "$REPO_BRANCH"; then
      error "Failed to fetch branch '$REPO_BRANCH' from origin"
      return 1
    fi

    if ! git -C "$REPO_DIR" rev-parse --verify FETCH_HEAD >/dev/null 2>&1; then
      error "Fetched branch '$REPO_BRANCH' is not available"
      return 1
    fi

    if ! git -C "$REPO_DIR" checkout -B "$REPO_BRANCH" FETCH_HEAD; then
      error "Failed to check out branch '$REPO_BRANCH'"
      return 1
    fi

    if ! git -C "$REPO_DIR" reset --hard FETCH_HEAD; then
      error "Failed to reset repository to fetched branch '$REPO_BRANCH'"
      return 1
    fi

    if [ ! -d "$INSTALL_DIR" ]; then
      error "Install directory not found in existing repository: $INSTALL_DIR"
      return 1
    fi

    return 0
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
  if ! git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" "$REPO_DIR"; then
    error "Failed to clone branch '$REPO_BRANCH' from $REPO_URL"
    return 1
  fi

  if [ ! -d "$INSTALL_DIR" ]; then
    error "Install directory not found after clone: $INSTALL_DIR"
    return 1
  fi

  return 0
}

run_startup_step_live() {
  build_compose_stack
  run_compose_stack

  if ! $SKIP_WRITE_CONFIG; then
    auto_detect_position
  fi
}