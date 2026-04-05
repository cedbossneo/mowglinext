#!/usr/bin/env bash

banner_tools() {
  echo ""
  echo -e "${CYAN}${BOLD}── Optional tools ──${NC}"
  echo ""
}

create_dockermgr_wrapper() {
  local target_cmd="$1"
  require_root_for "dockermgr wrapper"

  cat <<EOF | $SUDO tee /usr/local/bin/dockermgr > /dev/null
#!/usr/bin/env bash
exec ${target_cmd} "\$@"
EOF
  $SUDO chmod +x /usr/local/bin/dockermgr
  info "Command available: dockermgr -> ${target_cmd}"
}

install_docker_cli_tool() {
  banner_tools
  echo "=== Étape outils : gestionnaire Docker en ligne de commande ==="
  echo "1) Oui, installer lazydocker (recommandé)"
  echo "2) Oui, installer ctop (alternatif)"
  echo "3) Non"
  prompt "Ton choix" "3"
  local docker_cli="$REPLY"

  case "$docker_cli" in
    1)
      info "Installing lazydocker..."
      curl -fsSL https://raw.githubusercontent.com/jesseduffield/lazydocker/master/scripts/install_update_linux.sh | bash
      create_dockermgr_wrapper "lazydocker"
      ;;
    2)
      require_root_for "ctop"
      info "Installing ctop..."
      $SUDO apt install -y ctop
      create_dockermgr_wrapper "ctop"
      ;;
    3)
      info "No Docker CLI manager installed"
      ;;
    *)
      warn "Invalid choice. No Docker CLI manager installed."
      ;;
  esac
}

install_file_manager_tool() {
  banner_tools
  echo "=== Étape outils : gestionnaire de fichiers en ligne de commande ==="
  echo "1) Oui, installer Midnight Commander (mc)"
  echo "2) Oui, installer ranger"
  echo "3) Non"
  prompt "Ton choix" "3"
  local fileman_choice="$REPLY"

  require_root_for "file manager"

  case "$fileman_choice" in
    1)
      info "Installing Midnight Commander..."
      $SUDO apt install -y mc
      info "Command available: mc"
      ;;
    2)
      info "Installing ranger..."
      $SUDO apt install -y ranger
      info "Command available: ranger"
      ;;
    3)
      info "No file manager installed"
      ;;
    *)
      warn "Invalid choice. No file manager installed."
      ;;
  esac
}

install_debug_tools() {
  banner_tools
  echo "=== Étape outils : outils de développement et debug ==="
  echo "1) Tous les outils (recommandé)"
  echo "2) Outils essentiels seulement"
  echo "3) Aucun outil"
  prompt "Ton choix" "2"
  local debug_tools="$REPLY"

  require_root_for "debug tools"

  case "$debug_tools" in
    1)
      info "Installing full debug/tooling set..."
      $SUDO apt install -y \
        htop ncdu lsof strace gdb minicom screen picocom \
        tmux bat fd-find ripgrep jq yq \
        nmap iptraf-ng usbutils i2c-tools
      ;;
    2)
      info "Installing essential debug/tooling set..."
      $SUDO apt install -y \
        htop ncdu git tmux minicom screen jq usbutils
      ;;
    3)
      info "No debug/dev tools installed"
      ;;
    *)
      warn "Invalid choice. No debug/dev tools installed."
      ;;
  esac
}

install_optional_tools() {
  step "Optional tools"

  install_docker_cli_tool
  install_file_manager_tool
  install_debug_tools
  install_mowgli_helpers
}

create_helper_script() {
  local path="$1"
  local content="$2"

  require_root_for "helper script"

  printf '%s\n' "$content" | $SUDO tee "$path" > /dev/null
  $SUDO chmod +x "$path"
}

install_mowgli_helpers() {
  banner_tools
  echo "=== Étape outils : helpers Mowgli ==="

  if ! confirm "Installer les commandes helper Mowgli ?"; then
    info "No Mowgli helpers installed"
    return
  fi

  local project_dir="${INSTALL_DIR}"

  create_helper_script "/usr/local/bin/mowgli-up" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose up -d \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-down" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose down \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-restart" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose restart \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose logs -f \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-ps" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose ps"

  create_helper_script "/usr/local/bin/mowgli-check" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec bash ./mowglinext.sh --check"

  create_helper_script "/usr/local/bin/mowgli-gps-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose logs -f gps"

  create_helper_script "/usr/local/bin/mowgli-lidar-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec docker compose logs -f lidar"

  create_helper_script "/usr/local/bin/mowgli-shell" "#!/usr/bin/env bash
container=\"\${1:-mowgli-ros2}\"
exec docker exec -it \"\$container\" bash"

  create_helper_script "/usr/local/bin/mowgli-status" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
docker compose ps
echo
docker stats --no-stream 2>/dev/null || true"

  info "Installed Mowgli helper commands"
  echo ""
  echo "Available commands:"
  echo "  mowgli-up"
  echo "  mowgli-down"
  echo "  mowgli-restart"
  echo "  mowgli-logs"
  echo "  mowgli-ps"
  echo "  mowgli-check"
  echo "  mowgli-gps-logs"
  echo "  mowgli-lidar-logs"
  echo "  mowgli-shell [container]"
  echo "  mowgli-status"
}
