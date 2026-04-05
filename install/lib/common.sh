#colours
#info, warn, error, step
#prompt, confirm
#require_root_for

#!/usr/bin/env bash

# ── Colours & helpers ───────────────────────────────────────────────────────

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

info()  { echo -e "  ${GREEN}OK${NC}  $*"; }
warn()  { echo -e "  ${YELLOW}!!${NC}  $*"; }
fail()  { echo -e "  ${RED}FAIL${NC}  $*"; }
error() { echo -e "${RED}[x]${NC} $*" >&2; }
step()  { echo -e "\n${CYAN}${BOLD}── $* ──${NC}"; }
ask()   { echo -en "${BOLD}$1${NC} "; }

# Prompt with default value. Sets REPLY global.
prompt() {
  local answer
  echo -en "${BOLD}$1 [${2:-}]:${NC} " >/dev/tty
  read -r answer </dev/tty
  echo >/dev/tty
  REPLY="${answer:-$2}"
}

# Yes/no prompt. Usage: if confirm "Continue?"; then ...
confirm() {
  local answer
  echo -en "${BOLD}$1 [Y/n]:${NC} " >/dev/tty
  read -r answer </dev/tty
  echo >/dev/tty
  [[ "${answer,,}" != "n" ]]
}

command_exists() {
  command -v "$1" &>/dev/null
}

require_root_for() {
  if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
  else
    SUDO=""
  fi
}

require_root() {
  if [[ $EUID -ne 0 ]]; then
    error "This action requires root privileges. Please run as root or with sudo."
    exit 1
  fi
}