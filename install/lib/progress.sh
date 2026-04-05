#!/usr/bin/env bash

: "${INSTALL_DIR:=$PWD}"
LOG_DIR="${INSTALL_DIR}/logs/install"
LOG_FILE="${LOG_DIR}/install.log"

show_progress_bar() {
  local current="$1"
  local total="$2"
  local label="$3"

  local bar=""
  local i

  for ((i=1; i<=total; i++)); do
    if [ "$i" -le "$current" ]; then
      bar+="■"
    else
      bar+="□"
    fi
  done

  printf "[%s] %s" "$bar" "$label"
}

show_spinner() {
  local pid="$1"
  local delay=0.10
  local frames='|/-\'
  local i=0

  while kill -0 "$pid" 2>/dev/null; do
    printf "\r%s %c" "$SPINNER_PREFIX" "${frames:i++%${#frames}:1}"
    sleep "$delay"
  done
}

print_log_excerpt() {
  local logfile="$1"
  local lines="${2:-30}"

  [ -f "$logfile" ] || return 0

  echo ""
  echo "---- log excerpt ----"
  tail -n "$lines" "$logfile"
  echo "---------------------"
}

init_install_logs() {
  mkdir -p "$LOG_DIR"

  {
    echo "=================================================="
    echo "Mowgli install session"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "HOST: $(hostname 2>/dev/null || echo unknown)"
    echo "PWD : $(pwd)"
    echo "=================================================="
    echo ""
  } >> "$LOG_FILE"
}

progress_run() {
  local current="$1"
  local total="$2"
  local label="$3"
  local command="$4"

  mkdir -p "$LOG_DIR"

  local safe_label
  safe_label="$(echo "$label" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/_/g')"

  local logfile="${LOG_DIR}/$(printf '%02d' "$current")_${safe_label}.log"

  {
    echo "=================================================="
    echo "STEP ${current}/${total}: ${label}"
    echo "COMMAND: ${command}"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================="
  } >> "$LOG_FILE"

  : > "$logfile"
  {
    echo "=================================================="
    echo "STEP ${current}/${total}: ${label}"
    echo "COMMAND: ${command}"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================="
  } >> "$logfile"

  SPINNER_PREFIX="$(show_progress_bar "$current" "$total" "$label")"

  (
    set -e
    eval "$command"
  ) > >(tee -a "$logfile" >> "$LOG_FILE") \
    2> >(tee -a "$logfile" >> "$LOG_FILE" >&2) &
  local cmd_pid=$!

  show_spinner "$cmd_pid"
  wait "$cmd_pid"
  local rc=$?

  printf "\r\033[2K\n"

  if [ "$rc" -eq 0 ]; then
    printf "%s ... ${GREEN}OK${NC}\n" "$(show_progress_bar "$current" "$total" "$label")"

    if grep -Eiq 'warning|warn|deprecated' "$logfile"; then
      warn "Warnings detected during: $label"
      echo "     step log: $logfile"
      echo "     global log: $LOG_FILE"
    fi
  else
    printf "%s ... ${RED}FAIL${NC}\n" "$(show_progress_bar "$current" "$total" "$label")"
    error "Step failed: $label"
    echo "Step log: $logfile"
    echo "Global log: $LOG_FILE"
    print_log_excerpt "$logfile" 40
    exit "$rc"
  fi
}

progress_run_interactive() {
  local current="$1"
  local total="$2"
  local label="$3"
  shift 3

  mkdir -p "$LOG_DIR"

  {
    echo "=================================================="
    echo "STEP ${current}/${total}: ${label} (interactive)"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================="
  } >> "$LOG_FILE"

  printf "%s ...\n\n" "$(show_progress_bar "$current" "$total" "$label")"

  "$@"
  local rc=$?

  if [ "$rc" -eq 0 ]; then
    printf "${GREEN}OK${NC}\n"
  else
    printf "${RED}FAIL${NC}\n"
    echo "Global log: $LOG_FILE"
    exit "$rc"
  fi
}

progress_eval_interactive() {
  local current="$1"
  local total="$2"
  local label="$3"
  local command="$4"

  mkdir -p "$LOG_DIR"

  {
    echo "=================================================="
    echo "STEP ${current}/${total}: ${label} (interactive eval)"
    echo "COMMAND: ${command}"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================="
  } >> "$LOG_FILE"

  printf "%s ... \n\n" "$(show_progress_bar "$current" "$total" "$label")"

  eval "$command"
  local rc=$?

  if [ "$rc" -ne 0 ]; then
    error "Step failed: $label"
    echo "Global log: $LOG_FILE"
    exit "$rc"
  fi
}

progress_run_live() {
  local current="$1"
  local total="$2"
  local label="$3"
  shift 3

  mkdir -p "$LOG_DIR"

  {
    echo "=================================================="
    echo "STEP ${current}/${total}: ${label} (live)"
    echo "DATE: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================="
  } >> "$LOG_FILE"

  printf "%s ...\n\n" "$(show_progress_bar "$current" "$total" "$label")"

  "$@" 2>&1 | tee -a "$LOG_FILE"
  local rc=${PIPESTATUS[0]}

  echo ""

  if [ "$rc" -eq 0 ]; then
    printf "OK\n"
  else
    printf "${RED}FAIL${NC}\n"
    echo "Global log: $LOG_FILE"
    exit "$rc"
  fi
}