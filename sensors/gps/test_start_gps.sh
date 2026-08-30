#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

touch "$TMP_DIR/ros_setup.bash" "$TMP_DIR/gnss_setup.bash"

run_dry_run() {
  local config="$1"
  GNSS_CONFIG_PATH="$config" \
    ROS_SETUP_BASH="$TMP_DIR/ros_setup.bash" \
    GNSS_SIDECAR_SETUP_BASH="$TMP_DIR/gnss_setup.bash" \
    GNSS_DRY_RUN=true \
    GNSS_NTRIP_ENABLED=false \
    bash "$SCRIPT_DIR/start_gps.sh"
}

assert_contains() {
  local output="$1"
  local expected="$2"
  if [[ "$output" != *"$expected"* ]]; then
    printf 'FAILED: expected output to contain %q\n%s\n' "$expected" "$output" >&2
    exit 1
  fi
}

assert_not_contains() {
  local output="$1"
  local unexpected="$2"
  if [[ "$output" == *"$unexpected"* ]]; then
    printf 'FAILED: expected output not to contain %q\n%s\n' "$unexpected" "$output" >&2
    exit 1
  fi
}

cat >"$TMP_DIR/configured.yaml" <<'YAML'
mowgli:
  ros__parameters:
    gnss_receiver_family: unicore
    gnss_transport: serial
    gnss_serial_device: /dev/null
    gnss_rover_dynamic_mode: survey_mow
    gnss_unicore_rtk_timeout_s: 45
    gnss_unicore_dgps_timeout_s: 90
    gnss_ntrip_enabled: false
YAML

configured_output="$(run_dry_run "$TMP_DIR/configured.yaml")"
assert_contains "$configured_output" "[start_gps.sh] config_apply"
assert_contains "$configured_output" "--rover-dynamic-mode survey_mow"
assert_contains "$configured_output" "--rtk-timeout-s 45"
assert_contains "$configured_output" "--dgps-timeout-s 90"

apply_line="$(grep -nF '[start_gps.sh] config_apply' <<<"$configured_output" | cut -d: -f1)"
receiver_line="$(grep -nF '[start_gps.sh] receiver_node' <<<"$configured_output" | cut -d: -f1)"
if ((apply_line >= receiver_line)); then
  printf 'FAILED: config apply must remain ordered before receiver_node\n%s\n' "$configured_output" >&2
  exit 1
fi

cat >"$TMP_DIR/unset.yaml" <<'YAML'
mowgli:
  ros__parameters:
    gnss_receiver_family: unicore
    gnss_transport: serial
    gnss_serial_device: /dev/null
    gnss_ntrip_enabled: false
YAML

unset_output="$(run_dry_run "$TMP_DIR/unset.yaml")"
assert_not_contains "$unset_output" "--rover-dynamic-mode"
assert_not_contains "$unset_output" "--rtk-timeout-s"
assert_not_contains "$unset_output" "--dgps-timeout-s"
assert_not_contains "$unset_output" " 120"
assert_not_contains "$unset_output" " 300"

cat >"$TMP_DIR/unrelated.yaml" <<'YAML'
mowgli:
  ros__parameters:
    gnss_receiver_family: ublox
    gnss_transport: serial
    gnss_serial_device: /dev/null
    gnss_unicore_rtk_timeout_s: 45
    gnss_unicore_dgps_timeout_s: 90
    gnss_ntrip_enabled: false
YAML

unrelated_output="$(run_dry_run "$TMP_DIR/unrelated.yaml")"
assert_not_contains "$unrelated_output" "--rtk-timeout-s"
assert_not_contains "$unrelated_output" "--dgps-timeout-s"

printf 'PASS: start_gps.sh rover-policy dry-run tests\n'
