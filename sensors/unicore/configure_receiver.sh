#!/bin/bash
# =============================================================================
# Send a one-time UM98x rover configuration over the serial port.
#
# Symptoms this fixes:
#   * accuracy stuck at 10 m, RTK never validates, even with RTCM flowing
#   * /diagnostics carr_soln stays "none"
#
# Root cause: out-of-the-box / FRESET-state UM98x receivers default to
# something close to "rover, no NMEA enabled, no PVTSLNA" — and the
# default RTK timeout is short. Without an explicit MODE ROVER + LOG
# directives, the receiver outputs only minimal NMEA and the operator
# sees a single-point fix even though RTCM is being injected.
#
# We send commands using the port-agnostic short form
# (`LOG <msg> ONTIME <rate>`) so the receiver outputs on whichever
# physical UART the host is connected to.
#
# Reference: https://github.com/CentipedeRTK/docs-centipedeRTK
#            (assets/param_files/UM98x/UM980_aog_rover_last_CONFIG.txt)
#
# Usage: configure_receiver.sh [port] [baudrate]
#
# Persisted via SAVECONFIG in receiver NVRAM, so this only meaningfully
# changes anything on the first run; subsequent calls are idempotent.
# =============================================================================
set -euo pipefail

PORT="${1:-/dev/gps}"
BAUD="${2:-460800}"

if [ ! -c "$PORT" ]; then
  echo "[configure_receiver.sh] ERROR: $PORT is not a character device" >&2
  exit 1
fi

# Wait briefly for the device to settle (avoids races with udev rules).
for _ in $(seq 1 20); do
  [ -w "$PORT" ] && break
  sleep 0.1
done

stty -F "$PORT" "$BAUD" raw -echo -echoe -echok -ixon -ixoff || {
  echo "[configure_receiver.sh] ERROR: stty failed on $PORT @ $BAUD" >&2
  exit 1
}

CMDS=(
  # Rover mode + NMEA version + RTK timeouts.
  # SURVEY MOW = low-dynamics survey-grade rover with the mowing-specific
  # dynamics preset. UAV (which we tried first) is for drones — assumes
  # high vertical accelerations and hurts ambiguity resolution on a
  # ground robot that mostly translates. AUTOMOTIVE is the next-best
  # alternative if SURVEY MOW isn't supported on a given firmware rev.
  "MODE ROVER SURVEY MOW"
  "CONFIG NMEAVERSION V411"
  # 180 s tolerates short NTRIP outages without dropping back to single
  # point. Default is 60 s on most firmware revisions.
  "CONFIG RTK TIMEOUT 180"
  "CONFIG RTK RELIABILITY 4 3"
  "CONFIG DGPS TIMEOUT 300"
  # SIGNALGROUP 2 = GPS L1/L2 + GLO L1/L2 + GAL E1/E5b + BDS B1I/B2I —
  # the de-facto RTK rover signal set on UM98x.
  "CONFIG SIGNALGROUP 2"
  "CONFIG AGNSS ENABLE"

  # Constellation enables. Mowgli mowers are typically in mid-latitude
  # open sky, so we want everything except QZSS (regional, doesn't help
  # in EU/NA).
  "UNMASK GPS"
  "UNMASK GLO"
  "UNMASK GAL"
  "UNMASK BDS"
  "MASK QZSS"

  # Output messages — port-agnostic LOG form, rate is period in seconds.
  # Unicore log-name convention: base name + 'A' suffix = ASCII format.
  # PVTSLN + A = PVTSLNA (the message header is `#PVTSLNA,...` — note
  # SINGLE trailing A — and our parser keys on that string).
  # BESTNAV + A = BESTNAVA (message header `#BESTNAVA,...` — double A
  # because the base name itself ends in V, not A).
  "LOG GPGGA ONTIME 1"
  "LOG PVTSLNA ONTIME 0.1"
  "LOG BESTNAVA ONTIME 0.1"
  "LOG GNHPR ONTIME 0.1"
  "LOG GPGSV ONTIME 1"
  "LOG GLGSV ONTIME 1"
  "LOG GAGSV ONTIME 1"
  "LOG GBGSV ONTIME 1"

  # Persist
  "SAVECONFIG"
)

echo "[configure_receiver.sh] Sending UM98x rover config to ${PORT} @ ${BAUD}..."
exec 3>"$PORT"
trap 'exec 3>&-' EXIT
for cmd in "${CMDS[@]}"; do
  printf '%s\r\n' "$cmd" >&3
  echo "  -> $cmd"
  sleep 0.3
done

echo "[configure_receiver.sh] Done. Receiver config persisted via SAVECONFIG."
