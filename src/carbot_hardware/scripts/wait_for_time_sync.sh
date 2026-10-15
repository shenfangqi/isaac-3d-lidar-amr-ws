#!/usr/bin/env bash
# Wait until this boot's clock has been synchronized by NTP, for at most
# CARBOT_MICROROS_TIME_SYNC_WAIT_SECONDS (default 90; 0 skips the wait).
#
# The Jetson has no RTC backup: systemd-timesyncd restores the previous
# shutdown time at boot and NTP steps the clock forward about 30 s later.  An
# ESP32 session created in between keeps timestamps hours off, because the
# firmware resync is slew-limited (issue #95).  On timeout this warns and
# returns 0 so the chassis still starts without network access.

set -euo pipefail

flag="${CARBOT_TIME_SYNC_FLAG:-/run/systemd/timesync/synchronized}"
timeout_seconds="${CARBOT_MICROROS_TIME_SYNC_WAIT_SECONDS:-90}"

case "${timeout_seconds}" in
  ''|*[!0-9]*)
    echo "Invalid CARBOT_MICROROS_TIME_SYNC_WAIT_SECONDS: ${timeout_seconds}" >&2
    exit 64
    ;;
esac

if [[ "${timeout_seconds}" == "0" ]]; then
  echo "Skipping the NTP wait (CARBOT_MICROROS_TIME_SYNC_WAIT_SECONDS=0)"
  exit 0
fi

# timesyncd creates the flag (on tmpfs, so once per boot) at its first
# successful sync; timedatectl also covers a chrony-based clock.
synchronized() {
  [[ -e "${flag}" ]] && return 0
  [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)" == "yes" ]]
}

# Count with the monotonic boot clock: bash SECONDS follows the wall clock,
# which NTP steps forward exactly while we wait (2026-10-15: a 27 s wait was
# logged as 86 s; after a long power-off it would read hours).
uptime_seconds() {
  local up _
  read -r up _ < /proc/uptime
  echo "${up%.*}"
}

start_seconds=$(uptime_seconds)
last_notice_seconds=-10
until synchronized; do
  waited=$(($(uptime_seconds) - start_seconds))
  if ((waited >= timeout_seconds)); then
    echo "WARNING: NTP not synchronized after ${timeout_seconds} s; ESP32" \
      "timestamps will follow the restored clock until the agent restarts" \
      "(issue #95)" >&2
    exit 0
  fi
  if ((waited - last_notice_seconds >= 10)); then
    echo "Waiting for NTP synchronization before the ESP32 session" \
      "(${waited}/${timeout_seconds} s)" >&2
    last_notice_seconds=${waited}
  fi
  sleep 1
done
echo "Clock synchronized after $(($(uptime_seconds) - start_seconds)) s"
