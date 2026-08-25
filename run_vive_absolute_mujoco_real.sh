#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! pgrep -x vrserver >/dev/null; then
  echo "SteamVR vrserver is not running." >&2
  exit 1
fi

exec sudo --preserve-env=DISPLAY,XAUTHORITY,DBUS_SESSION_BUS_ADDRESS,XDG_RUNTIME_DIR \
  .venv/bin/python vive_absolute_mujoco_real_teleop.py "$@"
