#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! pgrep -x vrserver >/dev/null; then
  echo "SteamVR vrserver is not running." >&2
  exit 1
fi

exec .venv/bin/python vive_absolute_mujoco_teleop.py "$@"
