#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! pgrep -x vrserver >/dev/null; then
  echo "SteamVR vrserver is not running. Start SteamVR first." >&2
  exit 1
fi

exec .venv/bin/python vive_mujoco_teleop.py "$@"
