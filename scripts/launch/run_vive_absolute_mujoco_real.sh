#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_dir"

if ! pgrep -x vrserver >/dev/null; then
  echo "SteamVR vrserver is not running." >&2
  exit 1
fi

exec sudo --preserve-env=DISPLAY,XAUTHORITY,DBUS_SESSION_BUS_ADDRESS,XDG_RUNTIME_DIR \
  env PYTHONPATH="$project_dir/src${PYTHONPATH:+:$PYTHONPATH}" \
  .venv/bin/python -m openarm_teleop.vive_absolute_mujoco_real_teleop "$@"
