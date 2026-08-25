#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_dir"

if [[ "${1:-}" != "--confirm-hardware" ]]; then
  echo "Physical motion is disabled unless explicitly confirmed." >&2
  echo "Usage: $0 --confirm-hardware [options]" >&2
  exit 2
fi

if ! pgrep -x vrserver >/dev/null; then
  echo "SteamVR vrserver is not running." >&2
  exit 1
fi

for interface in can0 can1; do
  if ! ip link show "$interface" 2>/dev/null | grep -q 'UP'; then
    echo "$interface is missing or not UP; refusing physical mode." >&2
    exit 1
  fi
done

echo "OpenArm 1.0 shared absolute REAL mode"
echo "  right arm = can0; left arm = can1"
echo "  default output = DISABLED; calibrate, then press E explicitly"
echo "  keep the physical emergency stop accessible"

exec env PYTHONPATH="$project_dir/src${PYTHONPATH:+:$PYTHONPATH}" \
  .venv/bin/python -m openarm_teleop.vive_absolute_mujoco_real_teleop \
  --config config/openarm_safe_raw_zero.yaml \
  --mapping-mode shared \
  --position-step-mm 9 \
  --orientation-step-deg 3.5 \
  --command-hz 40 \
  --max-joint-step 0.0125 \
  --max-joint-acceleration 3.0 \
  --max-gripper-step 0.001 \
  --joint-limit-margin 0.04 \
  "$@"
