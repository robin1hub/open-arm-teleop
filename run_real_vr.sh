#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

if [[ "${1:-}" != "--confirm-hardware" ]]; then
  echo "Refusing to start real-arm control without explicit confirmation." >&2
  echo "After testing MuJoCo and clearing the workspace, run:" >&2
  echo "  $0 --confirm-hardware" >&2
  exit 2
fi

for interface in can0 can1; do
  if ! ip link show "$interface" 2>/dev/null | grep -Eq '<[^>]*\bUP\b[^>]*>'; then
    echo "$interface is missing or not UP." >&2
    echo "Connect both CAN-FD adapters, then run: sudo openarm-can-cli can_configure" >&2
    exit 1
  fi
done

if [[ ! -x .venv/bin/dora ]]; then
  echo "Missing .venv. Build the environment first." >&2
  exit 1
fi

echo "WARNING: this dataflow can command both physical arms."
echo "Right arm: can0; left arm: can1. Keep the emergency stop accessible."
echo "Run './.venv/bin/python verify_real_mujoco_alignment.py' first."
echo "The MuJoCo viewer mirrors measured follower qpos; IK posture pull-to-home is disabled."
exec .venv/bin/dora run dataflow-vr-real-no-camera.yaml --uv
