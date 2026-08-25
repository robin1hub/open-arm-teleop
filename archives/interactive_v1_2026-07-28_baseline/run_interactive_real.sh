#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

if [[ "${1:-}" != "--confirm-hardware" ]]; then
  echo "Refusing to connect physical arms without explicit confirmation." >&2
  echo "Clear the workspace, keep the emergency stop accessible, then run:" >&2
  echo "  $0 --confirm-hardware" >&2
  exit 2
fi

for interface in can0 can1; do
  if ! ip link show "$interface" 2>/dev/null | grep -Eq '<[^>]*\bUP\b[^>]*>'; then
    echo "$interface is missing or not UP; physical mode was not started." >&2
    exit 1
  fi
done

echo "Physical interactive mode:"
echo "  right arm = can0; left arm = can1"
echo "  initial state = measured physical posture"
echo "  command rate = 20 Hz; maximum joint step = 0.003 rad"
echo "  E = enable/disable selected arm; press D = move to simulation pose"
echo "  switching arms disables the currently enabled arm"
echo "No arm will be enabled automatically."

exec .venv/bin/python interactive_mujoco_ee_drag.py \
  --model-version v1 \
  --real \
  --confirm-hardware \
  --config "$project_dir/openarm_safe_raw_zero.yaml" \
  --command-hz 20 \
  --max-joint-step 0.003
