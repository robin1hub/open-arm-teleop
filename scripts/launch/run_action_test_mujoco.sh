#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_dir"

if [[ ! -x .venv/bin/dora ]]; then
  echo "Missing .venv. Build the OpenArm environment first." >&2
  exit 1
fi

echo "Simulation only: CAN and the physical OpenArm driver are not part of this dataflow."
echo "Expected motion: both arms move from home to a mirrored target in 3 seconds, then return."
exec .venv/bin/dora run dataflows/dataflow-action-test-mujoco.yaml --uv
