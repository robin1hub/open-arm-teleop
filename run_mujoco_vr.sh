#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

if [[ ! -x .venv/bin/dora ]]; then
  echo "Missing .venv. Build the environment first." >&2
  exit 1
fi

host_ip="$(hostname -I | awk '{print $1}')"
echo "Quest 3 target: ${host_ip:-<PC-IP>}:5006"
echo "Data collection UI: http://127.0.0.1:8000"
exec .venv/bin/dora run dataflow-vr-mujoco.yaml --uv

