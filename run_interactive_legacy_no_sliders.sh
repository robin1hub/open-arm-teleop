#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

if [[ "${1:-}" == "--real" ]]; then
  exec .venv/bin/python \
    archives/interactive_mujoco_ee_drag_no_sliders_2026-07-28.py \
    --model-version v1 \
    --real \
    --confirm-hardware \
    --config "$project_dir/openarm_safe_raw_zero.yaml" \
    --command-hz 40 \
    --max-joint-step 0.0087
fi

exec .venv/bin/python \
  archives/interactive_mujoco_ee_drag_no_sliders_2026-07-28.py \
  --model-version v1
