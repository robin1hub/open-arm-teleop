#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

python_bin="${PYTHON_BIN:-python3.12}"
if ! command -v "$python_bin" >/dev/null 2>&1; then
  echo "Missing $python_bin. Install Python 3.12 and python3.12-venv first." >&2
  exit 1
fi

for command_name in git ip pgrep; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Missing system command: $command_name" >&2
    exit 1
  fi
done

if [[ ! -d .venv ]]; then
  "$python_bin" -m venv .venv
fi

.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -r requirements-teleop-lock.txt
.venv/bin/python -m pip install --no-deps -e .

# Install the local, packaged nodes used by the current scripts. These are
# editable so later fixes in this directory are immediately active.
for node in \
  nodes/dora-openarm \
  nodes/dora-openarm-kinematics \
  nodes/dora-openarm-mujoco \
  nodes/dora-openarm-vr \
  nodes/dora-openarm-quitter; do
  .venv/bin/python -m pip install -e "$node"
done

PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}" \
  .venv/bin/python -m unittest -q tests/test_vive_real_soft_limits.py
.venv/bin/python -m compileall -q src/openarm_teleop scripts

echo
echo "Portable environment created successfully."
echo "Read AGENT_HANDOFF_DEPLOYMENT.md before connecting or enabling hardware."
