#!/usr/bin/env bash
# Create/update a dedicated virtualenv for sim_py and install dependencies.
# Usage:
#   ./scripts/setup_sim_py_venv.sh
#   ./scripts/setup_sim_py_venv.sh --with-rotorpy

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV_DIR="$ROOT_DIR/sim_py/.venv"
WITH_ROTORPY="false"

if [[ "${1:-}" == "--with-rotorpy" ]]; then
  WITH_ROTORPY="true"
fi

python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r "$ROOT_DIR/sim_py/requirements.txt"

if [[ "$WITH_ROTORPY" == "true" ]]; then
  "$VENV_DIR/bin/python" -m pip install -r "$ROOT_DIR/sim_py/requirements-rotorpy.txt"
fi

echo "sim_py venv ready: $VENV_DIR"
echo "Activate with:"
echo "  source sim_py/.venv/bin/activate"
