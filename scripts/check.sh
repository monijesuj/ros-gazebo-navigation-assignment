#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH=src/robot_demo python3 -m pytest -q src/robot_demo/test
python3 -m compileall -q src/robot_demo/robot_demo src/robot_demo/launch scripts
if command -v ruff >/dev/null; then
  ruff check src/robot_demo scripts
  ruff format --check src/robot_demo/robot_demo src/robot_demo/launch src/robot_demo/test scripts
fi
