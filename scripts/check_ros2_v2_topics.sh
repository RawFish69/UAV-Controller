#!/usr/bin/env bash
set -euo pipefail

echo "[check] Required nodes"
ros2 node list

echo "[check] Required topics"
ros2 topic list | grep -E '^/uav/(command|mission|telemetry|mission_status|backend/)'

echo "[check] Telemetry sample"
ros2 topic echo /uav/telemetry --once

echo "[check] Mission status sample (may wait if no mission active)"
timeout 3s ros2 topic echo /uav/mission_status --once || true

echo "[check] Backend odom sample"
ros2 topic echo /uav/backend/odom --once
