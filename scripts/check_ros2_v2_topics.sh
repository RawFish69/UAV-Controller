#!/usr/bin/env bash
set -euo pipefail

echo "[check] Required nodes"
ros2 node list

echo "[check] Required topics"
ros2 topic list | grep -E '^/uav1/(command|mission|telemetry|mission_status|backend/)'

echo "[check] Telemetry sample"
ros2 topic echo /uav1/telemetry --once

echo "[check] Mission status sample (may wait if no mission active)"
timeout 3s ros2 topic echo /uav1/mission_status --once || true

echo "[check] Backend odom sample"
ros2 topic echo /uav1/backend/odom --once
