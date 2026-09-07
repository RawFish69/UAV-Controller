#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if ! command -v pio >/dev/null 2>&1; then
  echo "PlatformIO CLI not found. Install it first: pip install platformio" >&2
  exit 1
fi

build_env() {
  local project="$1"
  local env="$2"
  echo "==> pio run -d firmware/${project} -e ${env}"
  pio run -d "firmware/${project}" -e "${env}"
}

build_env twin_wings twin_wings_esp32c3
build_env twin_wings twin_wings_esp32
build_env twin_wings twin_wings_f411
build_env twin_wings twin_wings_f405

build_env single_wing single_wing_esp32c3
build_env single_wing single_wing_esp32
build_env single_wing single_wing_f411
build_env single_wing single_wing_f405

build_env elrs elrs_tx
build_env elrs elrs_rx

build_env lora lora_433
build_env lora lora_868
build_env lora lora_915

build_env espnow transmitter
build_env espnow receiver

echo "All firmware environments built successfully."
