# Firmware

All ESP32 / PlatformIO projects in this repo live here. Each subfolder is an
independent PlatformIO project with its own `platformio.ini`, built with
`pio run -d firmware/<project>`.

| Project | Purpose | Radio / bus |
|---------|---------|-------------|
| [`espnow/`](espnow/) | Custom TX/RX link: IMU + joystick manual flight, and autonomous command relay from the ROS 2 stack | ESP-NOW (2.4 GHz), outputs CRSF / SBUS / PPM / iBus to the flight controller |
| [`elrs/`](elrs/) | ExpressLRS-compatible TX/RX for autonomous flight — computer sends CRSF over UART to TX, RX emits CRSF to the FC | SX1280 2.4 GHz FLRC |
| [`lora/`](lora/) | Point-to-point LoRa template for long-range, low-rate telemetry or a backup command channel | SX1276/SX1278/RFM9x (433/868/915 MHz) |
| [`gps/`](gps/) | GPS bring-up and telemetry module (NMEA + PMTK + UBX) | UART to GPS module |
| [`twin_wings/`](twin_wings/) | Twin Motor Flying Wing firmware: 2 motors through ESCs, 2 elevon servos, optional IMU/GPS | ESC/servo PWM |
| [`single_wing/`](single_wing/) | Single Motor Flying Wing firmware: 1 motor through ESC, elevon/aileron/elevator servos, optional IMU/GPS | ESC/servo PWM |

Host-side tools that talk to these boards over serial live in [`../tools/`](../tools/).

## Build

```bash
# Install PlatformIO, then build any project from the repo root:
pio run -d firmware/espnow
pio run -d firmware/elrs
pio run -d firmware/lora
pio run -d firmware/gps

# Upload a specific environment
pio run -d firmware/gps -e gps_auto -t upload
```

Environment names are defined per project — see each project's `platformio.ini`
and README.

A containerized PlatformIO toolchain is available: see [`../docker/README.md`](../docker/README.md).

To build every firmware environment in one pass:

```bash
bash scripts/build_firmware.sh
```

The current firmware target matrix is:

| Project | Environments |
|---------|--------------|
| `twin_wings` | `twin_wings_esp32c3`, `twin_wings_esp32`, `twin_wings_f411`, `twin_wings_f405` |
| `single_wing` | `single_wing_esp32c3`, `single_wing_esp32`, `single_wing_f411`, `single_wing_f405` |
| `elrs` | `elrs_tx`, `elrs_rx` |
| `lora` | `lora_433`, `lora_868`, `lora_915` |
| `espnow` | `transmitter`, `receiver` |

## Airframe support

These projects are airframe-agnostic: they carry RC channels and telemetry, and
do not assume a particular vehicle. Airframe-specific behavior (mixing,
allocation, control laws) lives in the control stack, not here — see the
airframe table in the [root README](../README.md#supported-airframes).
