# twin_wings

Independent PlatformIO firmware for a **Twin Motor Flying Wing**:

- 2 motors through ESCs
- 2 elevon servos
- Optional IMU and GPS support

This is a scaffold. The mixing skeleton is intentionally small and board-agnostic; the
control loop and sensor drivers are the next layer to add, using PX4 / ArduPilot /
Betaflight as references for fixed-wing control performance.

## Targets

| Environment | Board | MCU |
|-------------|-------|-----|
| `twin_wings_esp32c3` | `esp32-c3-devkitm-1` | ESP32-C3 |
| `twin_wings_esp32` | `esp32dev` | ESP32 WROOM |
| `twin_wings_f411` | `blackpill_f411ce` | STM32F411CE |
| `twin_wings_f405` | `genericSTM32F405RG` | STM32F405RG |

## Build and upload

```bash
cd firmware/twin_wings
pio run -e twin_wings_esp32c3
pio run -e twin_wings_esp32c3 -t upload
```

STM32 targets use the STM32duino Arduino core and may require selecting an upload protocol
for your board (ST-Link, DFU, serial). Adjust `platformio.ini` for your wiring.

## Pin defaults

Pins are defined in `src/main.cpp` with `#ifdef` guards per target. Treat these as
placeholders until the real flight controller board is chosen.

## PWM output

`src/pwm_output.cpp` provides a small board-aware PWM layer:

- ESP32: LEDC at 50 Hz for servos and 400 Hz for motors, 12-bit duty.
- STM32: `analogWrite` with 50 Hz / 400 Hz setup via `analogWriteFrequency`.

The servo neutral, travel, and ESC arming/calibration values still need to be tuned for
your actual hardware before flight.

## Bench control input

`src/control_input.cpp` accepts a simple serial command for bench testing:

```text
roll,pitch,yaw,throttle
```

Example: `0.2,-0.1,0.0,0.35` followed by Enter. This is not a flight input; replace it
with CRSF/SBUS/PPM or an autopilot command parser before flying.

## CRSF RC input

`src/rc_input.cpp` provides `rcInputReadCrsf(Stream&, ControlInput&)` for parsing
CRSF `RC_CHANNELS_PACKED` frames. The channel mapping is CH1 roll, CH2 pitch, CH3
throttle, CH4 yaw. Assign a receiver UART in `main.cpp` when ready.

## SBUS RC input

`src/rc_input.cpp` also provides `rcInputReadSbus(Stream&, ControlInput&)`. Set
`RC_INPUT_PROTOCOL` to `2` in `src/config.h`; the init helper uses inverted UART on
ESP32. On STM32, SBUS may require an external inverter circuit.

## Sensors

`src/sensors.h` defines the IMU and GPS interfaces. `src/sensors.cpp` now contains a
dependency-free MPU6050 driver over I2C (address `0x68`, ±8 g accel, ±500 deg/s gyro) that
populates `ImuData`, plus a minimal NMEA parser for GPS (`$GPGGA` fix/lat/lon/alt and
`$GPRMC` groundspeed). IMU I2C and GPS UART pins are in `src/config.h`.

## Controller

`src/controller.cpp` contains a proportional attitude/rate controller. It maps
roll/pitch/yaw errors to normalized actuator commands with gyro-rate damping and integral
trim (`kp*`/`kd*`/`ki*` gains) before `applyTwinWingMix()` converts them to motor/elevon
outputs. Tune those gains before flight, then wire L1/TECS guidance.

## GPS guidance

Set `ENABLE_GPS_GUIDANCE` to `1` in `src/config.h` and fill `WAYPOINT_LAT` /
`WAYPOINT_LON` / `WAYPOINT_ALT_M`. When a GPS fix is available, `guidanceToTarget()` in
`src/navigation.cpp` will override the bench serial input with L1/TECS commands to the
configured waypoint.
