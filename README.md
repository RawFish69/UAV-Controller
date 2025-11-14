# UAV-Controller

Multi-purpose control system for quadcopters. Supports autonomous control, hardware integration, and simulation.

## Features

- **Controllers**: PID (cascaded), LQR (optimal), MPC (WIP), RL (WIP)
- **Safety**: Multi-layer validation, slew limiting, timeout watchdog
- **Hardware**: Custom TX/RX with universal protocol support (CRSF, SBUS, PPM, iBus, FrSky)
- **TX/RX System**: Hybrid IMU+Joystick control, ESP-NOW wireless, multi-protocol receiver
- **Simulation**: 200Hz dynamics with RViz

## Architecture

**Manual Flight**:
```
TX (IMU+Joystick) → ESP-NOW → RX → Protocol Bridge → FC
                                     (CRSF/SBUS/PPM/iBus/FrSky)
```
No computer. TX_RX handles everything. Universal receiver works with any flight controller.

**Autonomous**:
```
ROS Controllers → Safety Gate → CRSF Adapter → UDP → TX → ESP-NOW → RX → Protocol → FC
```
Computer runs algorithms. TX relays commands. Protocol configurable for your FC.

**Simulation**:
```
Controllers → Safety Gate → Simulator → RViz
```
Software only. Algorithm development and tuning.

## Quick Start

### Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Simulation

```bash
ros2 launch sim_dyn sim_pid.launch.py  # PID
ros2 launch sim_dyn sim_lqr.launch.py  # LQR
```

### Run Autonomous (Hardware)

TX firmware needs UDP modification. See docs/HARDWARE.md.

```bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1
```

## Packages

| Package | Type | Purpose |
|---------|------|---------|
| `controllers_pid` | C++ | Cascaded PID with anti-windup |
| `controllers_lqr` | C++ | State-space LQR controller |
| `safety_gate` | C++ | Validation, limiting, routing |
| `adapters_crsf` | Python | ROS → TX bridge (UDP/Serial) |
| `sim_dyn` | Python | Dynamics + RViz |
| `common_msgs` | Messages | Custom message types |
| `TX_RX` | ESP32 | Custom transmitter/receiver system |
| `Utils` | Python | Universal protocol visualizer & monitor |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd/body_rate_thrust` | BodyRateThrust | Controller output |
| `/cmd/final/body_rate_thrust` | BodyRateThrust | After safety (sim) |
| `/cmd/final/rc` | VirtualRC | After safety (hardware) |
| `/state/odom` | Odometry | Current state |
| `/state/attitude` | QuaternionStamped | Orientation |
| `/state/angular_velocity` | Vector3Stamped | Body rates |

## Configuration

Config files in each package's `config/` directory:

- `pid_params.yaml` - Gains, anti-windup, limits
- `lqr_params.yaml` - K matrix, system model
- `safety_params.yaml` - Safety limits, timeouts
- `crsf_params.yaml` - TX connection settings
- `sim_params.yaml` - Dynamics model parameters

## Testing

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

## Monitoring

```bash
ros2 node list
ros2 topic hz /cmd/body_rate_thrust
ros2 topic echo /state/odom
./scripts/check_sim.sh
```

## Troubleshooting

**Quad falling in sim**: Check `c1=2.0` in `sim_params.yaml`  
**Nodes not starting**: Run `source ros2_ws/install/setup.bash`  
**Oscillations**: Reduce PID kp or increase kd gains

## Documentation

- **docs/SETUP_GUIDE.md** - Build, run, tune
- **docs/HARDWARE.md** - TX integration for autonomous mode
- **TX_RX/README.md** - Complete TX/RX system documentation
- **Utils/README.md** - Universal protocol visualizer & monitor

## TX_RX System

Custom ESP32-based transmitter and receiver for manual flight control:

**Transmitter Features:**
- Hybrid IMU (BNO085/MPU6050) + Joystick control
- ESP-NOW wireless (low latency)
- Configurable sensitivity and update rates

**Receiver Features:**
- **Universal Protocol Support**: Works with any flight controller
  - ✅ CRSF (Crossfire/ELRS) - Betaflight, INAV
  - ✅ SBUS (Futaba) - Universal compatibility  
  - ✅ PPM - Traditional flight controllers
  - ✅ iBus (FlySky) - FlySky receivers
  - ✅ FrSky S.PORT - FrSky telemetry systems
- Easy protocol switching via single config file
- Automatic failsafe handling

**Quick Setup:**
1. Edit `TX_RX/src/config.h` to select protocol
2. Flash transmitter and receiver
3. Configure flight controller to match protocol

See `TX_RX/README.md` for complete documentation.

## Notes

- ROS 2 Humble required for autonomous control
- TX_RX handles manual flight (no ROS/computer needed)
- Universal receiver works with Betaflight, INAV, ArduPilot, and traditional FCs
- MPC and RL controllers under development (lab collaboration)

---

## TODOs
- Add LoRa module for TX & RX
- Field test with manual control & tuning guide
- Hardware demo
- MPC & RL 


