# UAV-Controller

Multi-purpose control system for quadcopters. Supports autonomous control, hardware integration, and simulation.

## Features

- **Controllers**: PID (cascaded), LQR (optimal), MPC (WIP), RL (WIP)
- **Safety**: Multi-layer validation, slew limiting, timeout watchdog
- **Hardware**: Integrates with TX_RX via CRSF protocol
- **Simulation**: 200Hz dynamics with RViz

## Architecture

**Manual Flight**:
```
TX (IMU+Joystick) → ESP-NOW → RX → CRSF → FC
```
No computer. TX_RX handles everything.

**Autonomous**:
```
ROS Controllers → Safety Gate → CRSF Adapter → UDP → TX → ESP-NOW → RX → CRSF → FC
```
Computer runs algorithms. TX relays commands.

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

## Notes

- ROS 2 Humble required
- TX_RX handles manual flight (no ROS needed)
- MPC and RL controllers under development (lab collaboration)

---

## TODOs
- Add LoRa module for TX & RX
- Field test with manual control & tuning guide
- Hardware demo
- MPC & RL 


