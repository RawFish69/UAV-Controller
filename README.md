# UAV-Controller

A modular ROS 2 Humble system for firmware-agnostic UAV control. Supports autonomous control (LQR/PID), manual gesture control (hand IMU), safety features, hardware integration via CRSF, and full simulation.

## What Can It Do?

1. **Autonomous Control** - LQR or PID for waypoint tracking and autonomous missions
2. **Manual IMU Gestures** - Hand-mounted IMU TX for intuitive gesture-based piloting
3. **Simulation** - Test everything safely with RViz before flying
4. **Hardware Ready** - Works with TX_RX ESP32 project and Betaflight

## Documentation

- **docs/SETUP_GUIDE.md** - Build, run sim, tune, test
- **docs/HARDWARE.md** - All ESP32 setup (IMU TX + TX_RX integration)

## Architecture

### Control Modes

**1. Autonomous** - LQR/PID controllers for waypoints and missions

**2. Manual IMU TX** - Gesture control with hand-mounted IMU

**3. Simulation** - Safe testing with RViz

### System Stages

**1. Command Generation**
- **PID Controller**: Cascaded attitude + rate control with anti-windup
- **LQR Controller**: Optimal state-space control  
- **IMU Controller**: Gesture-based attitude setpoints from hand IMU
- Output: Body rates (rad/s, FRD) + thrust [0..1] or attitude + thrust

**2. Safety Gate** → Validates and routes commands
- Clamps body rates and thrust to safe limits
- Applies slew rate limiting (prevents sudden changes)
- Timeout watchdog (reverts to safe idle if no commands)
- Routes to either simulator or hardware based on mode

**3. Output Stage** → Depends on mode
- **Simulation mode**: Commands → dynamics simulator → RViz
- **Hardware mode**: Commands → CRSF adapter → TX_RX → Flight controller

### Data Flow Examples

**Autonomous (LQR) in Sim**:
```
LQR → /cmd/body_rate_thrust → Safety Gate → /cmd/final/body_rate_thrust → Simulator → RViz
                                     ↑
                              /state/attitude + /state/angular_velocity
```

**Manual IMU TX**:
```
IMU TX → Protocol Receiver → IMU Controller → PID Outer → Safety Gate → Hardware
```

**Hardware Path**:
```
Controller → Safety Gate → /cmd/final/rc → CRSF Adapter → UDP/Serial → TX_RX ESP32 → Betaflight
```

### Frame Conventions

- **Controllers**: FRD (Front-Right-Down) body rates - standard for quadcopters
- **Simulator**: ENU (East-North-Up) odometry - standard ROS convention
- **IMU**: Orientation mapped to FRD drone frame
- **Conversions**: Handled automatically by simulator and safety gate

## Topic API

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd/body_rate_thrust` | `BodyRateThrust` | Controller output (body rates + thrust) |
| `/cmd/attitude_thrust` | `AttitudeThrust` | Alternative attitude setpoint |
| `/cmd/final/body_rate_thrust` | `BodyRateThrust` | Safety-checked commands (sim mode) |
| `/cmd/final/rc` | `VirtualRC` | Virtual RC channels (crsf mode) |
| `/state/odom` | `nav_msgs/Odometry` | Simulated odometry (ENU frame) |
| `/state/attitude` | `geometry_msgs/QuaternionStamped` | Current attitude |
| `/state/angular_velocity` | `geometry_msgs/Vector3Stamped` | Current body rates |

## Quick Start

### Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

### Run Simulation

**Autonomous PID** (recommended first):
```bash
source ros2_ws/install/setup.bash
ros2 launch sim_dyn sim_pid.launch.py
```

**Autonomous LQR**:
```bash
source ros2_ws/install/setup.bash
ros2 launch sim_dyn sim_lqr.launch.py
```

**Manual IMU Control**:
```bash
source ros2_ws/install/setup.bash

# Option 1: Use launch file (starts controller + PID + safety)
ros2 launch manual_imu_controller imu_manual_control.launch.py mode:=sim

# Then separately run:
# - Simulator: ros2 launch sim_dyn sim_pid.launch.py (without PID node)
# - IMU driver: ros2 run your_imu_driver imu_node
# - Throttle source: ros2 topic pub /manual/throttle std_msgs/Float32 "data: 0.5"
```

RViz should open showing the quad. For autonomous mode, it hovers at 1m altitude. For manual mode, it responds to hand gestures.

### Monitor Topics

In another terminal:
```bash
source ros2_ws/install/setup.bash

# Check nodes are running
ros2 node list

# Watch controller output
ros2 topic echo /cmd/body_rate_thrust

# For IMU control, watch gesture commands
ros2 topic echo /cmd/attitude_thrust

# Watch simulator state
ros2 topic echo /state/odom

# Debug topic flow
./scripts/debug_topics.sh
```

### Hardware Operation

**Autonomous controllers**:
```bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1
```

**Manual IMU control**:
```bash
source ros2_ws/install/setup.bash
ros2 launch manual_imu_controller imu_manual_control.launch.py mode:=crsf
ros2 launch adapters_crsf crsf_adapter.launch.py transport:=udp

# Run IMU driver
ros2 run your_imu_driver imu_node
```

See `docs/SETUP_GUIDE.md` and `docs/HARDWARE.md` for detailed setup.

## Package Overview

### Control Packages

#### controllers_pid

C++ cascaded attitude + rate PID controller (see below for full details)

#### controllers_lqr

C++ LQR optimal controller (see below for full details)

#### manual_imu_controller

Python nodes for gesture control:
- **imu_protocol_receiver**: Receives from IMU TX ESP32 via custom protocol
- **imu_controller_node**: Maps orientation to drone attitude setpoints
- Configurable scaling, deadzones, limits
- Works with PID outer loop

See `docs/HARDWARE.md` for IMU TX hardware setup.

### Infrastructure Packages

#### common_msgs

Custom message definitions:
- `BodyRateThrust.msg` - Body rates (rad/s, FRD) + thrust [0..1]
- `AttitudeThrust.msg` - Quaternion attitude + thrust
- `VirtualRC.msg` - Virtual RC channels for CRSF transmission

#### safety_gate

C++ node providing safety checks and mode switching:
- **Mode switching**: `sim` (outputs body_rate_thrust) or `crsf` (outputs VirtualRC)
- **Safety limits**: Max tilt angle, max body rates, thrust range
- **Slew rate limiting**: Prevents rapid thrust/yaw changes
- **Timeout watchdog**: Reverts to safe idle if commands stop

**Key parameters** (`safety_params.yaml`):
- `mode`: sim | crsf
- `max_tilt_deg`: 35.0
- `max_body_rate`: 10.0 rad/s
- `thrust_min`: 0.05, `thrust_max`: 0.9
- `timeout_ms`: 200

#### controllers_lqr (Detailed)

C++ node implementing Linear Quadratic Regulator for attitude control:
- **State**: x = [φ, θ, ψ, p, q, r] (roll, pitch, yaw, body rates)
- **Linearized model**: Small-angle approximation around hover
- **Eigen-based**: Fast matrix operations for real-time control
- **Optimal gains**: K matrix computed from LQR solution

**Key parameters** (`lqr_params.yaml`):
- Inertia: `Jx`, `Jy`, `Jz`
- `mass`, `hover_thrust`
- Gain matrix `K` (6×6)
- Optional: A, B, Q, R matrices for online LQR computation

**Test**: `test_lqr_stability.cpp` verifies closed-loop stability

#### controllers_pid (Detailed)

C++ node implementing cascaded attitude + rate PID controller:
- **Inner loop**: Body rate PID (roll, pitch, yaw)
- **Outer loop** (optional): Attitude P controller generating rate setpoints
- **Anti-windup**: Integrator clamping + back-calculation
- **Derivative filtering**: 1st-order low-pass on measured rate
- **Feed-forward** (optional): Direct rate setpoint pass-through

**Key parameters** (`pid_params.yaml`):
- `use_attitude_outer`: true | false
- Per-axis gains: `kp`, `ki`, `kd`, `kff`
- Derivative filter: `tau_d`
- Anti-windup: `i_clamp`, `k_aw`
- `rate_limits`, `thrust_limits`

**Test**: `test_pid_antiwindup.cpp` verifies anti-windup and derivative filter stability

#### adapters_crsf

Python node bridging ROS 2 to ESP32 TX/RX project via UDP or Serial:
- **Pluggable packer API**: Customize packet format without changing node
- **Default packer**: `txrx_packer` sends DirectCommandPayload format (20 bytes: 4 floats + timestamp)
- **Transport modes**: UDP (wireless) or Serial (wired to TX)
- **Failsafe**: Sends safe idle packet if commands stale > `failsafe_ms`
- **Rate control**: Configurable streaming rate (default 150 Hz)

**Key parameters** (`crsf_params.yaml`):
- `transport`: udp | serial
- `udp_host`: 192.168.4.1, `udp_port`: 9000
- `serial_port`: /dev/ttyUSB0, `baud`: 115200
- `rate_hz`: 150, `failsafe_ms`: 200
- `packer`: txrx_packer (or custom packer module)

**TX_RX Integration**:
The TX firmware needs modification to accept UDP/Serial input. See `docs/HARDWARE.md` for complete guide with example code.

#### sim_dyn

Python simulator with rigid-body dynamics and RViz visualization:
- **Integration**: 200 Hz RK4 for smooth attitude propagation
- **Dynamics**: Attitude kinematics from body rates, thrust-based acceleration
- **Thrust model**: T = mg · (c₀ + c₁ · u) calibrated for hover ≈ 0.5
- **Drag** (optional): Linear velocity damping
- **Outputs**: Odometry (ENU), attitude, angular velocity, TF transforms
- **Visualization**: RViz with body axes and trajectory

**Key parameters** (`sim_params.yaml`):
- `mass`, `hover_thrust`
- `c0`, `c1` (thrust model coefficients)
- `kv_drag` (linear drag coefficient)
- `gravity`: 9.81

## Building

### Native Build (Recommended)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Rebuild single package**:
```bash
colcon build --packages-select controllers_pid
source install/setup.bash
```

### Using DevContainer

1. Open in VS Code with Remote-Containers extension
2. Reopen in container
3. Inside container: `cd ros2_ws && colcon build`

### Using Docker

```bash
docker build -t uav-controller -f docker/Dockerfile.humble .
docker run -it --rm --network=host uav-controller
```

## Tuning Tips

### LQR Tuning

1. Start with identity Q and R matrices
2. Increase Q diagonal elements for states you want tighter control
3. Increase R diagonal elements to penalize control effort (smoother response)
4. Verify closed-loop eigenvalues have negative real parts
5. Test with `ros2 launch sim_dyn sim_lqr.launch.py`

### PID Tuning

1. **Start with P-only**: Set `ki` and `kd` to 0, tune `kp` until ~80% desired response
2. **Add D term**: Increase `kd` to reduce overshoot, adjust `tau_d` if noisy
3. **Add I term**: Small `ki` to eliminate steady-state error, set `i_clamp` conservatively
4. **Anti-windup**: If saturation occurs often, increase `k_aw` (0.3–0.5 typical)
5. **Feed-forward** (optional): Add `kff` for faster response to known commands

**Typical starting values** (250-size quad):
- Rate P: 8.0, I: 0.6, D: 0.05
- Attitude P: 6.0
- Tau_d: 0.02 s (50 Hz filter)

### Safety Gate Tuning

- `max_tilt_deg`: Start conservative (30°), increase as confidence grows
- `timeout_ms`: 200 ms typical (5 Hz minimum command rate)
- `slew_rate_thrust`: 2.0–5.0 per second prevents sudden altitude changes
- `slew_rate_yaw`: 3.0–10.0 rad/s prevents aggressive spins

## TX_RX Hardware Integration

The CRSF adapter sends **DirectCommandPayload** format (20 bytes) to your TX ESP32:
- 4 floats: roll, pitch, yaw (-1 to 1), throttle (0 to 1)
- 1 uint32: timestamp

**Your TX firmware needs modification** to accept these packets via UDP or Serial and call `CustomProtocol_SendDirectCommand()`.

See **`TX_RX_INTEGRATION.md`** for:
- Complete integration guide
- Example code for UDP server
- Serial input parsing
- Testing procedures
- Troubleshooting tips

**Quick example** (add to TX firmware):
```cpp
WiFiUDP udp;
udp.begin(9000);

// In loop():
if (udp.parsePacket() == 20) {
  uint8_t buf[20];
  udp.read(buf, 20);
  float* f = (float*)buf;
  CustomProtocol_SendDirectCommand(f[0], f[1], f[2], f[3]);
}
```

## Safety Checklist (Before Hardware Flight)

- [ ] **Props OFF** for all initial tests
- [ ] Verify AUX channel mapping for arming (match Betaflight AUX configuration)
- [ ] Test failsafe: kill ROS node, verify TX_RX sends failsafe CRSF packets
- [ ] Verify CRSF link: `crsf_adapter_node` reports healthy TX to ESP32
- [ ] Check control authority: commanded rates match Betaflight blackbox data
- [ ] Test safety_gate timeout: stop publishing commands, verify safe idle
- [ ] Confirm thrust range: `hover_thrust` parameter achieves stable hover
- [ ] **Manual mode ready**: Always have manual RC override available
- [ ] **Kill switch**: Hardware arming switch on transmitter

## Testing & Development

### Run Tests

```bash
cd ros2_ws

# Build with tests
colcon build

# Run all tests
colcon test

# See results
colcon test-result --verbose

# Run specific package tests
colcon test --packages-select controllers_pid
colcon test-result --verbose
```

### Monitor Performance

```bash
# Check node rates
ros2 topic hz /cmd/body_rate_thrust  # ~100 Hz
ros2 topic hz /state/odom            # ~200 Hz

# Debug topic flow
./scripts/debug_topics.sh

# Watch specific topics
ros2 topic echo /cmd/final/body_rate_thrust
```

### Tuning Workflow

1. Edit config file (e.g., `ros2_ws/src/controllers_pid/config/pid_params.yaml`)
2. Rebuild: `colcon build --packages-select controllers_pid`
3. Relaunch: `ros2 launch sim_dyn sim_pid.launch.py`
4. Monitor and iterate

### Linting

```bash
# C++
find ros2_ws/src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i

# Python
flake8 ros2_ws/src/adapters_crsf ros2_ws/src/sim_dyn
```

## Common Workflows

### Workflow 1: First Time Running Sim

```bash
# Terminal 1: Build
cd UAV-Controller/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch sim
ros2 launch sim_dyn sim_pid.launch.py

# You should see:
# - RViz window with quad at 1m altitude
# - Terminal showing controller running at ~100Hz
# - No error messages
```

### Workflow 2: Tuning PID Gains

```bash
# Terminal 1: Run sim
cd UAV-Controller
source ros2_ws/install/setup.bash
ros2 launch sim_dyn sim_pid.launch.py

# Terminal 2: Monitor performance
source ros2_ws/install/setup.bash
ros2 topic echo /cmd/body_rate_thrust | grep thrust

# Terminal 3: Edit gains while watching behavior
nano ros2_ws/src/controllers_pid/config/pid_params.yaml
# Make changes, save

# Back in Terminal 1: Ctrl+C to stop, then:
ros2 launch sim_dyn sim_pid.launch.py
```

### Workflow 3: Testing Safety Gate

```bash
# Terminal 1: Run sim
ros2 launch sim_dyn sim_pid.launch.py

# Terminal 2: Kill controller to test timeout
ros2 lifecycle set /pid_controller shutdown

# Watch in RViz - quad should enter failsafe mode
# Check terminal output for safety gate messages
```

### Workflow 4: Manual Control Testing

```bash
# Terminal 1: Run just the simulator
ros2 run sim_dyn dynamics_node

# Terminal 2: Manually send commands
ros2 topic pub /cmd/final/body_rate_thrust \
  common_msgs/msg/BodyRateThrust \
  '{body_rates: {x: 0.0, y: 0.0, z: 0.0}, thrust: 0.5}' -r 100

# Try different thrust values to see response
```

### Workflow 5: Comparing Controllers

```bash
# Test PID
ros2 launch sim_dyn sim_pid.launch.py
# Observe behavior for 30 seconds, note any oscillations

# Ctrl+C, then test LQR
ros2 launch sim_dyn sim_lqr.launch.py
# Compare smoothness and response time
```

## Troubleshooting

**Q: Simulator diverges/oscillates**  
A: Check `hover_thrust` parameter matches your thrust model. Default 0.5 should work with c1=2.0.

**Q: Quad is falling in sim**
```bash
# Check thrust model
ros2 param get /dynamics_node c1  # Should be 2.0
ros2 param get /dynamics_node hover_thrust  # Should be 0.5

# Check controller is running
ros2 node list | grep controller

# Check commands are flowing
ros2 topic hz /cmd/body_rate_thrust  # Should be ~100 Hz
```

**Q: CRSF adapter not connecting to ESP32**
```bash
# Test connectivity
ping 192.168.4.1

# Check if TX is listening
# Look at TX serial monitor for UDP server messages

# Verify ROS is sending
ros2 topic echo /cmd/final/rc
```

**Q: Oscillations in sim**
```bash
# Reduce P gains
# Edit: ros2_ws/src/controllers_pid/config/pid_params.yaml
# Change kp: {x: 6.0, y: 6.0, z: 2.0}

# Or add more damping
# Increase kd: {x: 0.1, y: 0.1, z: 0.05}

# Rebuild and test
cd ros2_ws
colcon build --packages-select controllers_pid
source install/setup.bash
ros2 launch sim_dyn sim_pid.launch.py
```

**Q: Nodes not starting**
```bash
# Make sure you sourced the workspace
source ros2_ws/install/setup.bash

# Check for build errors
cd ros2_ws
colcon build --event-handlers console_direct+

# Check ROS 2 environment
ros2 doctor
```

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [CRSF Protocol](https://github.com/crsf-wg/crsf/wiki)
- [Betaflight](https://betaflight.com/)
- [LQR Theory](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
