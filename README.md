# UAV CRSF LQR PID Controller

A ROS 2 Humble setup for firmware-agnostic UAV control. Includes LQR and PID controllers, safety features, CRSF adapter for talking to your TX/RX hardware, and a simulator so you can test stuff before you actually fly.

## Architecture

```
┌─────────────┐
│ Controllers │──► LQR or PID
└──────┬──────┘
       │ /cmd/body_rate_thrust or /cmd/attitude_thrust
       ▼
┌─────────────┐
│ Safety Gate │──► Clamps, slew limits, timeout watchdog
└──────┬──────┘
       │
       ├─► [mode=sim] /cmd/final/body_rate_thrust ──► Simulator ──► RViz
       │
       └─► [mode=crsf] /cmd/final/rc ──► CRSF Adapter ──► UDP/Serial ──► ../TX_RX (ESP32) ──► CRSF UART ──► Betaflight
```

## Frame Conventions

- **Controllers**: Use FRD (Front-Right-Down) body rates for firmware-agnostic control
- **Simulator**: Publishes ENU (East-North-Up) odometry with TF transforms provided
- **Conversion**: Safety gate and adapter handle frame transformations as needed

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

### Simulation with LQR

```bash
./scripts/run_sim_lqr.sh
```

### Simulation with PID

```bash
./scripts/run_sim_pid.sh
```

### Hardware with CRSF (LQR)

```bash
./scripts/run_crsf_link_lqr.sh transport:=udp udp_host:=192.168.4.1 udp_port:=9000
```

### Hardware with CRSF (PID)

```bash
./scripts/run_crsf_link_pid.sh transport:=serial serial_port:=/dev/ttyUSB0 baud:=921600
```

## Package Overview

### common_msgs

Custom message definitions:
- `BodyRateThrust.msg` - Body rates (rad/s, FRD) + thrust [0..1]
- `AttitudeThrust.msg` - Quaternion attitude + thrust
- `VirtualRC.msg` - Virtual RC channels for CRSF transmission

### safety_gate

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

### controllers_lqr

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

### controllers_pid

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

### adapters_crsf

Python node bridging ROS 2 to ESP32 TX/RX project via UDP or Serial:
- **Pluggable packer API**: Customize packet format without changing node
- **Default packer**: `txrx_packer` matches `../TX_RX/` format (struct `<ffff12f>`)
- **Transport modes**: UDP (wireless ESP32-C3) or Serial (wired)
- **Failsafe**: Sends safe idle packet if commands stale > `failsafe_ms`
- **Rate control**: Configurable streaming rate (default 150 Hz)

**Key parameters** (`crsf_params.yaml`):
- `transport`: udp | serial
- `udp_host`: 192.168.4.1, `udp_port`: 9000
- `serial_port`: /dev/ttyUSB0, `baud`: 921600
- `rate_hz`: 150, `failsafe_ms`: 200
- `packer`: txrx_packer (or custom packer module)

**Packer customization**:
See `adapters_crsf/packers/base.py` for abstract interface. Implement `encode()` and `safe_idle()` methods to match your TX_RX firmware.

### sim_dyn

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

### Using DevContainer (Recommended)

1. Open in VS Code with Remote-Containers extension
2. Reopen in container (uses `.devcontainer/devcontainer.json`)
3. Inside container:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### Using Docker

```bash
docker build -t uav_crsf_lqr_pid -f docker/Dockerfile.humble .
docker run -it --rm uav_crsf_lqr_pid
```

### Native Build (Ubuntu 22.04 + ROS 2 Humble)

```bash
./scripts/build.sh
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

## Packer Customization for TX_RX Integration

The CRSF adapter uses a pluggable packer system to match your ESP32 firmware's packet format.

**Default packer** (`txrx_packer.py`):
- Format: `<ffff12f` (little-endian floats)
- Fields: roll, pitch, yaw, throttle, aux[0..11]
- Range: [-1, 1] for sticks, [0, 1] for throttle

**Custom packer example**:

```python
# adapters_crsf/packers/my_packer.py
from .base import PacketPacker
import struct

class MyPacker(PacketPacker):
    def encode(self, rc_msg):
        # Your custom format here
        return struct.pack('<HHH', 
            int((rc_msg.roll + 1) * 1000),
            int((rc_msg.pitch + 1) * 1000),
            int(rc_msg.throttle * 1000))
    
    def safe_idle(self):
        return struct.pack('<HHH', 1000, 1000, 0)
```

Then use: `packer:=my_packer` in launch arguments.

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

## Development & CI

### Running Tests

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

### Linting

```bash
# C++ (clang-format)
find ros2_ws/src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i

# Python (flake8)
flake8 ros2_ws/src/adapters_crsf ros2_ws/src/sim_dyn
```

### GitHub Actions

CI workflow automatically runs on push/PR:
- colcon build (all packages)
- C++ tests (LQR, PID)
- Python import checks
- Linters (ament, clang-format, flake8)

## Troubleshooting

**Q: Simulator diverges/oscillates**  
A: Check `hover_thrust` parameter matches your thrust model. Default 0.5 should work with default c0/c1.

**Q: CRSF adapter not connecting to ESP32**  
A: Verify UDP host/port. Try `ping 192.168.4.1` to check ESP32 reachability. Check ESP32 serial monitor for received packets.

**Q: PID has steady-state error**  
A: Increase `ki` slightly or check if `i_clamp` is too restrictive.

**Q: LQR unstable**  
A: Verify K matrix dimensions (6×6). Check closed-loop eigenvalues with test. Ensure `hover_thrust` is correct.

**Q: Safety gate always triggers timeout**  
A: Check controller is publishing at >5 Hz. Increase `timeout_ms` if running on slow hardware.

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [CRSF Protocol](https://github.com/crsf-wg/crsf/wiki)
- [Betaflight](https://betaflight.com/)
- [LQR Theory](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
