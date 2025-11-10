# UAV CRSF LQR PID - Repository Summary

## Complete Repository Structure

This repository contains a production-ready ROS 2 Humble system for UAV control with the following components:

### Packages Created

1. **common_msgs** - Custom message definitions
   - `BodyRateThrust.msg` - Body rates (FRD) + thrust
   - `AttitudeThrust.msg` - Quaternion attitude + thrust
   - `VirtualRC.msg` - Virtual RC channels (16 channels)

2. **safety_gate** (C++) - Safety checks and mode switching
   - Clamps body rates and thrust
   - Slew rate limiting (thrust & yaw)
   - Timeout watchdog (200ms default)
   - Dual mode: `sim` or `crsf`

3. **controllers_lqr** (C++ with Eigen) - LQR attitude controller
   - State: [φ, θ, ψ, p, q, r]
   - Linearized hover model
   - Configurable gain matrix K (4×6)
   - Unit tests for stability

4. **controllers_pid** (C++ with Eigen) - Cascaded PID controller
   - Inner loop: Rate PID (roll, pitch, yaw)
   - Outer loop: Attitude P controller (optional)
   - Anti-windup: Integrator clamping + back-calculation
   - Derivative filtering (1st-order low-pass)
   - Feed-forward support
   - Unit tests for anti-windup

5. **adapters_crsf** (Python) - CRSF adapter with pluggable packers
   - UDP or Serial transport
   - Pluggable packer API (`PacketPacker` abstract base)
   - Default `txrx_packer` matches ../TX_RX/ format
   - Failsafe with safe idle packets
   - UDP-to-Serial bridge utility

6. **sim_dyn** (Python) - Dynamics simulator with RViz
   - 200 Hz RK4 integration
   - Rigid-body attitude kinematics
   - Thrust model: T = mg(c₀ + c₁·u)
   - Optional linear drag
   - ENU odometry output
   - TF transforms (map → base_link)
   - RViz visualization config

### Launch Configurations

**Simulation:**
- `./scripts/run_sim_lqr.sh` - LQR + simulator + RViz
- `./scripts/run_sim_pid.sh` - PID + simulator + RViz

**Hardware:**
- `./scripts/run_crsf_link_lqr.sh` - LQR + CRSF adapter
- `./scripts/run_crsf_link_pid.sh` - PID + CRSF adapter

**Build:**
- `./scripts/build.sh` - Colcon build + source

### Configuration Files

Each package includes YAML config files with well-documented parameters:
- `safety_params.yaml` - Safety limits and timeouts
- `lqr_params.yaml` - LQR gains and model parameters
- `pid_params.yaml` - PID gains, anti-windup, derivative filters
- `crsf_params.yaml` - Transport settings, packer selection
- `sim_params.yaml` - Dynamics model parameters

### Testing

**C++ Tests:**
- `test_lqr_stability.cpp` - Closed-loop eigenvalue checks
- `test_pid_antiwindup.cpp` - Anti-windup, derivative filter, limits

**Python:**
- Import sanity checks in CI
- Flake8 linting

### Docker & DevContainer

- `.devcontainer/` - VS Code DevContainer with ROS 2 Humble
- `docker/Dockerfile.humble` - Standalone Docker build

### CI/CD

`.github/workflows/ci.yaml`:
- Colcon build
- C++ tests (ctest)
- Python import checks
- Linting (ament, clang-format, flake8)
- Docker image build

### Architecture

```
Controllers (LQR/PID) → Safety Gate → Mode Switch:
  ├─► [mode=sim] → /cmd/final/body_rate_thrust → Simulator → RViz
  └─► [mode=crsf] → /cmd/final/rc → CRSF Adapter → UDP/Serial → TX_RX (ESP32) → Betaflight
```

### Key Features

- **Firmware-agnostic**: Controllers use FRD body rates
- **Safety-first**: Multi-layer safety checks, timeouts, slew limits
- **Pluggable CRSF**: Customize packet format without changing code
- **Production-ready**: Full CI/CD, tests, documentation
- **Simulation**: Test behavior before hardware exists
- **Frame conversion**: FRD controllers, ENU simulator with TF

### Frame Conventions

- **Controllers**: FRD (Front-Right-Down) body frame
- **Simulator**: ENU (East-North-Up) world frame
- **Conversions**: Handled automatically in simulator and safety gate

### TX_RX Integration

The `adapters_crsf` package sends DirectCommandPayload format (20 bytes) to your TX:
- Format: 4 floats (roll, pitch, yaw, throttle) + timestamp
- **TX firmware needs modification**: Add UDP/Serial input handler
- See `TX_RX_INTEGRATION.md` for complete guide with example code
- UDP (WiFi) or Serial (USB) transport
- Custom packers available if you need different formats

### Next Steps

1. **Build**: `./scripts/build.sh`
2. **Test Sim**: `./scripts/run_sim_pid.sh`
3. **Tune**: Adjust parameters in config files
4. **Hardware**: Connect TX_RX, run `./scripts/run_crsf_link_pid.sh`
5. **Customize**: Add custom packer for different TX_RX formats

### Safety Checklist

Before hardware flight:
- [ ] Props OFF for all tests
- [ ] Verify AUX channel arming
- [ ] Test failsafe (kill node → verify safe idle)
- [ ] Check CRSF link health
- [ ] Validate thrust range (hover_thrust)
- [ ] Manual RC override ready
- [ ] Hardware kill switch available

### Documentation

See `README.md` for:
- Detailed tuning guide (LQR, PID)
- Topic API reference
- Packer customization
- Troubleshooting
- Contributing guidelines

---

**Status**: All packages implemented and ready to build  
**ROS Version**: ROS 2 Humble  
**License**: MIT

