# Quick Start Guide

## What You Need

- Ubuntu 22.04 (or just use Docker/DevContainer)
- ROS 2 Humble
- Git

## 1. Build It

```bash
cd UAV-Controller
./scripts/build.sh
```

## 2. Try It Out in Sim

### Option A: PID Controller (start here)

```bash
./scripts/run_sim_pid.sh
```

This fires up:
- The dynamics simulator (runs at 200 Hz)
- PID controller (cascaded attitude + rate control)
- Safety gate in sim mode
- RViz for visualization

You should see a quad hovering at 1m altitude in RViz.

### Option B: LQR Controller

```bash
./scripts/run_sim_lqr.sh
```

## 3. Connect to Real Hardware

### Modify Your TX Firmware First

Your ESP32 TX needs UDP or Serial input added. See `TX_RX_INTEGRATION.md` for the full guide with example code.

Quick summary: Add a UDP server on port 9000 that receives 20-byte packets and calls `CustomProtocol_SendDirectCommand()`.

### Once TX is Ready

Your ESP32 TX should be:
- Running in AP mode or connected to WiFi (IP: 192.168.4.1)
- Listening on UDP port 9000
- OR connected via USB (e.g., /dev/ttyUSB0)

### Launch CRSF Link

**With UDP (WiFi):**
```bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1 udp_port:=9000
```

**With Serial (USB):**
```bash
./scripts/run_crsf_link_pid.sh transport:=serial serial_port:=/dev/ttyUSB0 baud:=921600
```

### Monitor System

In separate terminals:

```bash
# Check published topics
ros2 topic list

# Monitor RC commands
ros2 topic echo /cmd/final/rc

# Monitor controller output
ros2 topic echo /cmd/body_rate_thrust

# Check node health
ros2 node list
```

## 4. Safety Checks

Before you even think about flying:

1. **Props OFF** - Seriously, take the props off for all bench tests
2. **Check failsafe**: 
   ```bash
   # In one terminal, run system
   ./scripts/run_crsf_link_pid.sh
   
   # Kill controller (Ctrl+C)
   # Verify CRSF adapter sends safe idle (throttle=0)
   ```
3. **Verify CRSF link**: Check TX_RX serial output for received packets
4. **Test arming**: Verify AUX channels match Betaflight config

## 5. Tuning

### PID Tuning

Just edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`:

```yaml
# Start conservative, increase gradually
kp:
  x: 8.0   # Roll rate P
  y: 8.0   # Pitch rate P
  z: 3.0   # Yaw rate P

ki:
  x: 0.6   # Roll rate I
  y: 0.6   # Pitch rate I
  z: 0.3   # Yaw rate I
```

Rebuild after changes:
```bash
./scripts/build.sh
```

### LQR Tuning

Edit `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`:

```yaml
# Gain matrix K (4x6)
# Compute offline with MATLAB/Python LQR solver
K: [
  6.0, 0.0, 0.0, 1.0, 0.0, 0.0,  # p_cmd from state
  0.0, 6.0, 0.0, 0.0, 1.0, 0.0,  # q_cmd
  0.0, 0.0, 3.0, 0.0, 0.0, 0.8,  # r_cmd
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0   # thrust_cmd
]
```

### Safety Gate Tuning

Edit `ros2_ws/src/safety_gate/config/safety_params.yaml`:

```yaml
max_tilt_deg: 35.0      # Max allowed tilt angle
max_body_rate: 10.0     # Max body rate (rad/s)
thrust_min: 0.05
thrust_max: 0.9
timeout_ms: 200         # Command timeout
```

## 6. Custom TX_RX Packet Format

If your TX_RX uses something different:

1. Create custom packer:

```python
# ros2_ws/src/adapters_crsf/adapters_crsf/packers/my_packer.py
from .base import PacketPacker
import struct

class MyPacker(PacketPacker):
    def encode(self, rc_msg):
        # Your custom format
        return struct.pack('<HHH', 
            int((rc_msg.roll + 1) * 1000),
            int((rc_msg.pitch + 1) * 1000),
            int(rc_msg.throttle * 1000))
    
    def safe_idle(self):
        return struct.pack('<HHH', 1000, 1000, 0)
```

2. Use in launch:

```bash
./scripts/run_crsf_link_pid.sh packer:=my_packer
```

## 7. Dev Environment Setup

### VS Code

1. Grab the Remote-Containers extension
2. Open the folder in VS Code
3. Hit "Reopen in Container"
4. Once inside:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

### Docker

```bash
docker build -t uav-controller -f docker/Dockerfile.humble .
docker run -it --rm --network=host uav-controller
```

## 8. Run Tests

```bash
cd ros2_ws
source install/setup.bash

# C++ tests
colcon test --packages-select controllers_lqr controllers_pid
colcon test-result --verbose

# Python import checks
python3 -c "import adapters_crsf.packers"
python3 -c "import sim_dyn.dynamics_node"
```

## 9. Troubleshooting

### Issue: Simulator diverges

**Solution**: Check `hover_thrust` parameter in `sim_params.yaml`. Default 0.5 should work.

### Issue: CRSF adapter not connecting

**Solution**: 
- Verify TX_RX is running and reachable: `ping 192.168.4.1`
- Check TX_RX serial monitor for received packets
- Try increasing `failsafe_ms` in `crsf_params.yaml`

### Issue: PID has steady-state error

**Solution**: Increase `ki` slightly or check `i_clamp` is not too restrictive.

### Issue: Safety gate always triggers timeout

**Solution**: Controller not publishing fast enough. Increase `timeout_ms` or check controller is running.

## 10. What's Next

- **Tune in sim**: Make sure you can get a stable hover with the default params
- **Adjust gains**: Check out the tuning guide in README.md
- **Bench test**: Verify everything works with props off
- **Actually fly**: Only when you're ready, have a safety pilot, and know what you're doing

## Full Documentation

See `README.md` for:
- Architecture details
- Frame conventions (FRD vs ENU)
- Complete API reference
- Advanced tuning guide
- Contributing guidelines

## Support

- Check `README.md` troubleshooting section
- Review test files for usage examples
- Inspect launch files for parameter examples

---

**Seriously though**: Always test with props OFF first. Don't be that person.

