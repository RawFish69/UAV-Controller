# Setup Guide

## Quick Start - Simulation

### Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Autonomous Control

```bash
# PID controller
ros2 launch sim_dyn sim_pid.launch.py

# Or LQR controller
ros2 launch sim_dyn sim_lqr.launch.py
```

Should see quad hovering at 1m in RViz.

### Run Manual IMU Control

```bash
# Launch stack
ros2 launch manual_imu_controller imu_manual_sim.launch.py

# In another terminal, send fake IMU data
ros2 topic pub /imu/data sensor_msgs/Imu \
  '{orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}' -r 50

# And throttle
ros2 topic pub /manual/throttle std_msgs/Float32 "data: 0.5" -r 10
```

## Debugging

### Check What's Running

```bash
ros2 node list  # See all nodes
ros2 topic list # See all topics
ros2 topic hz /cmd/body_rate_thrust  # Check rate
```

### Common Issues

**Quad falling in sim**:
```bash
# Check thrust model
ros2 param get /dynamics_node c1  # Should be 2.0

# Rebuild if needed
cd ros2_ws
colcon build --packages-select sim_dyn
source install/setup.bash
```

**Nodes not starting**:
```bash
source ros2_ws/install/setup.bash  # Make sure you source!
```

**Oscillations**:
Edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`:
- Reduce `kp` gains
- Increase `kd` gains
Rebuild: `colcon build --packages-select controllers_pid`

## Tuning

### PID Tuning

1. Edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`
2. Change gains (start with kp)
3. Rebuild: `colcon build --packages-select controllers_pid`
4. Relaunch and test

### IMU Gesture Sensitivity

Edit `ros2_ws/src/manual_imu_controller/config/imu_controller_params.yaml`:

```yaml
max_tilt_deg: 25.0      # Reduce for less aggressive
deadzone_deg: 8.0       # Increase for steadier
roll_scale: 0.8         # Reduce for less sensitive
pitch_scale: 0.8
```

Rebuild: `colcon build --packages-select manual_imu_controller`

## Docker Usage

### Build Image

```bash
docker build -t uav-controller -f docker/Dockerfile.humble .
```

### Run

```bash
docker run -it --rm --network=host uav-controller
```

### DevContainer (VS Code)

1. Install Remote-Containers extension
2. Open folder
3. Click "Reopen in Container"
4. Inside: `cd ros2_ws && colcon build`

## Testing

```bash
cd ros2_ws

# Run all tests
colcon test
colcon test-result --verbose

# Specific package
colcon test --packages-select controllers_pid
```

## Monitoring Tools

```bash
# Topic rates
ros2 topic hz /cmd/body_rate_thrust

# Plot data
ros2 run plotjuggler plotjuggler

# Node graph
ros2 run rqt_graph rqt_graph

# Parameters
ros2 param list /pid_controller
ros2 param get /dynamics_node c1
```

---

For hardware setup, see `docs/HARDWARE.md`

