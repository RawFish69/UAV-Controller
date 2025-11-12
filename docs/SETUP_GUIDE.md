# Setup Guide

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

### PID Simulation

```bash
source ros2_ws/install/setup.bash
ros2 launch sim_dyn sim_pid.launch.py
```

Quad hovers at 1m in RViz.

### LQR Simulation

```bash
source ros2_ws/install/setup.bash
ros2 launch sim_dyn sim_lqr.launch.py
```

## Debug

### Nodes

```bash
ros2 node list

# Should see:
# /dynamics_node
# /pid_controller (or /lqr_controller)
# /safety_gate
# /rviz2
```

### Topics

```bash
# Controller output
ros2 topic echo /cmd/body_rate_thrust

# After safety gate
ros2 topic echo /cmd/final/body_rate_thrust

# Simulator state
ros2 topic echo /state/odom

# Check rates
ros2 topic hz /cmd/body_rate_thrust  # ~100 Hz
ros2 topic hz /state/odom            # ~200 Hz
```

```bash
# Run diagnostics
./scripts/check_sim.sh
```

## Issues

**Quad falling**:
```bash
# Check thrust model
ros2 param get /dynamics_node c1  # Should be 2.0

# If wrong, rebuild
cd ros2_ws
colcon build --packages-select sim_dyn
source install/setup.bash
```

**Nodes not starting**:
```bash
# Always source workspace!
source ros2_ws/install/setup.bash

# Check for build errors
colcon build --event-handlers console_direct+
```

**Oscillations**:
- Edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`
- Reduce `kp` gains or increase `kd`
- Rebuild: `colcon build --packages-select controllers_pid`

## Tuning

### PID

Edit `ros2_ws/src/controllers_pid/config/pid_params.yaml`, change gains, rebuild.

### LQR

Edit `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`, adjust K matrix, rebuild.

### Safety

Edit `ros2_ws/src/safety_gate/config/safety_params.yaml` for limits.

## Testing

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```
