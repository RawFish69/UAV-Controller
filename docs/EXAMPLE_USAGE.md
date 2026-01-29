# Example: Using Controllers with Randomly Generated Terrain

This guide demonstrates a complete workflow for running a UAV controller simulation in a randomly generated terrain environment.

---

## Overview

This example shows how to:
1. Generate a random terrain (Forest, Mountains, or Plains)
2. Launch a controller simulation (PID, LQR, or MPC)
3. Visualize the UAV navigating through the terrain
4. Monitor and analyze the flight

## Setup and Build

**Note:** You do **not** need a Python virtual environment for the ROS 2 workspace.

Before running the examples, ensure you have built the workspace:

```bash
cd ros2_ws
# Ensure ROS is sourced (if not in your .bashrc)
# source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Alternatively, the helper scripts in the `scripts/` directory will handle sourcing and building automatically.

---

## Quick Example: Forest Terrain with MPC Controller

### Step 1: Generate Forest Terrain

Open a terminal and launch the terrain generator:

```bash
# Navigate to the ros2_ws directory in this repo
cd ros2_ws
source install/setup.bash

# Generate forest terrain
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest
```

**What happens:**
- Terrain generator node starts
- Random forest is generated based on config parameters
- Obstacles are published as RViz markers on `/terrain/obstacles` topic
- You'll see output like: `Generated forest with 70 trees`

**Configuration (optional):**
Edit `ros2_ws/src/terrain_generator/config/terrain_params.yaml` to customize:
```yaml
terrain_generator:
  ros__parameters:
    terrain_type: 'forest'
    space_dim: [100.0, 100.0, 50.0]  # 100m x 100m x 50m space
    forest:
      grid_size: 10
      radius_range: [0.5, 1.5]  # Tree radius 0.5-1.5m
      height_range: [5.0, 15.0]  # Tree height 5-15m
      density: 0.7  # 70% of grid cells have trees
```

### Step 2: Launch Controller Simulation

In a **new terminal**, launch the MPC controller simulation:

```bash
cd ros2_ws
source install/setup.bash

# Launch MPC controller with visualization
ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=true
```

**What happens:**
- Dynamics simulator starts (200 Hz)
- MPC controller starts (100 Hz)
- Safety gate starts
- RViz opens showing:
  - UAV position and orientation
  - Terrain obstacles (green cylinders for trees)
  - Trajectory visualization

### Step 3: Visualize in RViz

RViz should automatically open. If not, you can add displays manually:

1. **Add UAV Model:**
   - Click "Add" → "By display type" → "TF"
   - Enable "base_link" frame

2. **Add Terrain Obstacles:**
   - Click "Add" → "By topic" → `/terrain/obstacles`
   - Select "MarkerArray"

3. **Add Trajectory:**
   - Click "Add" → "By topic" → `/state/odom`
   - Select "Odometry"

4. **Configure View:**
   - Set Fixed Frame to "map"
   - Use mouse to navigate: drag to rotate, scroll to zoom

### Step 4: Monitor Flight

In a **third terminal**, monitor the flight:

```bash
cd ros2_ws
source install/setup.bash

# Monitor position
ros2 topic echo /state/odom --no-arr

# Check controller output
ros2 topic echo /cmd/body_rate_thrust

# Monitor publishing rates
ros2 topic hz /state/odom
ros2 topic hz /cmd/body_rate_thrust
```

---

## Complete Workflow Examples

### Example 1: Mountain Terrain with LQR Controller

**Terminal 1 - Generate Mountains:**
```bash
cd ros2_ws
source install/setup.bash

# Generate mountain terrain
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=mountains
```

**Terminal 2 - Run LQR Controller:**
```bash
cd ros2_ws
source install/setup.bash

# Launch simulation
ros2 launch sim_dyn sim_lqr.launch.py use_rviz:=true
```

**Terminal 3 - Monitor:**
```bash
# Watch the UAV navigate around mountain peaks
ros2 topic echo /state/odom --no-arr
```

### Example 2: Plains Terrain with PID Controller (Headless)

**Terminal 1 - Generate Plains:**
```bash
cd ros2_ws
source install/setup.bash

ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=plains
```

**Terminal 2 - Run PID Headless:**
```bash
cd ros2_ws
source install/setup.bash

# Headless mode (no RViz)
ros2 launch sim_dyn sim_pid.launch.py use_rviz:=false
```

**Terminal 3 - Monitor Topics:**
```bash
# Monitor position
ros2 topic echo /state/odom --no-arr

# Record data for later analysis
ros2 bag record /state/odom /cmd/body_rate_thrust /terrain/obstacles
```

### Example 3: Custom Forest Density with MPC

**Step 1: Customize Terrain**

Edit `ros2_ws/src/terrain_generator/config/terrain_params.yaml`:
```yaml
terrain_generator:
  ros__parameters:
    terrain_type: 'forest'
    space_dim: [150.0, 150.0, 60.0]  # Larger space
    forest:
      grid_size: 15  # More grid cells
      radius_range: [0.8, 2.0]  # Larger trees
      height_range: [8.0, 20.0]  # Taller trees
      density: 0.5  # Sparse forest (50% density)
```

**Step 2: Rebuild and Launch**
```bash
cd ros2_ws
colcon build --packages-select terrain_generator
source install/setup.bash

# Generate custom terrain
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest

# In another terminal, launch controller
ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=true
```

---

## Advanced: Using Scripts

### Using Helper Scripts

The repository includes helper scripts for easier launching:

**Generate Terrain + Run Simulation:**
```bash
# Terminal 1: Terrain
cd ros2_ws
source install/setup.bash
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest

# Terminal 2: Simulation
# (From repository root)
./scripts/run_sim_mpc.sh  # With visualization
# or
./scripts/run_sim_mpc.sh headless  # Without visualization
```

---

## Visualizing Results

### Real-Time Visualization in RViz

**Setup RViz Display:**
1. **Fixed Frame:** Set to "map"
2. **Add Displays:**
   - **TF:** Shows coordinate frames
   - **MarkerArray** (`/terrain/obstacles`): Shows terrain
   - **Odometry** (`/state/odom`): Shows UAV position
   - **Marker** (`/visualization_marker`): Shows UAV body axes

**View Controls:**
- **Rotate:** Left-click + drag
- **Pan:** Middle-click + drag (or Shift + left-click)
- **Zoom:** Scroll wheel
- **Focus:** Right-click on object → "Focus Camera"

### Recording and Playback

**Record Simulation:**
```bash
ros2 bag record /state/odom /cmd/body_rate_thrust /terrain/obstacles -o terrain_flight
```

**Playback:**
```bash
ros2 bag play terrain_flight

# In another terminal, launch RViz
rviz2 -d install/sim_dyn/share/sim_dyn/rviz/overview.rviz
```

---

## Analyzing Performance

### Check Controller Performance

**Monitor Control Effort:**
```bash
# Watch control commands
ros2 topic echo /cmd/body_rate_thrust

# Check for oscillations
ros2 topic hz /cmd/body_rate_thrust  # Should be smooth
```

**Visualize in rqt:**
```bash
rqt_plot /state/odom/pose/pose/position/z  # Altitude over time
rqt_plot /cmd/body_rate_thrust/body_rates/x  # Roll rate command
```

### Check Obstacle Avoidance

**Verify UAV Position vs Obstacles:**
```bash
# Get current position
ros2 topic echo /state/odom --once

# Check terrain obstacles
ros2 topic echo /terrain/obstacles --once
```

**Calculate Distance to Nearest Obstacle:**
You can write a simple Python script to monitor distances, or use RViz to visually verify the UAV maintains safe distance from obstacles.

---

## Customizing the Example

### Different Terrain Types

**Mountains:**
```yaml
terrain_generator:
  ros__parameters:
    terrain_type: 'mountains'
    mountains:
      num_peaks: 20  # More peaks
      base_size_range: [4.0, 10.0]
      height_range: [15.0, 35.0]
```

**Plains:**
```yaml
terrain_generator:
  ros__parameters:
    terrain_type: 'plains'
    plains:
      num_obstacles: 15
      obstacle_types: ['bush', 'rock', 'tree']
```

### Different Controllers

**PID Controller:**
- Good for: Simple hover and basic maneuvers
- Tune: `ros2_ws/src/controllers_pid/config/pid_params.yaml`

**LQR Controller:**
- Good for: Optimal control, smooth trajectories
- Tune: `ros2_ws/src/controllers_lqr/config/lqr_params.yaml`

**MPC Controller:**
- Good for: Predictive control, obstacle avoidance (with path planning)
- Tune: `ros2_ws/src/controllers_mpc/config/mpc_params.yaml`

### Adjusting Simulation Parameters

Edit `ros2_ws/src/sim_dyn/config/sim_params.yaml`:
```yaml
sim_dyn:
  ros__parameters:
    sim_rate: 200.0  # Simulation frequency
    mass: 1.0  # UAV mass (kg)
    hover_thrust: 0.5  # Normalized hover thrust
    gravity: 9.81  # Gravity (m/s²)
```

---

## Troubleshooting

### Terrain Not Showing in RViz

1. **Check topic is publishing:**
   ```bash
   ros2 topic hz /terrain/obstacles
   ```

2. **Add display manually:**
   - RViz → Add → By topic → `/terrain/obstacles` → MarkerArray

3. **Check frame:**
   - Set Fixed Frame to "map"

### Controller Not Responding

1. **Check nodes are running:**
   ```bash
   ros2 node list
   # Should see: /dynamics_node, /mpc_controller, /safety_gate
   ```

2. **Check topics:**
   ```bash
   ros2 topic list
   ros2 topic echo /cmd/body_rate_thrust
   ```

3. **Check for errors:**
   - Look at terminal output for error messages

### Terrain Generation Issues

1. **No obstacles generated:**
   - Check config file parameters
   - Verify terrain_generator node is running: `ros2 node list`
   - Check logs: `ros2 topic echo /rosout`

2. **Wrong terrain type:**
   - Verify launch argument: `terrain_type:=forest`
   - Check config file: `terrain_type: 'forest'`

---

## Next Steps

### Path Planning Integration

To add path planning around obstacles:
1. Use the terrain obstacles from `/terrain/obstacles`
2. Implement A* or RRT* path planner
3. Feed waypoints to controller
4. Controller follows path while avoiding obstacles

### Multiple UAVs

To simulate multiple UAVs:
1. Launch multiple controller nodes with different namespaces
2. Each UAV gets its own state topics
3. Share terrain obstacles between all UAVs

### Real Hardware Testing

Once simulation works:
1. Generate terrain and record waypoints
2. Transfer waypoints to hardware system
3. Use same controller on real UAV
4. Monitor via telemetry

---

## Summary

This example demonstrated:
- ✅ Generating random terrain (Forest/Mountains/Plains)
- ✅ Launching controller simulation
- ✅ Visualizing in RViz
- ✅ Monitoring flight data
- ✅ Customizing parameters

**Quick Command Summary:**
```bash
# Terminal 1: Terrain
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=forest

# Terminal 2: Controller
ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=true

# Terminal 3: Monitor
ros2 topic echo /state/odom
```

For more details, see the [Software Guide](SOFTWARE_GUIDE.md).
