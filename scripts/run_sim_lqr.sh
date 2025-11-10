#!/bin/bash
# Launch simulation with LQR controller

set -e

echo "=== Running Simulation with LQR Controller ==="

# Navigate to workspace
cd "$(dirname "$0")/../ros2_ws"

# Source ROS 2
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Build if needed
if [ ! -d "install" ]; then
    echo "Workspace not built. Building..."
    colcon build --symlink-install
fi

# Source workspace
source install/setup.bash

# Launch
echo "Launching simulator with LQR controller..."
ros2 launch sim_dyn sim_lqr.launch.py

