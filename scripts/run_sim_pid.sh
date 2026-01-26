#!/bin/bash
# Launch simulation with PID controller
# Usage: ./run_sim_pid.sh [headless]
#   headless: Set to 'true' or '1' to run without RViz (headless mode)

set -e

# Check for headless argument
HEADLESS="${1:-false}"
if [ "$HEADLESS" = "true" ] || [ "$HEADLESS" = "1" ] || [ "$HEADLESS" = "headless" ]; then
    USE_RVIZ="false"
    echo "=== Running Simulation with PID Controller (HEADLESS MODE) ==="
else
    USE_RVIZ="true"
    echo "=== Running Simulation with PID Controller (WITH VISUALIZATION) ==="
fi

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
echo "Launching simulator with PID controller (use_rviz=$USE_RVIZ)..."
ros2 launch sim_dyn sim_pid.launch.py use_rviz:=$USE_RVIZ

