#!/bin/bash
# Launch simulation with LQR controller
# Usage: ./run_sim_lqr.sh [headless]
#   headless: Set to 'true' or '1' to run without RViz (headless mode)

set -e

# Check for headless argument
HEADLESS="${1:-false}"
if [ "$HEADLESS" = "true" ] || [ "$HEADLESS" = "1" ] || [ "$HEADLESS" = "headless" ]; then
    USE_RVIZ="false"
    echo "=== Running Simulation with LQR Controller (HEADLESS MODE) ==="
else
    USE_RVIZ="true"
    echo "=== Running Simulation with LQR Controller (WITH VISUALIZATION) ==="
fi

# Navigate to workspace
cd "$(dirname "$0")/../ros2_ws"

# Source ROS 2
for distro in jazzy humble foxy; do if [ -f "/opt/ros/$distro/setup.bash" ]; then source "/opt/ros/$distro/setup.bash"; break; fi; done

# Build if needed
if [ ! -d "install" ]; then
    echo "Workspace not built. Building..."
    colcon build --symlink-install
fi

# Source workspace
source install/setup.bash

# Launch
echo "Launching simulator with LQR controller (use_rviz=$USE_RVIZ)..."
ros2 launch sim_dyn sim_lqr.launch.py use_rviz:=$USE_RVIZ

