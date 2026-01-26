#!/bin/bash
# Launch terrain generator and simulation together
# Usage: ./run_terrain_sim.sh [controller] [terrain_type] [headless]
#   controller: pid, lqr, or mpc (default: pid)
#   terrain_type: forest, mountains, or plains (default: forest)
#   headless: true/false (default: false)

set -e

CONTROLLER="${1:-pid}"
TERRAIN_TYPE="${2:-forest}"
HEADLESS="${3:-false}"

if [ "$HEADLESS" = "true" ] || [ "$HEADLESS" = "1" ]; then
    USE_RVIZ="false"
    echo "=== Running Terrain Simulation (HEADLESS) ==="
else
    USE_RVIZ="true"
    echo "=== Running Terrain Simulation (WITH VISUALIZATION) ==="
fi

echo "Controller: $CONTROLLER"
echo "Terrain: $TERRAIN_TYPE"
echo "RViz: $USE_RVIZ"

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

# Launch terrain generator in background
echo "Starting terrain generator..."
ros2 launch terrain_generator terrain_generator.launch.py terrain_type:=$TERRAIN_TYPE &
TERRAIN_PID=$!

# Wait a moment for terrain to generate
sleep 2

# Launch simulation
echo "Starting simulation with $CONTROLLER controller..."
case $CONTROLLER in
    pid)
        ros2 launch sim_dyn sim_pid.launch.py use_rviz:=$USE_RVIZ
        ;;
    lqr)
        ros2 launch sim_dyn sim_lqr.launch.py use_rviz:=$USE_RVIZ
        ;;
    mpc)
        ros2 launch sim_dyn sim_mpc.launch.py use_rviz:=$USE_RVIZ
        ;;
    *)
        echo "Unknown controller: $CONTROLLER"
        echo "Use: pid, lqr, or mpc"
        kill $TERRAIN_PID 2>/dev/null
        exit 1
        ;;
esac

# Cleanup
kill $TERRAIN_PID 2>/dev/null
