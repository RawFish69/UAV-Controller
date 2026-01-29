#!/bin/bash
# Launch CRSF link with LQR controller for hardware

set -e

echo "=== Running CRSF Link with LQR Controller ==="

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

# Parse arguments (pass through to launch file)
ARGS="$@"

# Launch LQR controller
echo "Launching LQR controller..."
ros2 launch controllers_lqr lqr.launch.py &
LQR_PID=$!

# Wait a bit for controller to start
sleep 2

# Launch safety gate in CRSF mode
echo "Launching safety gate (CRSF mode)..."
ros2 launch safety_gate safety_gate.launch.py mode:=crsf &
SAFETY_PID=$!

# Wait a bit for safety gate to start
sleep 1

# Launch CRSF adapter
echo "Launching CRSF adapter..."
ros2 launch adapters_crsf crsf_adapter.launch.py $ARGS &
CRSF_PID=$!

# Trap Ctrl+C to kill all nodes
trap "kill $LQR_PID $SAFETY_PID $CRSF_PID 2>/dev/null; exit" INT TERM

echo "=== System running. Press Ctrl+C to stop. ==="
wait

