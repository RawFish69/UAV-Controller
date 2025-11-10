#!/bin/bash
# Launch CRSF link with PID controller for hardware

set -e

echo "=== Running CRSF Link with PID Controller ==="

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

# Parse arguments (pass through to launch file)
ARGS="$@"

# Launch PID controller
echo "Launching PID controller..."
ros2 launch controllers_pid pid.launch.py &
PID_PID=$!

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
trap "kill $PID_PID $SAFETY_PID $CRSF_PID 2>/dev/null; exit" INT TERM

echo "=== System running. Press Ctrl+C to stop. ==="
wait

