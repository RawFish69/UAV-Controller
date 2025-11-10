#!/bin/bash
# Launch manual IMU control for hardware

set -e

echo "=== Manual IMU Control - Hardware Mode ==="

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

echo ""
echo "Starting components:"
echo "  - IMU Controller"
echo "  - PID Controller (attitude tracking)"
echo "  - Safety Gate (CRSF mode)"
echo "  - CRSF Adapter"
echo ""
echo "NOTE: You need to run your IMU driver separately:"
echo "  ros2 run <your_imu_pkg> <imu_node>"
echo ""
echo "Also run throttle source (joystick or manual):"
echo "  ros2 run joy joy_node  # If using joystick"
echo ""
echo "SAFETY: Props off for testing!"
echo ""

# Launch IMU controller + PID + safety gate
ros2 launch manual_imu_controller imu_manual_control.launch.py mode:=crsf &
CTRL_PID=$!

sleep 2

# Launch CRSF adapter
ros2 launch adapters_crsf crsf_adapter.launch.py $ARGS &
CRSF_PID=$!

# Trap Ctrl+C
trap "kill $CTRL_PID $CRSF_PID 2>/dev/null; exit" INT TERM

echo "=== System running. Press Ctrl+C to stop. ==="
wait

