#!/bin/bash
# Launch manual IMU control in simulation

set -e

echo "=== Manual IMU Control - Simulation Mode ==="

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

echo ""
echo "Starting components:"
echo "  - Simulator"
echo "  - IMU Controller"  
echo "  - PID Controller (attitude tracking)"
echo "  - Safety Gate"
echo "  - RViz"
echo ""
echo "NOTE: You need to run your IMU driver separately:"
echo "  ros2 run <your_imu_pkg> <imu_node>"
echo ""
echo "And publish throttle:"
echo "  ros2 topic pub /manual/throttle std_msgs/Float32 \"data: 0.5\" -r 10"
echo ""

# Launch
ros2 launch manual_imu_controller imu_manual_sim.launch.py

