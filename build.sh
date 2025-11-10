#!/bin/bash
set -e

cd /workspace/ros2_ws

# Clean the workspace
rm -rf build/ install/ log/

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Python packages in development mode
cd src/sim_dyn && pip3 install -e . && cd ../..
cd src/adapters_crsf && pip3 install -e . && cd ../..

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Verify installation
echo "Checking for dynamics_node executable:"
which dynamics_node || echo "dynamics_node not found"

echo "Checking package installation:"
ros2 pkg list | grep sim_dyn

echo "Checking lib directory:"
ls -l install/sim_dyn/lib/