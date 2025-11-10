#!/bin/bash
# Quick diagnostic for simulator issues

echo "=== Simulator Diagnostics ==="
echo ""

echo "1. Checking if nodes are running:"
ros2 node list
echo ""

echo "2. Checking simulator parameters:"
echo "c1 (thrust coefficient):"
ros2 param get /dynamics_node c1 2>/dev/null || echo "  Node not running or param not found"
echo "c0:"
ros2 param get /dynamics_node c0 2>/dev/null || echo "  Node not running"
echo "hover_thrust:"
ros2 param get /dynamics_node hover_thrust 2>/dev/null || echo "  Node not running"
echo ""

echo "3. Checking controller parameters:"
echo "hover_thrust (controller):"
ros2 param get /pid_controller hover_thrust 2>/dev/null || ros2 param get /lqr_controller hover_thrust 2>/dev/null || echo "  Controller not running"
echo ""

echo "4. Checking if controller is publishing:"
timeout 2s ros2 topic echo /cmd/body_rate_thrust --once 2>/dev/null
if [ $? -eq 124 ]; then
  echo "  ❌ Controller NOT publishing to /cmd/body_rate_thrust"
else
  echo "  ✓ Controller is publishing"
fi
echo ""

echo "5. Checking if safety gate is publishing:"
timeout 2s ros2 topic echo /cmd/final/body_rate_thrust --once 2>/dev/null
if [ $? -eq 124 ]; then
  echo "  ❌ Safety gate NOT publishing to /cmd/final/body_rate_thrust"
else
  echo "  ✓ Safety gate is publishing"
fi
echo ""

echo "6. Checking simulator state:"
echo "Current altitude (z position):"
timeout 2s ros2 topic echo /state/odom --once 2>/dev/null | grep -A 2 "position:" | grep "z:" || echo "  Could not read odom"
echo ""

echo "7. Actual thrust being commanded:"
timeout 2s ros2 topic echo /cmd/final/body_rate_thrust --once 2>/dev/null | grep "thrust:" || echo "  Could not read thrust"
echo ""

echo "=== Quick Fix ==="
echo "If c1 is not 2.0, run:"
echo "  cd ros2_ws"
echo "  colcon build --packages-select sim_dyn"
echo "  source install/setup.bash"
echo "  ros2 launch sim_dyn sim_pid.launch.py"

