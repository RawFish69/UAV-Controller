#!/bin/bash
# Debug script to check topic flow in simulation

echo "=== Checking ROS 2 Topic Flow ==="
echo ""
echo "1. Checking if nodes are running:"
ros2 node list
echo ""

echo "2. Checking controller output (/cmd/body_rate_thrust):"
timeout 2s ros2 topic echo /cmd/body_rate_thrust --once
echo ""

echo "3. Checking safety gate output (/cmd/final/body_rate_thrust):"
timeout 2s ros2 topic echo /cmd/final/body_rate_thrust --once
echo ""

echo "4. Checking simulator state (/state/odom):"
timeout 2s ros2 topic echo /state/odom --once
echo ""

echo "5. Topic connections:"
ros2 topic info /cmd/body_rate_thrust
ros2 topic info /cmd/final/body_rate_thrust
ros2 topic info /state/attitude
ros2 topic info /state/angular_velocity

