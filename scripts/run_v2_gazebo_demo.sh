#!/usr/bin/env bash
set -euo pipefail

echo "Terminal 1:"
echo "  ros2 launch sim_gazebo bringup.launch.py"
echo
echo "Terminal 2:"
echo "  ros2 launch ground_station ground.launch.py start_planner:=true start_monitor:=true"
echo
echo "Terminal 3:"
echo "  ros2 run ground_station ground_station_demo_mission"
echo
echo "Manual override example:"
echo "  ros2 run ground_station ground_station_cli -- --mode manual --manual-override --arm --vx 0.5 --duration-sec 3"
