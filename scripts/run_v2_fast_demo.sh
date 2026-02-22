#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 0 ]]; then
  MODE="$1"
else
  MODE="offboard"
fi

if [[ "$MODE" == "onboard" ]]; then
  ros2 launch sim_fast bringup.launch.py \
    start_offboard_planner:=false \
    start_onboard_planner:=true \
    start_demo:=true \
    demo_planning_mode:=onboard
else
  ros2 launch sim_fast bringup.launch.py \
    start_offboard_planner:=true \
    start_onboard_planner:=false \
    start_demo:=true \
    demo_planning_mode:=offboard
fi
