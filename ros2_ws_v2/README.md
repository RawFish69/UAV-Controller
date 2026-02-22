# ROS2 Workspace V2 (Gazebo + Ground Station / Air Unit)

This workspace contains the rebuilt ROS2 + Gazebo simulation stack for UAV-Controller.

## Status

- V2 workspace scaffolded
- `drone_msgs` interfaces defined
- Additional packages are present and will be filled incrementally

## Build (Linux / ROS2 Humble)

```bash
cd ros2_ws_v2
colcon build --symlink-install
source install/setup.bash
```
