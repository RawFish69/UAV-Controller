import time
from typing import Optional

import rclpy
from drone_msgs.msg import Command, MissionStatus, Trajectory, Waypoint
from geometry_msgs.msg import Quaternion
from rclpy.node import Node


class GroundStationDemoMission(Node):
    """Basic demo mission: takeoff -> hover -> 3 waypoints -> land."""

    def __init__(self) -> None:
        super().__init__('ground_station_demo_mission')
        self.declare_parameter('command_topic', '/uav1/command')
        self.declare_parameter('mission_topic', '/uav1/mission')
        self.declare_parameter('mission_status_topic', '/uav1/mission_status')
        self.declare_parameter('loop_rate_hz', 5.0)
        self.declare_parameter('takeoff_wait_sec', 4.0)
        self.declare_parameter('hover_wait_sec', 2.0)
        self.declare_parameter('landing_wait_sec', 8.0)
        self.declare_parameter('mission_timeout_sec', 30.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.mission_topic = self.get_parameter('mission_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.loop_rate_hz = float(self.get_parameter('loop_rate_hz').value)
        self.takeoff_wait_sec = float(self.get_parameter('takeoff_wait_sec').value)
        self.hover_wait_sec = float(self.get_parameter('hover_wait_sec').value)
        self.landing_wait_sec = float(self.get_parameter('landing_wait_sec').value)
        self.mission_timeout_sec = float(self.get_parameter('mission_timeout_sec').value)

        self.pub_cmd = self.create_publisher(Command, self.command_topic, 10)
        self.pub_mission = self.create_publisher(Trajectory, self.mission_topic, 10)
        self.create_subscription(MissionStatus, self.mission_status_topic, self._on_mission_status, 20)
        self.last_mission_status: Optional[MissionStatus] = None

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self.last_mission_status = msg

    def _publish_cmd(self, mode: int, arm: bool = False, disarm: bool = False) -> None:
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence_id = 100
        msg.mode_request = mode
        msg.planning_mode = Command.PLANNING_OFFBOARD
        msg.arm = arm
        msg.disarm = disarm
        msg.manual_override = False
        msg.source_id = 'ground_station_demo_mission'
        self.pub_cmd.publish(msg)

    def _publish_demo_trajectory(self) -> None:
        traj = Trajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'map'
        traj.frame_id = 'map'
        traj.sequence_id = 42

        q = Quaternion()
        q.w = 1.0

        points = [
            (2.0, 0.0, 1.5),
            (2.0, 2.0, 1.5),
            (0.0, 2.0, 1.2),
        ]
        for i, (x, y, z) in enumerate(points):
            wp = Waypoint()
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z
            wp.pose.orientation = q
            wp.hold_time_sec = 1.0 if i < len(points) - 1 else 0.5
            wp.acceptance_radius_m = 0.5
            wp.desired_speed_mps = 1.0
            traj.waypoints.append(wp)

        self.pub_mission.publish(traj)
        self.get_logger().info(f'Published demo trajectory with {len(traj.waypoints)} waypoints')

    def _loop_for(self, duration_sec: float, mode: int, arm: bool = False, disarm: bool = False) -> None:
        end = time.time() + duration_sec
        dt = 1.0 / max(self.loop_rate_hz, 1.0)
        while rclpy.ok() and time.time() < end:
            self._publish_cmd(mode, arm=arm, disarm=disarm)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(dt)

    def run(self) -> None:
        self.get_logger().info('Demo: arm + takeoff')
        self._loop_for(self.takeoff_wait_sec, Command.MODE_TAKEOFF, arm=True)

        self.get_logger().info('Demo: hover')
        self._loop_for(self.hover_wait_sec, Command.MODE_HOVER, arm=True)

        self.get_logger().info('Demo: mission')
        self._publish_demo_trajectory()
        end = time.time() + self.mission_timeout_sec
        dt = 1.0 / max(self.loop_rate_hz, 1.0)
        while rclpy.ok() and time.time() < end:
            self._publish_cmd(Command.MODE_MISSION, arm=True)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_mission_status and self.last_mission_status.complete:
                break
            time.sleep(dt)

        self.get_logger().info('Demo: land')
        self._loop_for(self.landing_wait_sec, Command.MODE_LAND, arm=True)
        self.get_logger().info('Demo: disarm')
        self._loop_for(0.5, Command.MODE_IDLE, disarm=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationDemoMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
