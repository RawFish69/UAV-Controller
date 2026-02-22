import time
from typing import Optional

import rclpy
from drone_msgs.msg import Command, MissionStatus, Telemetry, Trajectory, Waypoint
from drone_msgs.srv import PlanPath
from geometry_msgs.msg import Quaternion
from rclpy.node import Node


PLANNING_MODE_MAP = {
    'offboard': Command.PLANNING_OFFBOARD,
    'onboard': Command.PLANNING_ONBOARD,
}


class GroundStationDemoMission(Node):
    """Basic demo mission: takeoff -> hover -> path -> land."""

    def __init__(self) -> None:
        super().__init__('ground_station_demo_mission')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('telemetry_topic', '/uav/telemetry')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('planner_service_topic', '/gs/planner/plan_path')
        self.declare_parameter('planning_mode', 'offboard')  # offboard | onboard
        self.declare_parameter('use_planner', True)
        self.declare_parameter('planner_type', 'rrtstar')
        self.declare_parameter('terrain_profile', 'plains')
        self.declare_parameter('loop_rate_hz', 5.0)
        self.declare_parameter('takeoff_wait_sec', 4.0)
        self.declare_parameter('hover_wait_sec', 2.0)
        self.declare_parameter('landing_wait_sec', 8.0)
        self.declare_parameter('mission_timeout_sec', 30.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.mission_topic = self.get_parameter('mission_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.planner_service_topic = self.get_parameter('planner_service_topic').value
        self.planning_mode_name = str(self.get_parameter('planning_mode').value).lower()
        self.use_planner = bool(self.get_parameter('use_planner').value)
        self.planner_type = self.get_parameter('planner_type').value
        self.terrain_profile = self.get_parameter('terrain_profile').value
        self.loop_rate_hz = float(self.get_parameter('loop_rate_hz').value)
        self.takeoff_wait_sec = float(self.get_parameter('takeoff_wait_sec').value)
        self.hover_wait_sec = float(self.get_parameter('hover_wait_sec').value)
        self.landing_wait_sec = float(self.get_parameter('landing_wait_sec').value)
        self.mission_timeout_sec = float(self.get_parameter('mission_timeout_sec').value)
        self.planning_mode = PLANNING_MODE_MAP.get(self.planning_mode_name, Command.PLANNING_OFFBOARD)

        self.pub_cmd = self.create_publisher(Command, self.command_topic, 10)
        self.pub_mission = self.create_publisher(Trajectory, self.mission_topic, 10)
        self.create_subscription(MissionStatus, self.mission_status_topic, self._on_mission_status, 20)
        self.create_subscription(Telemetry, self.telemetry_topic, self._on_telemetry, 20)
        self.last_mission_status: Optional[MissionStatus] = None
        self.last_telemetry: Optional[Telemetry] = None

        self.planner_client = self.create_client(PlanPath, self.planner_service_topic)
        self.sequence_id = 100

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self.last_mission_status = msg

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.last_telemetry = msg

    def _publish_cmd(self, mode: int, arm: bool = False, disarm: bool = False) -> None:
        self.sequence_id += 1
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence_id = self.sequence_id
        msg.mode_request = mode
        msg.planning_mode = self.planning_mode
        msg.arm = arm
        msg.disarm = disarm
        msg.manual_override = False
        msg.source_id = 'ground_station_demo_mission'
        self.pub_cmd.publish(msg)

    def _build_handcrafted_trajectory(self) -> Trajectory:
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
        return traj

    def _wait_for_planner(self, timeout_sec: float = 5.0) -> bool:
        end = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end:
            if self.planner_client.wait_for_service(timeout_sec=0.2):
                return True
        return False

    def _plan_offboard_trajectory(self, goal_wp: Waypoint) -> Optional[Trajectory]:
        if self.last_telemetry is None:
            self.get_logger().warn('No telemetry yet; cannot call offboard planner. Falling back to handcrafted mission.')
            return None
        if not self._wait_for_planner():
            self.get_logger().warn('Planner service unavailable; falling back to handcrafted mission.')
            return None

        req = PlanPath.Request()
        req.start = self.last_telemetry.pose
        req.goal = goal_wp.pose
        req.planner_type = str(self.planner_type)
        req.terrain_profile = str(self.terrain_profile)
        req.collision_inflation_m = 0.5
        future = self.planner_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn('Planner future timed out/failed; falling back to handcrafted mission.')
            return None
        result = future.result()
        if not result.success:
            self.get_logger().warn(f'Planner returned failure: {result.message}')
            return None
        traj = result.trajectory
        if traj.sequence_id == 0:
            traj.sequence_id = 42
        self.get_logger().info(f'Offboard planner generated {len(traj.waypoints)} waypoints')
        return traj

    def _publish_demo_trajectory(self) -> None:
        base_traj = self._build_handcrafted_trajectory()

        if self.planning_mode == Command.PLANNING_OFFBOARD and self.use_planner:
            planned = self._plan_offboard_trajectory(base_traj.waypoints[-1])
            traj = planned if planned is not None else base_traj
        elif self.planning_mode == Command.PLANNING_ONBOARD:
            # Goal-only trajectory convention: air-side mission executor calls local planner.
            traj = Trajectory()
            traj.header = base_traj.header
            traj.frame_id = base_traj.frame_id
            traj.sequence_id = base_traj.sequence_id
            traj.waypoints.append(base_traj.waypoints[-1])
        else:
            traj = base_traj

        self.pub_mission.publish(traj)
        self.get_logger().info(
            f"Published mission ({len(traj.waypoints)} waypoints, planning_mode={self.planning_mode_name})"
        )

    def _loop_for(self, duration_sec: float, mode: int, arm: bool = False, disarm: bool = False) -> None:
        end = time.time() + duration_sec
        dt = 1.0 / max(self.loop_rate_hz, 1.0)
        while rclpy.ok() and time.time() < end:
            self._publish_cmd(mode, arm=arm, disarm=disarm)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(dt)

    def run(self) -> None:
        self.get_logger().info(f'Demo: arm + takeoff (planning_mode={self.planning_mode_name})')
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
