import math
from typing import Optional

import rclpy
from drone_msgs.msg import Command, MissionStatus, Telemetry, Trajectory
from drone_msgs.srv import PlanPath
from geometry_msgs.msg import Twist
from rclpy.node import Node


STATUS_IDLE = 0
STATUS_ACTIVE = 1
STATUS_PAUSED = 2
STATUS_COMPLETE = 3


class MissionExecutorNode(Node):
    """Follow trajectories and support onboard planning via a local planner service."""

    def __init__(self) -> None:
        super().__init__('mission_executor_node')
        self.declare_parameter('mission_topic', '/uav1/mission')
        self.declare_parameter('command_topic', '/uav1/command')
        self.declare_parameter('telemetry_raw_topic', '/uav1/backend/telemetry_raw')
        self.declare_parameter('mission_cmd_topic', '/uav1/internal/mission_cmd_vel')
        self.declare_parameter('mission_status_topic', '/uav1/mission_status')
        self.declare_parameter('planner_service_topic', '/uav1/planner/plan_path')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('position_kp', 0.8)
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('altitude_gain', 0.9)
        self.declare_parameter('default_planner_type', 'rrtstar')
        self.declare_parameter('default_terrain_profile', 'plains')

        self.mission_topic = self.get_parameter('mission_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.telemetry_raw_topic = self.get_parameter('telemetry_raw_topic').value
        self.mission_cmd_topic = self.get_parameter('mission_cmd_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.planner_service_topic = self.get_parameter('planner_service_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.position_kp = float(self.get_parameter('position_kp').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.altitude_gain = float(self.get_parameter('altitude_gain').value)
        self.default_planner_type = self.get_parameter('default_planner_type').value
        self.default_terrain_profile = self.get_parameter('default_terrain_profile').value

        self.pub_cmd = self.create_publisher(Twist, self.mission_cmd_topic, 10)
        self.pub_status = self.create_publisher(MissionStatus, self.mission_status_topic, 10)
        self.planner_client = self.create_client(PlanPath, self.planner_service_topic)

        self.create_subscription(Trajectory, self.mission_topic, self._on_mission, 10)
        self.create_subscription(Telemetry, self.telemetry_raw_topic, self._on_telemetry, 20)
        self.create_subscription(Command, self.command_topic, self._on_command, 20)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.current_traj: Optional[Trajectory] = None
        self.pending_onboard_request: Optional[Trajectory] = None
        self.pending_planner_future = None
        self.active_idx = 0
        self.current_pos = None
        self.current_mode = Command.MODE_IDLE
        self.planning_mode = Command.PLANNING_OFFBOARD
        self.manual_override = False
        self.hold_started = None
        self.mission_complete = False
        self.active_mission_sequence_id = 0

    def _on_mission(self, msg: Trajectory) -> None:
        self.active_mission_sequence_id = int(msg.sequence_id)
        self.active_idx = 0
        self.hold_started = None
        self.mission_complete = False

        if self.planning_mode == Command.PLANNING_ONBOARD:
            self.pending_onboard_request = msg
            self.pending_planner_future = None
            self.current_traj = None
            self.get_logger().info('Onboard planning requested from goal trajectory')
        else:
            self.current_traj = msg
            self.pending_onboard_request = None
            self.get_logger().info(f'Offboard mission received: {len(msg.waypoints)} waypoints')

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.current_pos = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _on_command(self, msg: Command) -> None:
        self.current_mode = int(msg.mode_request)
        self.planning_mode = int(msg.planning_mode)
        self.manual_override = bool(msg.manual_override)

    def _publish_zero(self) -> None:
        self.pub_cmd.publish(Twist())

    def _publish_status(self, state: int, complete: bool, text: str) -> None:
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.current_traj is not None:
            msg.mission_sequence_id = int(self.current_traj.sequence_id)
            msg.total_waypoints = len(self.current_traj.waypoints)
        else:
            msg.mission_sequence_id = int(self.active_mission_sequence_id)
        msg.active_waypoint_index = int(self.active_idx)
        msg.state = int(state)
        msg.complete = bool(complete)
        msg.status_text = text
        self.pub_status.publish(msg)

    def _tick_onboard_planning(self) -> None:
        if self.planning_mode != Command.PLANNING_ONBOARD:
            return
        if self.pending_onboard_request is None:
            return

        if self.pending_planner_future is not None:
            if not self.pending_planner_future.done():
                return
            try:
                result = self.pending_planner_future.result()
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f'Onboard planner call failed: {exc}')
                self.pending_planner_future = None
                return
            if result is None or not result.success:
                err = 'no response' if result is None else result.message
                self.get_logger().error(f'Onboard planner failed: {err}')
                self.pending_planner_future = None
                return

            self.current_traj = result.trajectory
            if self.current_traj.sequence_id == 0:
                self.current_traj.sequence_id = self.active_mission_sequence_id
            self.pending_onboard_request = None
            self.pending_planner_future = None
            self.active_idx = 0
            self.hold_started = None
            self.mission_complete = False
            self.get_logger().info(
                f'Onboard planner produced {len(self.current_traj.waypoints)} waypoints'
            )
            return

        if self.current_pos is None:
            return
        if not self.pending_onboard_request.waypoints:
            return
        if not self.planner_client.wait_for_service(timeout_sec=0.0):
            return

        goal_wp = self.pending_onboard_request.waypoints[-1]
        req = PlanPath.Request()
        req.start.position.x = float(self.current_pos[0])
        req.start.position.y = float(self.current_pos[1])
        req.start.position.z = float(self.current_pos[2])
        req.start.orientation.w = 1.0
        req.goal = goal_wp.pose
        req.planner_type = self.default_planner_type
        req.terrain_profile = self.default_terrain_profile
        req.collision_inflation_m = 0.5
        self.pending_planner_future = self.planner_client.call_async(req)
        self._publish_status(STATUS_IDLE, False, 'onboard planner requested')

    def _tick(self) -> None:
        self._tick_onboard_planning()

        if self.current_traj is None or not self.current_traj.waypoints:
            self._publish_zero()
            if self.pending_onboard_request is not None:
                self._publish_status(STATUS_IDLE, False, 'waiting for onboard planner result')
            else:
                self._publish_status(STATUS_IDLE, False, 'no mission')
            return
        if self.current_pos is None:
            self._publish_zero()
            self._publish_status(STATUS_IDLE, False, 'waiting for telemetry')
            return
        if self.manual_override:
            self._publish_zero()
            self._publish_status(STATUS_PAUSED, False, 'paused: manual override')
            return
        if self.current_mode != Command.MODE_MISSION:
            self._publish_zero()
            self._publish_status(STATUS_PAUSED, self.mission_complete, 'paused: mode != mission')
            return
        if self.mission_complete:
            self._publish_zero()
            self._publish_status(STATUS_COMPLETE, True, 'mission complete')
            return
        if self.active_idx >= len(self.current_traj.waypoints):
            self.mission_complete = True
            self._publish_zero()
            self._publish_status(STATUS_COMPLETE, True, 'mission complete')
            return

        wp = self.current_traj.waypoints[self.active_idx]
        target = (
            float(wp.pose.position.x),
            float(wp.pose.position.y),
            float(wp.pose.position.z),
        )
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        dz = target[2] - self.current_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        acceptance = max(0.1, float(wp.acceptance_radius_m))

        if dist <= acceptance:
            hold_time = max(0.0, float(wp.hold_time_sec))
            now = self.get_clock().now()
            if hold_time > 0.0:
                if self.hold_started is None:
                    self.hold_started = now
                held = (now - self.hold_started).nanoseconds / 1e9
                if held < hold_time:
                    self._publish_zero()
                    self._publish_status(
                        STATUS_ACTIVE,
                        False,
                        f'holding wp {self.active_idx} ({held:.1f}/{hold_time:.1f}s)',
                    )
                    return
            self.hold_started = None
            self.active_idx += 1
            if self.active_idx >= len(self.current_traj.waypoints):
                self.mission_complete = True
                self._publish_zero()
                self._publish_status(STATUS_COMPLETE, True, 'mission complete')
                return
            wp = self.current_traj.waypoints[self.active_idx]
            target = (
                float(wp.pose.position.x),
                float(wp.pose.position.y),
                float(wp.pose.position.z),
            )
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dz = target[2] - self.current_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        cmd = Twist()
        cmd.linear.x = self.position_kp * dx
        cmd.linear.y = self.position_kp * dy
        cmd.linear.z = self.altitude_gain * dz

        desired_speed = float(wp.desired_speed_mps) if float(wp.desired_speed_mps) > 0.0 else self.max_speed_mps
        max_speed = max(0.1, min(desired_speed, self.max_speed_mps))
        norm = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2 + cmd.linear.z ** 2)
        if norm > max_speed:
            scale = max_speed / norm
            cmd.linear.x *= scale
            cmd.linear.y *= scale
            cmd.linear.z *= scale

        self.pub_cmd.publish(cmd)
        self._publish_status(
            STATUS_ACTIVE,
            False,
            f'following wp {self.active_idx + 1}/{len(self.current_traj.waypoints)} dist={dist:.2f}m',
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
