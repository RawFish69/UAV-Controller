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
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('telemetry_raw_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('mission_cmd_topic', '/uav/internal/mission_cmd_vel')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('planner_service_topic', '/uav/planner/plan_path')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('position_kp', 0.8)
        self.declare_parameter('max_speed_mps', 2.5)
        self.declare_parameter('max_xy_speed_mps', 2.0)
        self.declare_parameter('max_z_speed_mps', 1.0)
        self.declare_parameter('altitude_gain', 0.9)
        self.declare_parameter('velocity_damping_xy', 0.35)
        self.declare_parameter('velocity_damping_z', 0.25)
        self.declare_parameter('approach_slowdown_distance_m', 3.0)
        self.declare_parameter('approach_min_speed_scale', 0.25)
        self.declare_parameter('max_accel_cmd_mps2', 2.5)
        self.declare_parameter('heading_control_enabled', True)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('max_yaw_rate_rps', 1.2)
        self.declare_parameter('heading_min_xy_error_m', 0.4)
        self.declare_parameter('yaw_alignment_min_speed_scale', 0.1)
        self.declare_parameter('default_planner_type', 'astar')
        self.declare_parameter('default_terrain_profile', 'plains')
        self.declare_parameter('planner_collision_inflation_m', 1.0)
        # Output command frame expected by the backend velocity controller.
        # Gazebo multicopter velocity control expects body-frame twist commands.
        self.declare_parameter('command_frame', 'body')  # body | world

        self.mission_topic = self.get_parameter('mission_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.telemetry_raw_topic = self.get_parameter('telemetry_raw_topic').value
        self.mission_cmd_topic = self.get_parameter('mission_cmd_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.planner_service_topic = self.get_parameter('planner_service_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.position_kp = float(self.get_parameter('position_kp').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.max_xy_speed_mps = float(self.get_parameter('max_xy_speed_mps').value)
        self.max_z_speed_mps = float(self.get_parameter('max_z_speed_mps').value)
        self.altitude_gain = float(self.get_parameter('altitude_gain').value)
        self.velocity_damping_xy = float(self.get_parameter('velocity_damping_xy').value)
        self.velocity_damping_z = float(self.get_parameter('velocity_damping_z').value)
        self.approach_slowdown_distance_m = float(
            self.get_parameter('approach_slowdown_distance_m').value
        )
        self.approach_min_speed_scale = float(
            self.get_parameter('approach_min_speed_scale').value
        )
        self.max_accel_cmd_mps2 = float(self.get_parameter('max_accel_cmd_mps2').value)
        self.heading_control_enabled = bool(self.get_parameter('heading_control_enabled').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_rate_rps = float(self.get_parameter('max_yaw_rate_rps').value)
        self.heading_min_xy_error_m = float(self.get_parameter('heading_min_xy_error_m').value)
        self.yaw_alignment_min_speed_scale = float(
            self.get_parameter('yaw_alignment_min_speed_scale').value
        )
        self.default_planner_type = self.get_parameter('default_planner_type').value
        self.default_terrain_profile = self.get_parameter('default_terrain_profile').value
        self.planner_collision_inflation_m = float(
            self.get_parameter('planner_collision_inflation_m').value
        )
        self.command_frame = str(self.get_parameter('command_frame').value).strip().lower()

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
        self.current_vel = None
        self.current_yaw = None
        self.current_mode = Command.MODE_IDLE
        self.planning_mode = Command.PLANNING_OFFBOARD
        self.manual_override = False
        self.hold_started = None
        self.mission_complete = False
        self.active_mission_sequence_id = 0
        self._last_cmd_linear = (0.0, 0.0, 0.0)
        self._last_cmd_time = None

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
        self.current_vel = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        )
        q = msg.pose.orientation
        self.current_yaw = self._yaw_from_quat(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )

    def _on_command(self, msg: Command) -> None:
        self.current_mode = int(msg.mode_request)
        self.planning_mode = int(msg.planning_mode)
        self.manual_override = bool(msg.manual_override)

    def _publish_zero(self) -> None:
        self._last_cmd_linear = (0.0, 0.0, 0.0)
        self._last_cmd_time = self.get_clock().now()
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
        req.collision_inflation_m = float(self.planner_collision_inflation_m)
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

        vx, vy, vz = self.current_vel if self.current_vel is not None else (0.0, 0.0, 0.0)
        cmd = Twist()
        cmd.linear.x = (self.position_kp * dx) - (self.velocity_damping_xy * vx)
        cmd.linear.y = (self.position_kp * dy) - (self.velocity_damping_xy * vy)
        cmd.linear.z = (self.altitude_gain * dz) - (self.velocity_damping_z * vz)

        desired_speed = (
            float(wp.desired_speed_mps)
            if float(wp.desired_speed_mps) > 0.0
            else self.max_speed_mps
        )
        max_speed = max(0.1, min(desired_speed, self.max_speed_mps))
        approach_scale = self._approach_speed_scale(dist=dist, acceptance=acceptance)

        # Limit XY and Z commands separately before final combined clamp.
        xy_norm = math.hypot(cmd.linear.x, cmd.linear.y)
        xy_limit = max(0.05, min(max_speed, self.max_xy_speed_mps) * approach_scale)
        if xy_norm > xy_limit:
            xy_scale = xy_limit / max(xy_norm, 1e-6)
            cmd.linear.x *= xy_scale
            cmd.linear.y *= xy_scale

        z_limit = max(0.05, min(max_speed, self.max_z_speed_mps) * approach_scale)
        cmd.linear.z = max(-z_limit, min(z_limit, cmd.linear.z))

        norm = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2 + cmd.linear.z ** 2)
        if norm > max_speed:
            scale = max_speed / max(norm, 1e-6)
            cmd.linear.x *= scale
            cmd.linear.y *= scale
            cmd.linear.z *= scale

        if self.heading_control_enabled:
            xy_dist = math.hypot(dx, dy)
            if self.current_yaw is not None and xy_dist > max(0.05, self.heading_min_xy_error_m):
                # Drone nose is body +Y. At yaw θ, body +Y points to (-sin θ, cos θ) in world.
                # To face (dx,dy) we need atan2(-dx, dy), not atan2(dy, dx).
                desired_yaw = math.atan2(-dx, dy)
                yaw_err = self._wrap_angle(desired_yaw - self.current_yaw)
                cmd.angular.z = max(
                    -abs(self.max_yaw_rate_rps),
                    min(abs(self.max_yaw_rate_rps), self.yaw_kp * yaw_err),
                )
                # Reduce translational speed more aggressively when badly misaligned.
                align = max(
                    self.yaw_alignment_min_speed_scale,
                    math.cos(min(abs(yaw_err), math.pi / 2.0)),
                )
                cmd.linear.x *= align
                cmd.linear.y *= align
            else:
                cmd.angular.z = 0.0

        # Convert to body frame.
        # Drone nose is body +Y. Project world velocity onto body +Y (forward) and zero body +X (right).
        # body +Y in world at yaw θ = (-sin θ,  cos θ)
        # body +X in world at yaw θ = ( cos θ,  sin θ)
        if self.command_frame == 'body' and self.current_yaw is not None:
            wx = float(cmd.linear.x)
            wy = float(cmd.linear.y)
            s = math.sin(self.current_yaw)
            c = math.cos(self.current_yaw)
            body_fwd = (-s * wx) + (c * wy)   # projection onto body +Y (nose)
            cmd.linear.x = 0.0
            cmd.linear.y = body_fwd

        cmd.linear.x, cmd.linear.y, cmd.linear.z = self._limit_cmd_slew(
            float(cmd.linear.x),
            float(cmd.linear.y),
            float(cmd.linear.z),
        )
        self.pub_cmd.publish(cmd)
        self._publish_status(
            STATUS_ACTIVE,
            False,
            f'following wp {self.active_idx + 1}/{len(self.current_traj.waypoints)} dist={dist:.2f}m',
        )

    def _approach_speed_scale(self, dist: float, acceptance: float) -> float:
        slowdown_dist = max(0.0, self.approach_slowdown_distance_m)
        min_scale = max(0.05, min(1.0, self.approach_min_speed_scale))
        if slowdown_dist <= 0.0:
            return 1.0
        if dist >= slowdown_dist:
            return 1.0
        if dist <= acceptance:
            return min_scale
        span = max(1e-6, slowdown_dist - acceptance)
        t = max(0.0, min(1.0, (dist - acceptance) / span))
        return min_scale + (1.0 - min_scale) * t

    def _limit_cmd_slew(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        max_accel = float(self.max_accel_cmd_mps2)
        now = self.get_clock().now()
        if max_accel <= 0.0 or self._last_cmd_time is None:
            self._last_cmd_linear = (x, y, z)
            self._last_cmd_time = now
            return x, y, z

        dt = max(0.0, (now - self._last_cmd_time).nanoseconds / 1e9)
        self._last_cmd_time = now
        if dt <= 1e-6:
            self._last_cmd_linear = (x, y, z)
            return x, y, z

        max_delta = max_accel * dt
        px, py, pz = self._last_cmd_linear
        dx = max(-max_delta, min(max_delta, x - px))
        dy = max(-max_delta, min(max_delta, y - py))
        dz = max(-max_delta, min(max_delta, z - pz))
        out = (px + dx, py + dy, pz + dz)
        self._last_cmd_linear = out
        return out

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


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
