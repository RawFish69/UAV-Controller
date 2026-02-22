import math
from typing import Optional

import rclpy
from drone_msgs.msg import Command, Telemetry
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CommandManagerNode(Node):
    """Authoritative air-unit state machine and backend command arbiter."""

    def __init__(self) -> None:
        super().__init__('command_manager_node')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('telemetry_raw_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('mission_cmd_topic', '/uav/internal/mission_cmd_vel')
        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('telemetry_topic', '/uav/telemetry')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 2.0)
        self.declare_parameter('mission_cmd_timeout_sec', 1.0)
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter('takeoff_rate_mps', 0.8)
        self.declare_parameter('land_rate_mps', 0.5)
        self.declare_parameter('land_altitude_threshold_m', 0.15)
        self.declare_parameter('manual_scale_xy', 1.5)
        self.declare_parameter('manual_scale_z', 1.0)
        self.declare_parameter('manual_scale_yaw', 1.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.telemetry_raw_topic = self.get_parameter('telemetry_raw_topic').value
        self.mission_cmd_topic = self.get_parameter('mission_cmd_topic').value
        self.backend_cmd_topic = self.get_parameter('backend_cmd_topic').value
        self.backend_enable_topic = self.get_parameter('backend_enable_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.mission_cmd_timeout_sec = float(self.get_parameter('mission_cmd_timeout_sec').value)
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.takeoff_rate_mps = float(self.get_parameter('takeoff_rate_mps').value)
        self.land_rate_mps = float(self.get_parameter('land_rate_mps').value)
        self.land_altitude_threshold_m = float(self.get_parameter('land_altitude_threshold_m').value)
        self.manual_scale_xy = float(self.get_parameter('manual_scale_xy').value)
        self.manual_scale_z = float(self.get_parameter('manual_scale_z').value)
        self.manual_scale_yaw = float(self.get_parameter('manual_scale_yaw').value)

        self.pub_backend_cmd = self.create_publisher(Twist, self.backend_cmd_topic, 10)
        self.pub_backend_enable = self.create_publisher(Bool, self.backend_enable_topic, 10)
        self.pub_telemetry = self.create_publisher(Telemetry, self.telemetry_topic, 20)

        self.create_subscription(Command, self.command_topic, self._on_command, 20)
        self.create_subscription(Telemetry, self.telemetry_raw_topic, self._on_telemetry_raw, 20)
        self.create_subscription(Twist, self.mission_cmd_topic, self._on_mission_cmd, 20)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.mode = Command.MODE_IDLE
        self.planning_mode = Command.PLANNING_OFFBOARD
        self.armed = False
        self.manual_override = False
        self.last_manual_cmd = Twist()
        self.last_mission_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_mission_cmd_time = self.get_clock().now()
        self.last_raw_telemetry: Optional[Telemetry] = None
        self.last_output = Twist()
        self.status_text = 'idle'

        self.get_logger().info('Command manager started')

    def _on_command(self, msg: Command) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.planning_mode = int(msg.planning_mode)
        self.manual_override = bool(msg.manual_override)

        if msg.disarm:
            self.armed = False
            self.mode = Command.MODE_IDLE
            self.status_text = 'disarmed'
        elif msg.arm:
            self.armed = True
            self.status_text = 'armed'

        self.mode = int(msg.mode_request)
        self.last_manual_cmd = self._command_to_manual_twist(msg)

    def _on_telemetry_raw(self, msg: Telemetry) -> None:
        self.last_raw_telemetry = msg
        self._publish_enriched_telemetry(msg)

    def _on_mission_cmd(self, msg: Twist) -> None:
        self.last_mission_cmd = msg
        self.last_mission_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        cmd_timeout = (now - self.last_cmd_time).nanoseconds / 1e9
        mission_timeout = (now - self.last_mission_cmd_time).nanoseconds / 1e9

        if cmd_timeout > self.command_timeout_sec and self.armed and not self.manual_override:
            if self.mode not in (Command.MODE_HOVER, Command.MODE_LAND):
                self.mode = Command.MODE_HOVER
                self.status_text = f'command timeout {cmd_timeout:.1f}s -> hover'

        enable = Bool()
        enable.data = bool(self.armed)
        cmd = Twist()
        altitude = self._current_altitude()

        if not self.armed:
            self.status_text = 'idle/disarmed'
        elif self.manual_override or self.mode == Command.MODE_MANUAL_OVERRIDE:
            cmd = self.last_manual_cmd
            self.status_text = 'manual override'
        elif self.mode == Command.MODE_TAKEOFF:
            if altitude is not None and altitude >= self.takeoff_altitude_m:
                self.mode = Command.MODE_HOVER
                self.status_text = 'takeoff complete -> hover'
            else:
                cmd.linear.z = self.takeoff_rate_mps
                self.status_text = 'takeoff'
        elif self.mode == Command.MODE_HOVER:
            self.status_text = 'hover'
        elif self.mode == Command.MODE_MISSION:
            if mission_timeout <= self.mission_cmd_timeout_sec:
                cmd = self.last_mission_cmd
                self.status_text = 'mission tracking'
            else:
                self.status_text = 'mission waiting for setpoint'
        elif self.mode == Command.MODE_LAND:
            if altitude is not None and altitude <= self.land_altitude_threshold_m:
                self.armed = False
                enable.data = False
                self.mode = Command.MODE_IDLE
                self.status_text = 'landed -> disarmed'
            else:
                cmd.linear.z = -abs(self.land_rate_mps)
                self.status_text = 'landing'
        else:
            self.status_text = f'mode {self.mode} idle behavior'

        self.last_output = cmd
        self.pub_backend_enable.publish(enable)
        self.pub_backend_cmd.publish(cmd)

        if self.last_raw_telemetry is not None:
            self._publish_enriched_telemetry(self.last_raw_telemetry)

    def _publish_enriched_telemetry(self, raw: Telemetry) -> None:
        msg = Telemetry()
        msg.header = raw.header
        msg.pose = raw.pose
        msg.twist = raw.twist
        msg.battery_percent = raw.battery_percent
        msg.current_mode = int(self.mode)
        msg.planning_mode = int(self.planning_mode)
        msg.armed = bool(self.armed)
        msg.manual_override_active = bool(self.manual_override)
        msg.status_text = self.status_text or raw.status_text
        self.pub_telemetry.publish(msg)

    def _current_altitude(self) -> Optional[float]:
        if self.last_raw_telemetry is None:
            return None
        return float(self.last_raw_telemetry.pose.position.z)

    def _command_to_manual_twist(self, msg: Command) -> Twist:
        out = Twist()
        vs = msg.velocity_setpoint
        if not self._is_zero_twist(vs):
            out = vs
            return out

        if msg.manual.active:
            out.linear.x = float(msg.manual.pitch) * self.manual_scale_xy
            out.linear.y = float(msg.manual.roll) * self.manual_scale_xy
            out.linear.z = (float(msg.manual.throttle) - 0.5) * 2.0 * self.manual_scale_z
            out.angular.z = float(msg.manual.yaw) * self.manual_scale_yaw
        return out

    @staticmethod
    def _is_zero_twist(t: Twist) -> bool:
        vals = (
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        )
        return all(math.isclose(v, 0.0, abs_tol=1e-6) for v in vals)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
