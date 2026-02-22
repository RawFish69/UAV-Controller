import time

import rclpy
from drone_msgs.msg import MissionStatus, Telemetry
from rclpy.node import Node


class GroundStationTelemetryMonitor(Node):
    def __init__(self) -> None:
        super().__init__('ground_station_telemetry_monitor')
        self.declare_parameter('telemetry_topic', '/uav/telemetry')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('telemetry_log_period_sec', 1.0)
        self.declare_parameter('mission_status_on_change_only', True)
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.telemetry_log_period_sec = float(self.get_parameter('telemetry_log_period_sec').value)
        self.mission_status_on_change_only = bool(
            self.get_parameter('mission_status_on_change_only').value
        )
        self._last_telemetry_log_t = 0.0
        self._last_mission_status_key = None

        self.create_subscription(Telemetry, self.telemetry_topic, self._on_telemetry, 20)
        self.create_subscription(MissionStatus, self.mission_status_topic, self._on_mission_status, 20)
        self.get_logger().info(f'Monitoring telemetry on {self.telemetry_topic}')

    def _on_telemetry(self, msg: Telemetry) -> None:
        now_t = time.monotonic()
        if self.telemetry_log_period_sec > 0.0:
            if (now_t - self._last_telemetry_log_t) < self.telemetry_log_period_sec:
                return
        self._last_telemetry_log_t = now_t

        p = msg.pose.position
        v = msg.twist.linear
        self.get_logger().info(
            f"mode={msg.current_mode} armed={msg.armed} manual={msg.manual_override_active} "
            f"pos=({p.x:.2f},{p.y:.2f},{p.z:.2f}) vel=({v.x:.2f},{v.y:.2f},{v.z:.2f}) "
            f"bat={msg.battery_percent:.1f}% status='{msg.status_text}'"
        )

    def _on_mission_status(self, msg: MissionStatus) -> None:
        key = (
            int(msg.mission_sequence_id),
            int(msg.active_waypoint_index),
            int(msg.total_waypoints),
            int(msg.state),
            bool(msg.complete),
            str(msg.status_text),
        )
        if self.mission_status_on_change_only and key == self._last_mission_status_key:
            return
        self._last_mission_status_key = key

        self.get_logger().info(
            f"mission seq={msg.mission_sequence_id} wp={msg.active_waypoint_index}/{msg.total_waypoints} "
            f"state={msg.state} complete={msg.complete} '{msg.status_text}'"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationTelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
