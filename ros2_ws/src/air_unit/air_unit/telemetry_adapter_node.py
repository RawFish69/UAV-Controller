import math

import rclpy
from drone_msgs.msg import Telemetry
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TelemetryAdapterNode(Node):
    """Convert backend odometry into the V2 telemetry message (raw/base form)."""

    def __init__(self) -> None:
        super().__init__('telemetry_adapter_node')
        self.declare_parameter('backend_odom_topic', '/uav/backend/odom')
        self.declare_parameter('backend_telemetry_raw_topic', '/uav/backend/telemetry_raw')
        self.declare_parameter('battery_drain_per_sec', 0.02)

        self.backend_odom_topic = self.get_parameter('backend_odom_topic').value
        self.backend_telemetry_raw_topic = self.get_parameter('backend_telemetry_raw_topic').value
        self.battery_drain_per_sec = float(self.get_parameter('battery_drain_per_sec').value)

        self.pub_raw = self.create_publisher(Telemetry, self.backend_telemetry_raw_topic, 20)
        self.sub_odom = self.create_subscription(
            Odometry,
            self.backend_odom_topic,
            self._on_odom,
            20,
        )

        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'Telemetry adapter listening on {self.backend_odom_topic}, publishing {self.backend_telemetry_raw_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Telemetry()
        out.header = msg.header
        out.pose = msg.pose.pose
        out.twist = msg.twist.twist

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        out.battery_percent = float(max(0.0, 100.0 - self.battery_drain_per_sec * elapsed))
        out.current_mode = 0
        out.planning_mode = 0
        out.armed = False
        out.manual_override_active = False

        speed = math.sqrt(
            out.twist.linear.x ** 2 +
            out.twist.linear.y ** 2 +
            out.twist.linear.z ** 2
        )
        out.status_text = f'backend odom ok; speed={speed:.2f} m/s'
        self.pub_raw.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
