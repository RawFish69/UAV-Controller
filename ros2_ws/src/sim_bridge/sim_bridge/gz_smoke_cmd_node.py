import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class GzSmokeCmdNode(Node):
    """Publish a simple command pattern to validate Gazebo multicopter motion."""

    def __init__(self) -> None:
        super().__init__('gz_smoke_cmd_node')
        self.declare_parameter('cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('enable_topic', '/uav/backend/enable')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('duration_sec', 12.0)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_enable = self.create_publisher(Bool, self.enable_topic, 10)

        self.start_time = self.get_clock().now()
        self.enable_msg = Bool()
        self.enable_msg.data = True

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Smoke command node publishing to {self.cmd_topic} / {self.enable_topic} at {self.rate_hz:.1f} Hz'
        )

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.pub_enable.publish(self.enable_msg)

        cmd = Twist()
        if elapsed < 2.0:
            cmd.linear.z = 0.8
        elif elapsed < 4.0:
            cmd.linear.z = 0.2
        elif elapsed < 7.0:
            cmd.linear.x = 0.8
            cmd.linear.z = 0.1
        elif elapsed < 10.0:
            cmd.linear.y = 0.6
            cmd.angular.z = 0.5
        elif elapsed < self.duration_sec:
            cmd.linear.z = -0.4
        else:
            # Send a final zero and remain idle.
            cmd = Twist()
        # Small oscillation on yaw to make motion visible during hover/translation.
        if elapsed < self.duration_sec and elapsed > 2.0:
            cmd.angular.z += 0.1 * math.sin(elapsed * 2.0)

        self.pub_cmd.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzSmokeCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
