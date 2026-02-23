import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class GazeboBackendAdapterNode(Node):
    """Bridge internal backend topics to ros_gz-bridged Gazebo topics."""

    def __init__(self) -> None:
        super().__init__('gazebo_backend_adapter_node')

        self.declare_parameter('uav_namespace', '/uav')
        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('backend_odom_topic', '/uav/backend/odom')
        self.declare_parameter('gz_cmd_topic', '/lr_drone/gazebo/command/twist')
        self.declare_parameter('gz_enable_topic', '/lr_drone/enable')
        self.declare_parameter('gz_odom_topic', '/model/lr_drone/odometry')
        self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('publish_zero_on_timeout', True)

        self.backend_cmd_topic = self.get_parameter('backend_cmd_topic').value
        self.backend_enable_topic = self.get_parameter('backend_enable_topic').value
        self.backend_odom_topic = self.get_parameter('backend_odom_topic').value
        self.gz_cmd_topic = self.get_parameter('gz_cmd_topic').value
        self.gz_enable_topic = self.get_parameter('gz_enable_topic').value
        self.gz_odom_topic = self.get_parameter('gz_odom_topic').value
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.publish_zero_on_timeout = bool(self.get_parameter('publish_zero_on_timeout').value)

        self.last_cmd_time = self.get_clock().now()
        self.enabled = False
        self.last_cmd = Twist()

        self.pub_gz_cmd = self.create_publisher(Twist, self.gz_cmd_topic, 10)
        self.pub_gz_enable = self.create_publisher(Bool, self.gz_enable_topic, 10)
        self.pub_backend_odom = self.create_publisher(Odometry, self.backend_odom_topic, 10)

        self.sub_backend_cmd = self.create_subscription(
            Twist,
            self.backend_cmd_topic,
            self._on_backend_cmd,
            10,
        )
        self.sub_backend_enable = self.create_subscription(
            Bool,
            self.backend_enable_topic,
            self._on_backend_enable,
            10,
        )
        self.sub_gz_odom = self.create_subscription(
            Odometry,
            self.gz_odom_topic,
            self._on_gz_odom,
            20,
        )

        self.timeout_timer = self.create_timer(0.1, self._on_timeout_check)
        self.enable_republish_timer = self.create_timer(1.0, self._republish_enable_if_armed)

        self._startup_hover_active = True
        self._startup_hover_timer = self.create_timer(0.05, self._publish_startup_hover)

        self.get_logger().info(
            'Gazebo backend adapter started (startup hover active): %s -> %s, %s -> %s, %s -> %s'
            % (
                self.backend_cmd_topic,
                self.gz_cmd_topic,
                self.backend_enable_topic,
                self.gz_enable_topic,
                self.gz_odom_topic,
                self.backend_odom_topic,
            )
        )

    def _publish_startup_hover(self) -> None:
        """Keep the drone inert on the ground until the first arm command arrives."""
        if not self._startup_hover_active:
            self._startup_hover_timer.cancel()
            return
        enable_msg = Bool()
        enable_msg.data = False
        self.pub_gz_enable.publish(enable_msg)

    def _on_backend_cmd(self, msg: Twist) -> None:
        if self._startup_hover_active:
            return
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.pub_gz_cmd.publish(msg)

    def _on_backend_enable(self, msg: Bool) -> None:
        if self._startup_hover_active:
            if msg.data:
                self._startup_hover_active = False
                self.get_logger().info('Arm received – ending startup hover, handing control to command manager')
            else:
                return
        self.enabled = bool(msg.data)
        self.pub_gz_enable.publish(msg)

    def _republish_enable_if_armed(self) -> None:
        """Re-publish enable to Gazebo periodically when armed so the plugin reliably gets it."""
        if not self.enabled:
            return
        msg = Bool()
        msg.data = True
        self.pub_gz_enable.publish(msg)

    def _on_gz_odom(self, msg: Odometry) -> None:
        if msg.header.frame_id in ('', 'odom'):
            msg.header.frame_id = 'map'
        if msg.child_frame_id == '':
            msg.child_frame_id = 'base_link'
        self.pub_backend_odom.publish(msg)

    def _on_timeout_check(self) -> None:
        if not self.publish_zero_on_timeout:
            return
        if not self.enabled:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed <= self.command_timeout_sec:
            return

        zero = Twist()
        if not self._twist_is_zero(self.last_cmd):
            self.pub_gz_cmd.publish(zero)
            self.last_cmd = zero
            self.get_logger().warn(
                f'Command timeout ({elapsed:.2f}s): publishing zero twist'
            )

    @staticmethod
    def _twist_is_zero(msg: Twist) -> bool:
        vals = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        return all(math.isclose(v, 0.0, abs_tol=1e-6) for v in vals)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboBackendAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
