from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class SimResetRecoveryNode(Node):
    """Detect Gazebo reset events and re-initialize backend command/enable topics."""

    def __init__(self) -> None:
        super().__init__('sim_reset_recovery_node')

        self.declare_parameter('enable_reset_recovery', True)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('odom_topic', '/uav/backend/odom')
        self.declare_parameter('backend_enable_topic', '/uav/backend/enable')
        self.declare_parameter('backend_cmd_topic', '/uav/backend/cmd_twist')
        self.declare_parameter('path_marker_topic', '/gs/planner/planned_path_markers')
        self.declare_parameter('clear_path_markers_on_reset', True)
        self.declare_parameter('settle_time_sec', 1.0)
        self.declare_parameter('stable_samples_required', 10)
        self.declare_parameter('max_reenable_speed_mps', 0.5)
        self.declare_parameter('min_valid_altitude_m', -2.0)
        self.declare_parameter('max_valid_altitude_m', 100.0)
        self.declare_parameter('reset_cooldown_sec', 1.5)

        self.enable_reset_recovery = bool(self.get_parameter('enable_reset_recovery').value)
        self.clock_topic = str(self.get_parameter('clock_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.backend_enable_topic = str(self.get_parameter('backend_enable_topic').value)
        self.backend_cmd_topic = str(self.get_parameter('backend_cmd_topic').value)
        self.path_marker_topic = str(self.get_parameter('path_marker_topic').value)
        self.clear_path_markers_on_reset = bool(
            self.get_parameter('clear_path_markers_on_reset').value
        )
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.stable_samples_required = int(self.get_parameter('stable_samples_required').value)
        self.max_reenable_speed_mps = float(self.get_parameter('max_reenable_speed_mps').value)
        self.min_valid_altitude_m = float(self.get_parameter('min_valid_altitude_m').value)
        self.max_valid_altitude_m = float(self.get_parameter('max_valid_altitude_m').value)
        self.reset_cooldown_sec = float(self.get_parameter('reset_cooldown_sec').value)

        self.pub_enable = self.create_publisher(Bool, self.backend_enable_topic, 10)
        self.pub_cmd = self.create_publisher(Twist, self.backend_cmd_topic, 10)
        self.pub_path_markers = self.create_publisher(MarkerArray, self.path_marker_topic, 10)

        self.create_subscription(Clock, self.clock_topic, self._on_clock, 20)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self._timer = self.create_timer(0.05, self._tick)

        self._last_clock_sec: Optional[float] = None
        self._last_reset_monotonic = -1e9
        self._recovering = False
        self._recover_start_monotonic = 0.0
        self._stable_samples = 0
        self._last_odom: Optional[Odometry] = None
        self._last_odom_monotonic = 0.0

        self.get_logger().info(
            'Reset recovery watching %s (odom=%s, enable=%s, cmd=%s)'
            % (self.clock_topic, self.odom_topic, self.backend_enable_topic, self.backend_cmd_topic)
        )

    def _on_clock(self, msg: Clock) -> None:
        if not self.enable_reset_recovery:
            return
        sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self._last_clock_sec is not None and sec + 1e-6 < self._last_clock_sec:
            now = time.monotonic()
            if (now - self._last_reset_monotonic) >= max(0.1, self.reset_cooldown_sec):
                self._trigger_reset_recovery(
                    reason=f'clock moved backwards ({self._last_clock_sec:.3f} -> {sec:.3f})'
                )
        self._last_clock_sec = sec

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_monotonic = time.monotonic()

    def _trigger_reset_recovery(self, reason: str) -> None:
        self._last_reset_monotonic = time.monotonic()
        self._recovering = True
        self._recover_start_monotonic = self._last_reset_monotonic
        self._stable_samples = 0
        self.get_logger().warn(f'Detected simulation reset: {reason}. Starting backend recovery.')
        if self.clear_path_markers_on_reset and self.path_marker_topic:
            self._publish_deleteall_path_markers()

    def _tick(self) -> None:
        if not self._recovering:
            return

        # Keep override active during recovery so command_manager cannot immediately re-enable.
        self.pub_enable.publish(Bool(data=False))
        self.pub_cmd.publish(Twist())

        now = time.monotonic()
        if now - self._recover_start_monotonic < max(0.0, self.settle_time_sec):
            return
        if self._last_odom is None:
            return
        if (now - self._last_odom_monotonic) > 1.0:
            self._stable_samples = 0
            return

        if not self._odom_is_stable(self._last_odom):
            self._stable_samples = 0
            return

        self._stable_samples += 1
        if self._stable_samples < max(1, self.stable_samples_required):
            return

        self.pub_enable.publish(Bool(data=True))
        self._recovering = False
        self.get_logger().info(
            f'Reset recovery complete after {now - self._recover_start_monotonic:.2f}s'
        )

    def _odom_is_stable(self, msg: Odometry) -> bool:
        p = msg.pose.pose.position
        t = msg.twist.twist.linear
        vals = (p.x, p.y, p.z, t.x, t.y, t.z)
        if not all(math.isfinite(float(v)) for v in vals):
            return False
        if float(p.z) < self.min_valid_altitude_m or float(p.z) > self.max_valid_altitude_m:
            return False
        speed = math.sqrt(float(t.x) ** 2 + float(t.y) ** 2 + float(t.z) ** 2)
        if speed > max(0.05, self.max_reenable_speed_mps):
            return False
        return True

    def _publish_deleteall_path_markers(self) -> None:
        msg = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        msg.markers.append(m)
        self.pub_path_markers.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimResetRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
