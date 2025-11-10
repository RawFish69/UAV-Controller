#!/usr/bin/env python3
"""
IMU Gesture Controller Node

Reads IMU data from IMU TX and converts orientation to drone attitude setpoints.
Enables intuitive gesture control:
- Tilt hand forward → drone pitches forward
- Roll hand → drone rolls
- Throttle from potentiometer or joystick

PID controller's outer loop tracks the attitude setpoint.
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from common_msgs.msg import AttitudeThrust
from std_msgs.msg import Float32


class IMUControllerNode(Node):
    """Converts IMU TX orientation to drone attitude commands."""

    def __init__(self):
        super().__init__('imu_controller_node')

        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('throttle_topic', '/manual/throttle')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('max_tilt_deg', 30.0)
        self.declare_parameter('hover_throttle', 0.5)
        self.declare_parameter('deadzone_deg', 5.0)
        self.declare_parameter('orientation_mode', 'direct')  # direct or rate
        self.declare_parameter('roll_scale', 1.0)
        self.declare_parameter('pitch_scale', 1.0)
        self.declare_parameter('yaw_rate_scale', 1.0)

        # Get parameters
        imu_topic = self.get_parameter('imu_topic').value
        throttle_topic = self.get_parameter('throttle_topic').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.max_tilt_deg = self.get_parameter('max_tilt_deg').value
        self.hover_throttle = self.get_parameter('hover_throttle').value
        self.deadzone_deg = self.get_parameter('deadzone_deg').value
        self.orientation_mode = self.get_parameter('orientation_mode').value
        self.roll_scale = self.get_parameter('roll_scale').value
        self.pitch_scale = self.get_parameter('pitch_scale').value
        self.yaw_rate_scale = self.get_parameter('yaw_rate_scale').value

        # State
        self.current_imu_quat = None
        self.throttle = self.hover_throttle
        self.yaw_offset = 0.0  # For yaw centering

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        self.sub_throttle = self.create_subscription(
            Float32,
            throttle_topic,
            self.throttle_callback,
            10
        )

        # Publisher
        self.pub_cmd = self.create_publisher(
            AttitudeThrust,
            '/cmd/attitude_thrust',
            10
        )

        # Timer for control loop
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'IMU Controller initialized: {self.orientation_mode} mode, '
            f'max_tilt={self.max_tilt_deg}°'
        )

    def imu_callback(self, msg):
        """Receive IMU orientation from IMU TX."""
        self.current_imu_quat = msg.orientation

    def throttle_callback(self, msg):
        """Receive throttle command from joystick or other input."""
        self.throttle = np.clip(msg.data, 0.0, 1.0)

    def control_loop(self):
        """Generate attitude setpoint from IMU TX orientation."""
        if self.current_imu_quat is None:
            return

        # Convert quaternion to rotation object
        q = self.current_imu_quat
        hand_rot = Rotation.from_quat([q.x, q.y, q.z, q.w])

        # Get Euler angles from hand orientation (in radians)
        hand_euler = hand_rot.as_euler('xyz', degrees=False)
        hand_roll, hand_pitch, hand_yaw = hand_euler

        # Apply scaling and limits
        roll_deg = np.degrees(hand_roll) * self.roll_scale
        pitch_deg = np.degrees(hand_pitch) * self.pitch_scale

        # Apply deadzone
        if abs(roll_deg) < self.deadzone_deg:
            roll_deg = 0.0
        if abs(pitch_deg) < self.deadzone_deg:
            pitch_deg = 0.0

        # Clamp to max tilt
        roll_deg = np.clip(roll_deg, -self.max_tilt_deg, self.max_tilt_deg)
        pitch_deg = np.clip(pitch_deg, -self.max_tilt_deg, self.max_tilt_deg)

        # Convert back to radians
        roll_cmd = np.radians(roll_deg)
        pitch_cmd = np.radians(pitch_deg)

        # Yaw: use rate control from hand yaw velocity
        # Or use absolute yaw with offset for centering
        yaw_cmd = hand_yaw - self.yaw_offset

        # Create attitude command (FRD frame)
        # Convert roll, pitch, yaw to quaternion
        cmd_rot = Rotation.from_euler('xyz', [roll_cmd, pitch_cmd, yaw_cmd])
        cmd_quat = cmd_rot.as_quat()  # [x, y, z, w]

        # Publish attitude setpoint
        msg = AttitudeThrust()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        msg.attitude.x = cmd_quat[0]
        msg.attitude.y = cmd_quat[1]
        msg.attitude.z = cmd_quat[2]
        msg.attitude.w = cmd_quat[3]
        msg.thrust = self.throttle

        self.pub_cmd.publish(msg)

    def center_yaw(self):
        """Reset yaw reference to current IMU TX orientation."""
        if self.current_imu_quat is not None:
            q = self.current_imu_quat
            hand_rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            _, _, yaw = hand_rot.as_euler('xyz', degrees=False)
            self.yaw_offset = yaw
            self.get_logger().info('Yaw centered')


def main(args=None):
    rclpy.init(args=args)
    node = IMUControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

