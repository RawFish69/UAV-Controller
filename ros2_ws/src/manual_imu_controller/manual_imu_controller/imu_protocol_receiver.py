#!/usr/bin/env python3
"""
IMU Protocol Receiver

Receives IMU data packets from IMU TX ESP32 via UDP or Serial.
Publishes to standard ROS IMU and throttle topics.

Packet format (ImuDataPayload):
  float qw, qx, qy, qz  # Quaternion
  float throttle        # 0.0 to 1.0
  uint32_t timestamp
  uint8_t flags         # Bit 0: calibrated, Bit 1: button
"""

import struct
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class ImuProtocolReceiver(Node):
    """Receives IMU data via custom protocol from IMU TX ESP32."""

    def __init__(self):
        super().__init__('imu_protocol_receiver')

        # Declare parameters
        self.declare_parameter('transport', 'udp')
        self.declare_parameter('udp_port', 8000)  # Receive port
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout_ms', 500)

        # Get parameters
        self.transport = self.get_parameter('transport').value
        self.udp_port = self.get_parameter('udp_port').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud').value
        self.timeout_ms = self.get_parameter('timeout_ms').value

        # Packet format: 4 floats + 1 float + 1 uint32 + 1 uint8 = 25 bytes
        self.packet_format = '<fffffIB'
        self.packet_size = struct.calcsize(self.packet_format)

        # Initialize transport
        self.transport_handle = None
        if self.transport == 'udp':
            self._init_udp()
        elif self.transport == 'serial':
            self._init_serial()
        else:
            self.get_logger().error(f'Invalid transport: {self.transport}')
            raise ValueError(f'Invalid transport: {self.transport}')

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_throttle = self.create_publisher(Float32, '/manual/throttle', 10)
        self.pub_calibrated = self.create_publisher(Bool, '/imu/calibrated', 10)
        self.pub_button = self.create_publisher(Bool, '/manual/button', 10)

        # State
        self.last_packet_time = self.get_clock().now()

        # Timer for receiving packets
        self.timer = self.create_timer(0.001, self.receive_packet)  # 1kHz check rate

        # Timeout timer
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(
            f'IMU Protocol Receiver initialized: {self.transport}, '
            f'packet size: {self.packet_size} bytes'
        )

    def _init_udp(self):
        """Initialize UDP socket."""
        try:
            self.transport_handle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.transport_handle.bind(('0.0.0.0', self.udp_port))
            self.transport_handle.settimeout(0.001)  # Non-blocking
            self.get_logger().info(f'UDP receiver ready on port {self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            raise

    def _init_serial(self):
        """Initialize serial port."""
        if not HAS_SERIAL:
            self.get_logger().error('pyserial not installed')
            raise ImportError('pyserial not installed')

        try:
            self.transport_handle = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud,
                timeout=0.001
            )
            self.get_logger().info(
                f'Serial receiver ready: {self.serial_port} @ {self.baud}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

    def receive_packet(self):
        """Receive and parse IMU packets."""
        if self.transport_handle is None:
            return

        try:
            data = None
            if self.transport == 'udp':
                try:
                    data, addr = self.transport_handle.recvfrom(1024)
                except socket.timeout:
                    return
            elif self.transport == 'serial':
                if self.transport_handle.in_waiting >= self.packet_size:
                    data = self.transport_handle.read(self.packet_size)

            if data and len(data) == self.packet_size:
                self.parse_packet(data)

        except Exception as e:
            self.get_logger().error(
                f'Receive error: {e}',
                throttle_duration_sec=1.0
            )

    def parse_packet(self, data):
        """Parse IMU data packet."""
        try:
            qw, qx, qy, qz, throttle, timestamp, flags = struct.unpack(
                self.packet_format, data
            )

            now = self.get_clock().now()
            self.last_packet_time = now

            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = 'hand_imu'
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            # Note: angular velocity and linear accel not provided
            self.pub_imu.publish(imu_msg)

            # Publish throttle
            throttle_msg = Float32()
            throttle_msg.data = throttle
            self.pub_throttle.publish(throttle_msg)

            # Publish calibration status
            calibrated = bool(flags & 0x01)
            calib_msg = Bool()
            calib_msg.data = calibrated
            self.pub_calibrated.publish(calib_msg)

            # Publish button state
            button_pressed = bool(flags & 0x02)
            button_msg = Bool()
            button_msg.data = button_pressed
            self.pub_button.publish(button_msg)

        except Exception as e:
            self.get_logger().error(
                f'Parse error: {e}',
                throttle_duration_sec=1.0
            )

    def check_timeout(self):
        """Check for packet timeout."""
        elapsed_ms = (self.get_clock().now() - self.last_packet_time).nanoseconds / 1e6
        if elapsed_ms > self.timeout_ms:
            self.get_logger().warn(
                f'No IMU data for {elapsed_ms:.0f} ms',
                throttle_duration_sec=2.0
            )

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.transport_handle is not None:
            if self.transport == 'serial':
                self.transport_handle.close()
            self.get_logger().info('Transport closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuProtocolReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

