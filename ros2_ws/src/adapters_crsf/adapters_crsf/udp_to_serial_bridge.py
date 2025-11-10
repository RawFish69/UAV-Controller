#!/usr/bin/env python3
"""
UDP to Serial Bridge

Forwards UDP packets to Serial port. Useful when TX_RX ESP32 is
connected via USB cable instead of WiFi.

Usage:
    ros2 run adapters_crsf udp_to_serial_bridge \
        --ros-args -p udp_port:=9000 -p serial_port:=/dev/ttyUSB0 -p baud:=921600
"""

import socket
import serial
import rclpy
from rclpy.node import Node


class UDPToSerialBridge(Node):
    """Bridge UDP packets to Serial port."""

    def __init__(self):
        super().__init__('udp_to_serial_bridge')

        # Declare parameters
        self.declare_parameter('udp_port', 9000)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)

        # Get parameters
        self.udp_port = self.get_parameter('udp_port').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud').value

        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', self.udp_port))
        self.udp_socket.settimeout(0.1)  # Non-blocking with timeout

        # Initialize serial port
        self.serial_port_handle = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud,
            timeout=0.01
        )

        self.get_logger().info(
            f'Bridge started: UDP port {self.udp_port} -> Serial {self.serial_port} @ {self.baud}'
        )

        # Timer for receiving UDP packets
        self.timer = self.create_timer(0.001, self.forward_packets)  # 1 kHz check rate

    def forward_packets(self):
        """Receive UDP and forward to Serial."""
        try:
            data, addr = self.udp_socket.recvfrom(1024)
            if data:
                self.serial_port_handle.write(data)
        except socket.timeout:
            pass  # No data available
        except Exception as e:
            self.get_logger().error(f'Forward error: {e}', throttle_duration_sec=1.0)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.serial_port_handle:
            self.serial_port_handle.close()
        if self.udp_socket:
            self.udp_socket.close()
        self.get_logger().info('Bridge closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPToSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

