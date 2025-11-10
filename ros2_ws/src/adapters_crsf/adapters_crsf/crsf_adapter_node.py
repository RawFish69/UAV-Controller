#!/usr/bin/env python3
"""
CRSF Adapter Node

Subscribes to /cmd/final/rc and transmits to TX_RX ESP32 via UDP or Serial.
Uses pluggable packer system to match firmware packet format.
"""

import importlib
import socket
import time

import rclpy
from rclpy.node import Node
from common_msgs.msg import VirtualRC

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class CRSFAdapterNode(Node):
    """CRSF adapter with pluggable packet packer."""

    def __init__(self):
        super().__init__('crsf_adapter_node')

        # Declare parameters
        self.declare_parameter('transport', 'udp')
        self.declare_parameter('udp_host', '192.168.4.1')
        self.declare_parameter('udp_port', 9000)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('rate_hz', 150)
        self.declare_parameter('failsafe_ms', 200)
        self.declare_parameter('packer', 'txrx_packer')

        # Get parameters
        self.transport = self.get_parameter('transport').value
        self.udp_host = self.get_parameter('udp_host').value
        self.udp_port = self.get_parameter('udp_port').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.failsafe_ms = self.get_parameter('failsafe_ms').value
        packer_name = self.get_parameter('packer').value

        # Load packer
        self.packer = self._load_packer(packer_name)

        # Initialize transport
        self.transport_handle = None
        if self.transport == 'udp':
            self._init_udp()
        elif self.transport == 'serial':
            self._init_serial()
        else:
            self.get_logger().error(f'Invalid transport: {self.transport}')
            raise ValueError(f'Invalid transport: {self.transport}')

        # State
        self.last_rc_msg = None
        self.last_rc_time = self.get_clock().now()

        # Subscriber
        self.sub_rc = self.create_subscription(
            VirtualRC,
            '/cmd/final/rc',
            self.rc_callback,
            10
        )

        # Timer for periodic transmission
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.send_packet)

        self.get_logger().info(
            f'CRSF Adapter initialized: {self.transport} @ {self.rate_hz} Hz, '
            f'packer: {packer_name}'
        )

    def _load_packer(self, packer_name):
        """Dynamically load packer module."""
        try:
            # Try to import from adapters_crsf.packers
            module_name = f'adapters_crsf.packers.{packer_name}'
            module = importlib.import_module(module_name)
            
            # Get class name (capitalize first letter, add Packer suffix if not present)
            class_name = ''.join(word.capitalize() for word in packer_name.split('_'))
            if not class_name.endswith('Packer'):
                class_name += 'Packer'
            
            packer_class = getattr(module, class_name)
            packer = packer_class()
            
            self.get_logger().info(f'Loaded packer: {packer_name} ({class_name})')
            return packer
        except Exception as e:
            self.get_logger().error(f'Failed to load packer {packer_name}: {e}')
            raise

    def _init_udp(self):
        """Initialize UDP socket."""
        try:
            self.transport_handle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info(f'UDP socket ready: {self.udp_host}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            raise

    def _init_serial(self):
        """Initialize serial port."""
        if not HAS_SERIAL:
            self.get_logger().error('pyserial not installed, cannot use serial transport')
            raise ImportError('pyserial not installed')

        try:
            self.transport_handle = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud,
                timeout=0.01
            )
            self.get_logger().info(
                f'Serial port ready: {self.serial_port} @ {self.baud} baud'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

    def rc_callback(self, msg):
        """Handle incoming RC commands."""
        self.last_rc_msg = msg
        self.last_rc_time = self.get_clock().now()

    def send_packet(self):
        """Send packet at configured rate."""
        if self.transport_handle is None:
            return

        # Check for timeout (failsafe)
        elapsed_ms = (self.get_clock().now() - self.last_rc_time).nanoseconds / 1e6
        
        if elapsed_ms > self.failsafe_ms or self.last_rc_msg is None:
            # Send safe idle
            packet = self.packer.safe_idle()
            if elapsed_ms > self.failsafe_ms * 2:  # Warn only occasionally
                self.get_logger().warn(
                    f'Failsafe triggered ({elapsed_ms:.0f} ms), sending safe idle',
                    throttle_duration_sec=2.0
                )
        else:
            # Send normal command
            packet = self.packer.encode(self.last_rc_msg)

        # Transmit
        try:
            if self.transport == 'udp':
                self.transport_handle.sendto(packet, (self.udp_host, self.udp_port))
            elif self.transport == 'serial':
                self.transport_handle.write(packet)
        except Exception as e:
            self.get_logger().error(f'Transmission error: {e}', throttle_duration_sec=1.0)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.transport_handle is not None:
            if self.transport == 'serial':
                self.transport_handle.close()
            self.get_logger().info('Transport closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CRSFAdapterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

