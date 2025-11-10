"""
TX_RX packer: matches the DirectCommandPayload format from ../TX_RX/ ESP32 project.

Format matches protocol.h DirectCommandPayload:
  struct DirectCommandPayload {
    float roll;       // -1.0 to 1.0
    float pitch;      // -1.0 to 1.0
    float yaw;        // -1.0 to 1.0
    float throttle;   // 0.0 to 1.0
    uint32_t timestamp;
  };

Total: 4 floats + 1 uint32 = 20 bytes

Note: Your TX firmware needs to be modified to accept UDP/Serial input.
Add a UDP server or Serial parser that receives these packets and calls
CustomProtocol_SendDirectCommand(roll, pitch, yaw, throttle).
"""

import struct
import time
from .base import PacketPacker


class TXRXPacker(PacketPacker):
    """Packer for TX_RX ESP32 project DirectCommandPayload format."""

    def __init__(self):
        # Format: 4 floats + 1 uint32 (little-endian)
        # <ffff = roll, pitch, yaw, throttle (floats)
        # I = timestamp (uint32)
        self.format = '<ffffI'
        self.packet_size = struct.calcsize(self.format)  # 20 bytes

    def encode(self, rc_msg):
        """
        Encode VirtualRC to TX_RX DirectCommandPayload format.

        Args:
            rc_msg: VirtualRC message with roll, pitch, yaw, throttle

        Returns:
            bytes: 20-byte packet
        """
        # Get timestamp in milliseconds
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        
        return struct.pack(self.format,
                          rc_msg.roll,
                          rc_msg.pitch,
                          rc_msg.yaw,
                          rc_msg.throttle,
                          timestamp)

    def safe_idle(self):
        """
        Generate safe idle packet.

        Returns:
            bytes: Packet with zero throttle and centered controls
        """
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        
        return struct.pack(self.format,
                          0.0,  # roll centered
                          0.0,  # pitch centered
                          0.0,  # yaw centered
                          0.0,  # throttle zero
                          timestamp)

