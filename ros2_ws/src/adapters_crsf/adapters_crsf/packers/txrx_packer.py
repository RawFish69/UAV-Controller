"""
TX_RX packer: matches the format of the ../TX_RX/ ESP32 project.

Default format: little-endian <ffff12f
- 4 floats: roll, pitch, yaw, throttle
- 12 floats: aux[0..11]

Total: 16 floats * 4 bytes = 64 bytes
"""

import struct
from .base import PacketPacker


class TXRXPacker(PacketPacker):
    """Packer for TX_RX ESP32 project format."""

    def __init__(self):
        # Format: 16 floats (roll, pitch, yaw, throttle, aux[0..11])
        self.format = '<16f'
        self.packet_size = struct.calcsize(self.format)

    def encode(self, rc_msg):
        """
        Encode VirtualRC to TX_RX format.

        Args:
            rc_msg: VirtualRC message with roll, pitch, yaw, throttle, aux[12]

        Returns:
            bytes: 64-byte packet
        """
        # Pack: roll, pitch, yaw, throttle, aux[0..11]
        data = [
            rc_msg.roll,
            rc_msg.pitch,
            rc_msg.yaw,
            rc_msg.throttle,
        ]
        
        # Add aux channels (ensure we have 12)
        aux = list(rc_msg.aux) if hasattr(rc_msg, 'aux') else [0.0] * 12
        while len(aux) < 12:
            aux.append(0.0)
        data.extend(aux[:12])

        return struct.pack(self.format, *data)

    def safe_idle(self):
        """
        Generate safe idle packet.

        Returns:
            bytes: Packet with zero throttle and centered controls
        """
        data = [
            0.0,  # roll
            0.0,  # pitch
            0.0,  # yaw
            0.0,  # throttle (zero for safety)
        ]
        data.extend([0.0] * 12)  # aux channels (disarmed)

        return struct.pack(self.format, *data)

