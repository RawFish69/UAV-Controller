"""
Abstract base class for packet packers.

Custom packers should inherit from PacketPacker and implement
encode() and safe_idle() methods to match TX_RX firmware format.
"""

from abc import ABC, abstractmethod


class PacketPacker(ABC):
    """Abstract packet packer interface."""

    @abstractmethod
    def encode(self, rc_msg):
        """
        Encode VirtualRC message to bytes for transmission.

        Args:
            rc_msg: VirtualRC ROS message

        Returns:
            bytes: Encoded packet ready for UDP/Serial transmission
        """
        pass

    @abstractmethod
    def safe_idle(self):
        """
        Generate safe idle packet (zero throttle, disarmed).

        Returns:
            bytes: Safe idle packet
        """
        pass

