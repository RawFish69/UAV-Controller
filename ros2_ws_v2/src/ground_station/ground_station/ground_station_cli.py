import argparse
import time

import rclpy
from drone_msgs.msg import Command
from geometry_msgs.msg import Twist
from rclpy.node import Node


MODE_MAP = {
    'idle': Command.MODE_IDLE,
    'takeoff': Command.MODE_TAKEOFF,
    'hover': Command.MODE_HOVER,
    'mission': Command.MODE_MISSION,
    'land': Command.MODE_LAND,
    'rtl': Command.MODE_RTL,
    'manual': Command.MODE_MANUAL_OVERRIDE,
}

PLANNING_MAP = {
    'onboard': Command.PLANNING_ONBOARD,
    'offboard': Command.PLANNING_OFFBOARD,
}


class GroundStationCliNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('ground_station_cli')
        self.args = args
        self.pub = self.create_publisher(Command, args.command_topic, 10)
        self.sequence_id = int(args.sequence_id)
        self.get_logger().info(f'Publishing commands to {args.command_topic}')

    def build_msg(self) -> Command:
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence_id = self.sequence_id
        msg.mode_request = MODE_MAP[self.args.mode]
        msg.planning_mode = PLANNING_MAP[self.args.planning_mode]
        msg.arm = bool(self.args.arm)
        msg.disarm = bool(self.args.disarm)
        msg.manual_override = bool(self.args.manual_override)
        msg.source_id = self.args.source_id

        msg.manual.active = any(
            v is not None for v in (
                self.args.manual_roll, self.args.manual_pitch,
                self.args.manual_yaw, self.args.manual_throttle,
            )
        )
        msg.manual.roll = float(self.args.manual_roll or 0.0)
        msg.manual.pitch = float(self.args.manual_pitch or 0.0)
        msg.manual.yaw = float(self.args.manual_yaw or 0.0)
        msg.manual.throttle = float(self.args.manual_throttle if self.args.manual_throttle is not None else 0.5)

        vel = Twist()
        vel.linear.x = float(self.args.vx)
        vel.linear.y = float(self.args.vy)
        vel.linear.z = float(self.args.vz)
        vel.angular.z = float(self.args.yaw_rate)
        msg.velocity_setpoint = vel
        return msg

    def run(self) -> None:
        period = 1.0 / max(float(self.args.rate_hz), 1.0)
        end_time = time.time() + float(self.args.duration_sec)
        while rclpy.ok() and time.time() <= end_time:
            msg = self.build_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.get_logger().info('Command publishing finished')


def _parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Ground station CLI command publisher')
    parser.add_argument('--command-topic', default='/uav1/command')
    parser.add_argument('--mode', choices=sorted(MODE_MAP), default='hover')
    parser.add_argument('--planning-mode', choices=sorted(PLANNING_MAP), default='offboard')
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--disarm', action='store_true')
    parser.add_argument('--manual-override', action='store_true')
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--vz', type=float, default=0.0)
    parser.add_argument('--yaw-rate', type=float, default=0.0)
    parser.add_argument('--manual-roll', type=float, default=None)
    parser.add_argument('--manual-pitch', type=float, default=None)
    parser.add_argument('--manual-yaw', type=float, default=None)
    parser.add_argument('--manual-throttle', type=float, default=None)
    parser.add_argument('--duration-sec', type=float, default=1.0)
    parser.add_argument('--rate-hz', type=float, default=5.0)
    parser.add_argument('--sequence-id', type=int, default=1)
    parser.add_argument('--source-id', default='ground_station_cli')
    return parser.parse_args(argv)


def main(args=None) -> None:
    rclpy.init(args=args)
    cli_args = _parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = GroundStationCliNode(cli_args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
