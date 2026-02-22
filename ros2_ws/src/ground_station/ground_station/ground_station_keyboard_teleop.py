import argparse
import sys
from typing import Optional

import rclpy
from drone_msgs.msg import Command
from geometry_msgs.msg import Twist
from rclpy.node import Node


class _KeyboardReader:
    """Cross-platform non-blocking single-char keyboard reader."""

    def __init__(self) -> None:
        self._is_windows = sys.platform.startswith("win")
        self._enabled = False
        self._fd = None
        self._old_term = None
        self._msvcrt = None
        self._select = None

    def __enter__(self) -> "_KeyboardReader":
        if self._is_windows:
            try:
                import msvcrt  # type: ignore

                self._msvcrt = msvcrt
                self._enabled = True
            except Exception:
                self._enabled = False
            return self

        try:
            import select  # type: ignore
            import termios  # type: ignore
            import tty  # type: ignore

            if not sys.stdin.isatty():
                self._enabled = False
                return self

            self._select = select
            self._fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._enabled = True
        except Exception:
            self._enabled = False
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._is_windows or not self._enabled or self._fd is None or self._old_term is None:
            return
        try:
            import termios  # type: ignore

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass

    @property
    def enabled(self) -> bool:
        return self._enabled

    def read_char(self) -> Optional[str]:
        if not self._enabled:
            return None

        if self._is_windows:
            assert self._msvcrt is not None
            if not self._msvcrt.kbhit():
                return None
            ch = self._msvcrt.getwch()
            # Swallow Windows special-key prefix and the follow-up code.
            if ch in ("\x00", "\xe0"):
                if self._msvcrt.kbhit():
                    _ = self._msvcrt.getwch()
                return None
            return ch

        assert self._select is not None
        ready, _, _ = self._select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        try:
            return sys.stdin.read(1)
        except Exception:
            return None


MODE_MAP = {
    "idle": Command.MODE_IDLE,
    "takeoff": Command.MODE_TAKEOFF,
    "hover": Command.MODE_HOVER,
    "mission": Command.MODE_MISSION,
    "land": Command.MODE_LAND,
    "rtl": Command.MODE_RTL,
    "manual": Command.MODE_MANUAL_OVERRIDE,
}

PLANNING_MAP = {
    "onboard": Command.PLANNING_ONBOARD,
    "offboard": Command.PLANNING_OFFBOARD,
}


class GroundStationKeyboardTeleopNode(Node):
    """Keyboard teleop publisher for /uav command messages."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ground_station_keyboard_teleop")
        self.args = args
        self.pub = self.create_publisher(Command, args.command_topic, 10)

        self.sequence_id = int(args.sequence_id)
        self.mode_request = MODE_MAP["manual"]
        self.planning_mode = PLANNING_MAP[args.planning_mode]
        self.manual_override = True
        self.arm_cmd_pending = False
        self.disarm_cmd_pending = False

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.scale = 1.0

        self._reader_cm = _KeyboardReader()
        self._reader = self._reader_cm.__enter__()
        if not self._reader.enabled:
            self.get_logger().warning(
                "Keyboard input is not available (stdin is not a TTY?). Teleop will publish but not read keys."
            )

        period = 1.0 / max(float(args.rate_hz), 1.0)
        self.timer = self.create_timer(period, self._tick)

        self._print_help()
        self.get_logger().info(f"Publishing keyboard teleop commands to {args.command_topic}")

    def destroy_node(self) -> bool:
        try:
            self._reader_cm.__exit__(None, None, None)
        finally:
            pass
        return super().destroy_node()

    def _print_help(self) -> None:
        help_text = (
            "\nKeyboard Teleop (focus terminal)\n"
            "  Move: w/s=+/-vx, a/d=+/-vy, r/f=+/-vz, q/e=+/-yaw_rate\n"
            "  Reset: space=zero all, z=zero horizontal, x=zero vertical+yaw\n"
            "  Scale: +/- adjusts command increment scale\n"
            "  Modes: m=manual, h=hover, t=takeoff, g=land, i=idle\n"
            "  Arm: u=arm, j=disarm\n"
            "  Planning: o=offboard, p=onboard\n"
            "  Toggle: v=manual_override on/off\n"
            "  Quit: ESC or Ctrl-C\n"
        )
        print(help_text, flush=True)

    def _clip(self) -> None:
        self.vx = max(-self.args.max_vxy, min(self.args.max_vxy, self.vx))
        self.vy = max(-self.args.max_vxy, min(self.args.max_vxy, self.vy))
        self.vz = max(-self.args.max_vz, min(self.args.max_vz, self.vz))
        self.yaw_rate = max(-self.args.max_yaw_rate, min(self.args.max_yaw_rate, self.yaw_rate))
        self.scale = max(0.1, min(5.0, self.scale))

    def _set_mode(self, name: str) -> None:
        self.mode_request = MODE_MAP[name]
        self.get_logger().info(f"Mode -> {name}")

    def _process_key(self, ch: str) -> None:
        if not ch:
            return

        if ch in ("\x03",):  # Ctrl-C
            raise KeyboardInterrupt
        if ch in ("\x1b",):  # ESC
            raise KeyboardInterrupt

        k = ch.lower()
        dv_xy = float(self.args.vxy_step) * self.scale
        dv_z = float(self.args.vz_step) * self.scale
        dyaw = float(self.args.yaw_step) * self.scale

        if k == "w":
            self.vx += dv_xy
        elif k == "s":
            self.vx -= dv_xy
        elif k == "d":
            self.vy += dv_xy
        elif k == "a":
            self.vy -= dv_xy
        elif k == "r":
            self.vz += dv_z
        elif k == "f":
            self.vz -= dv_z
        elif k == "q":
            self.yaw_rate += dyaw
        elif k == "e":
            self.yaw_rate -= dyaw
        elif k == " ":
            self.vx = self.vy = self.vz = self.yaw_rate = 0.0
        elif k == "z":
            self.vx = self.vy = 0.0
        elif k == "x":
            self.vz = self.yaw_rate = 0.0
        elif k in {"+", "="}:
            self.scale *= 1.25
        elif k == "-":
            self.scale /= 1.25
        elif k == "u":
            self.arm_cmd_pending = True
            self.disarm_cmd_pending = False
            self.get_logger().info("Arm command queued")
        elif k == "j":
            self.disarm_cmd_pending = True
            self.arm_cmd_pending = False
            self.get_logger().info("Disarm command queued")
        elif k == "m":
            self._set_mode("manual")
        elif k == "h":
            self._set_mode("hover")
        elif k == "t":
            self._set_mode("takeoff")
        elif k == "g":
            self._set_mode("land")
        elif k == "i":
            self._set_mode("idle")
        elif k == "o":
            self.planning_mode = PLANNING_MAP["offboard"]
            self.get_logger().info("Planning mode -> offboard")
        elif k == "p":
            self.planning_mode = PLANNING_MAP["onboard"]
            self.get_logger().info("Planning mode -> onboard")
        elif k == "v":
            self.manual_override = not self.manual_override
            self.get_logger().info(f"manual_override -> {self.manual_override}")
        else:
            return

        self._clip()
        self._print_status()

    def _print_status(self) -> None:
        mode_name = next((k for k, v in MODE_MAP.items() if v == self.mode_request), str(self.mode_request))
        planning_name = (
            "onboard" if self.planning_mode == Command.PLANNING_ONBOARD else "offboard"
        )
        print(
            (
                f"[teleop] mode={mode_name} manual_override={self.manual_override} planning={planning_name} "
                f"vel=(x={self.vx:.2f}, y={self.vy:.2f}, z={self.vz:.2f}) yaw={self.yaw_rate:.2f} "
                f"scale={self.scale:.2f}"
            ),
            flush=True,
        )

    def _build_msg(self) -> Command:
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence_id = self.sequence_id
        self.sequence_id += 1

        msg.mode_request = int(self.mode_request)
        msg.planning_mode = int(self.planning_mode)
        msg.manual_override = bool(self.manual_override)

        msg.arm = bool(self.arm_cmd_pending)
        msg.disarm = bool(self.disarm_cmd_pending)
        self.arm_cmd_pending = False
        self.disarm_cmd_pending = False

        msg.manual.active = False
        msg.manual.roll = 0.0
        msg.manual.pitch = 0.0
        msg.manual.yaw = 0.0
        msg.manual.throttle = 0.5

        vel = Twist()
        vel.linear.x = float(self.vx)
        vel.linear.y = float(self.vy)
        vel.linear.z = float(self.vz)
        vel.angular.z = float(self.yaw_rate)
        msg.velocity_setpoint = vel
        msg.source_id = self.args.source_id
        return msg

    def _tick(self) -> None:
        if self._reader.enabled:
            while True:
                ch = self._reader.read_char()
                if ch is None:
                    break
                self._process_key(ch)

        self.pub.publish(self._build_msg())


def _parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ground station keyboard teleop publisher")
    parser.add_argument("--command-topic", default="/uav/command")
    parser.add_argument("--planning-mode", choices=sorted(PLANNING_MAP), default="offboard")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--vxy-step", type=float, default=0.2)
    parser.add_argument("--vz-step", type=float, default=0.2)
    parser.add_argument("--yaw-step", type=float, default=0.15)
    parser.add_argument("--max-vxy", type=float, default=2.0)
    parser.add_argument("--max-vz", type=float, default=1.5)
    parser.add_argument("--max-yaw-rate", type=float, default=1.5)
    parser.add_argument("--sequence-id", type=int, default=1)
    parser.add_argument("--source-id", default="ground_station_keyboard_teleop")
    return parser.parse_args(argv)


def main(args=None) -> None:
    rclpy.init(args=args)
    cli_args = _parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = GroundStationKeyboardTeleopNode(cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
