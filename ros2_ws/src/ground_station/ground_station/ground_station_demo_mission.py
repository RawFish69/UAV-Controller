import time
from typing import Optional

import rclpy
from drone_msgs.msg import Command, MissionStatus, Telemetry, Trajectory, Waypoint
from drone_msgs.srv import PlanPath
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


PLANNING_MODE_MAP = {
    'offboard': Command.PLANNING_OFFBOARD,
    'onboard': Command.PLANNING_ONBOARD,
}


class GroundStationDemoMission(Node):
    """Basic demo mission: takeoff -> hover -> path -> land."""

    def __init__(self) -> None:
        super().__init__('ground_station_demo_mission')
        self.declare_parameter('command_topic', '/uav/command')
        self.declare_parameter('mission_topic', '/uav/mission')
        self.declare_parameter('telemetry_topic', '/uav/telemetry')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('planner_service_topic', '/gs/planner/plan_path')
        self.declare_parameter('planning_mode', 'offboard')  # offboard | onboard
        self.declare_parameter('use_planner', True)
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('terrain_profile', 'forest')
        self.declare_parameter('planner_collision_inflation_m', 1.5)
        self.declare_parameter('planner_failure_policy', 'fallback_planner')
        self.declare_parameter('planner_fallback_order', ['astar', 'rrt'])
        self.declare_parameter('allow_handcrafted_fallback', True)
        self.declare_parameter('planner_call_timeout_sec', 30.0)
        self.declare_parameter('max_total_planning_sec', 12.0)
        self.declare_parameter('mission_goal_xyz', [170.0, 120.0, 3.0])
        self.declare_parameter('mission_via1_xyz', [20.0, 10.0, 3.0])
        self.declare_parameter('mission_via2_xyz', [70.0, 40.0, 3.0])
        self.declare_parameter('loop_rate_hz', 5.0)
        self.declare_parameter('takeoff_wait_sec', 4.0)
        self.declare_parameter('hover_wait_sec', 2.0)
        self.declare_parameter('pre_mission_hover_sec', 2.0)
        self.declare_parameter('landing_wait_sec', 8.0)
        self.declare_parameter('mission_timeout_sec', 240.0)
        self.declare_parameter('wait_for_telemetry_sec', 30.0)
        self.declare_parameter('terrain_marker_topic', '/terrain/obstacles')
        self.declare_parameter('wait_for_terrain_sec', 45.0)
        self.declare_parameter('min_terrain_markers', 80)
        self.declare_parameter('path_marker_topic', '/gs/planner/planned_path_markers')
        self.declare_parameter('wait_for_path_markers_sec', 30.0)
        self.declare_parameter('min_path_markers', 3)
        self.declare_parameter('require_path_markers_before_takeoff', True)

        self.command_topic = self.get_parameter('command_topic').value
        self.mission_topic = self.get_parameter('mission_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.planner_service_topic = self.get_parameter('planner_service_topic').value
        self.planning_mode_name = str(self.get_parameter('planning_mode').value).lower()
        self.use_planner = bool(self.get_parameter('use_planner').value)
        self.planner_type = self.get_parameter('planner_type').value
        self.terrain_profile = self.get_parameter('terrain_profile').value
        self.planner_collision_inflation_m = float(
            self.get_parameter('planner_collision_inflation_m').value
        )
        self.planner_failure_policy = str(
            self.get_parameter('planner_failure_policy').value
        ).strip().lower()
        self.planner_fallback_order = [
            str(v).strip().lower()
            for v in list(self.get_parameter('planner_fallback_order').value)
            if str(v).strip()
        ]
        self.allow_handcrafted_fallback = bool(
            self.get_parameter('allow_handcrafted_fallback').value
        )
        self.planner_call_timeout_sec = float(self.get_parameter('planner_call_timeout_sec').value)
        self.max_total_planning_sec = float(self.get_parameter('max_total_planning_sec').value)
        self.mission_goal_xyz = list(self.get_parameter('mission_goal_xyz').value)
        self.mission_via1_xyz = list(self.get_parameter('mission_via1_xyz').value)
        self.mission_via2_xyz = list(self.get_parameter('mission_via2_xyz').value)
        self.loop_rate_hz = float(self.get_parameter('loop_rate_hz').value)
        self.takeoff_wait_sec = float(self.get_parameter('takeoff_wait_sec').value)
        self.hover_wait_sec = float(self.get_parameter('hover_wait_sec').value)
        self.pre_mission_hover_sec = float(self.get_parameter('pre_mission_hover_sec').value)
        self.landing_wait_sec = float(self.get_parameter('landing_wait_sec').value)
        self.mission_timeout_sec = float(self.get_parameter('mission_timeout_sec').value)
        self.wait_for_telemetry_sec = float(self.get_parameter('wait_for_telemetry_sec').value)
        self.terrain_marker_topic = str(self.get_parameter('terrain_marker_topic').value)
        self.wait_for_terrain_sec = float(self.get_parameter('wait_for_terrain_sec').value)
        self.min_terrain_markers = int(self.get_parameter('min_terrain_markers').value)
        self.path_marker_topic = str(self.get_parameter('path_marker_topic').value)
        self.wait_for_path_markers_sec = float(self.get_parameter('wait_for_path_markers_sec').value)
        self.min_path_markers = int(self.get_parameter('min_path_markers').value)
        self.require_path_markers_before_takeoff = bool(
            self.get_parameter('require_path_markers_before_takeoff').value
        )
        self.planning_mode = PLANNING_MODE_MAP.get(self.planning_mode_name, Command.PLANNING_OFFBOARD)

        self.pub_cmd = self.create_publisher(Command, self.command_topic, 10)
        self.pub_mission = self.create_publisher(Trajectory, self.mission_topic, 10)
        self.create_subscription(MissionStatus, self.mission_status_topic, self._on_mission_status, 20)
        self.create_subscription(Telemetry, self.telemetry_topic, self._on_telemetry, 20)
        self.create_subscription(MarkerArray, self.terrain_marker_topic, self._on_terrain_markers, 10)
        self.create_subscription(MarkerArray, self.path_marker_topic, self._on_path_markers, 10)
        self.last_mission_status: Optional[MissionStatus] = None
        self.last_telemetry: Optional[Telemetry] = None
        self.last_terrain_marker_count = 0
        self.last_path_marker_count = 0

        self.planner_client = self.create_client(PlanPath, self.planner_service_topic)
        self.sequence_id = 100

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self.last_mission_status = msg

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.last_telemetry = msg

    def _on_terrain_markers(self, msg: MarkerArray) -> None:
        count = 0
        for m in msg.markers:
            if int(m.action) == Marker.DELETEALL:
                continue
            if int(m.action) == Marker.ADD:
                count += 1
        self.last_terrain_marker_count = count

    def _on_path_markers(self, msg: MarkerArray) -> None:
        count = 0
        for m in msg.markers:
            if int(m.action) == Marker.DELETEALL:
                continue
            if int(m.action) == Marker.ADD:
                count += 1
        self.last_path_marker_count = count

    def _publish_cmd(self, mode: int, arm: bool = False, disarm: bool = False) -> None:
        self.sequence_id += 1
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence_id = self.sequence_id
        msg.mode_request = mode
        msg.planning_mode = self.planning_mode
        msg.arm = arm
        msg.disarm = disarm
        msg.manual_override = False
        msg.source_id = 'ground_station_demo_mission'
        self.pub_cmd.publish(msg)

    def _build_handcrafted_trajectory(self) -> Trajectory:
        traj = Trajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'map'
        traj.frame_id = 'map'
        traj.sequence_id = 42

        q = Quaternion()
        q.w = 1.0
        via1 = tuple(float(v) for v in self.mission_via1_xyz[:3])
        via2 = tuple(float(v) for v in self.mission_via2_xyz[:3])
        goal = tuple(float(v) for v in self.mission_goal_xyz[:3])
        points = [via1, via2, goal]
        for i, (x, y, z) in enumerate(points):
            wp = Waypoint()
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z
            wp.pose.orientation = q
            wp.hold_time_sec = 1.0 if i < len(points) - 1 else 0.5
            wp.acceptance_radius_m = 0.5
            wp.desired_speed_mps = 1.0
            traj.waypoints.append(wp)
        return traj

    def _wait_for_planner(self, timeout_sec: float = 5.0) -> bool:
        end = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end:
            if self.planner_client.wait_for_service(timeout_sec=0.2):
                return True
        return False

    def _plan_offboard_trajectory(self, goal_wp: Waypoint) -> Optional[Trajectory]:
        if self.last_telemetry is None:
            self.get_logger().warn('No telemetry yet; cannot call offboard planner.')
            return None
        if not self._wait_for_planner():
            self.get_logger().warn('Planner service unavailable.')
            return None

        planner_candidates = []
        requested = str(self.planner_type).strip().lower()
        if requested:
            planner_candidates.append(requested)

        if self.planner_failure_policy == 'fallback_planner':
            fallback_order = self.planner_fallback_order or ['astar', 'rrt']
        elif self.planner_failure_policy == 'handcrafted':
            fallback_order = []
        else:
            fallback_order = []

        for p in fallback_order:
            p_norm = p.strip().lower()
            if p_norm and p_norm not in planner_candidates:
                planner_candidates.append(p_norm)

        requested_inflation = float(self.planner_collision_inflation_m)
        # Retry with progressively less conservative but still safe-ish inflation if dense forests
        # cause all planners to reject outputs. This keeps obstacle checks enabled.
        inflation_candidates = []
        for v in (requested_inflation, requested_inflation * 0.85, 1.5):
            vv = max(1.0, float(v))
            if all(abs(vv - existing) > 1e-6 for existing in inflation_candidates):
                inflation_candidates.append(vv)

        planning_start = time.time()
        for inflation_m in inflation_candidates:
            if (time.time() - planning_start) >= self.max_total_planning_sec:
                self.get_logger().warn(
                    f'Planner total time cap ({self.max_total_planning_sec:.0f}s) reached; using handcrafted.'
                )
                break
            for planner_type in planner_candidates:
                if (time.time() - planning_start) >= self.max_total_planning_sec:
                    break
                req = PlanPath.Request()
                req.start = self.last_telemetry.pose
                req.goal = goal_wp.pose
                req.planner_type = planner_type
                req.terrain_profile = str(self.terrain_profile)
                req.collision_inflation_m = float(inflation_m)

                self.get_logger().info(
                    f"Requesting planner type='{planner_type}' terrain='{self.terrain_profile}' "
                    f"inflation={inflation_m:.2f}"
                )
                future = self.planner_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=self.planner_call_timeout_sec)
                if not future.done() or future.result() is None:
                    self.get_logger().warn(
                        f"Planner '{planner_type}' timed out/failed after "
                        f"{self.planner_call_timeout_sec:.1f}s (inflation={inflation_m:.2f})"
                    )
                    continue

                result = future.result()
                if not result.success:
                    self.get_logger().warn(
                        f"Planner '{planner_type}' failed (inflation={inflation_m:.2f}): {result.message}"
                    )
                    continue

                traj = result.trajectory
                if traj.sequence_id == 0:
                    traj.sequence_id = 42
                self.get_logger().info(
                    f"Offboard planner generated {len(traj.waypoints)} waypoints using "
                    f"{planner_type} (inflation={inflation_m:.2f})"
                )
                return traj

        self.get_logger().warn('All planner attempts failed.')
        return None

    def _publish_demo_trajectory(self) -> bool:
        base_traj = self._build_handcrafted_trajectory()

        if self.planning_mode == Command.PLANNING_OFFBOARD and self.use_planner:
            planned = self._plan_offboard_trajectory(base_traj.waypoints[-1])
            if planned is not None:
                traj = planned
            elif self.planner_failure_policy in {'abort', 'hover'} and not self.allow_handcrafted_fallback:
                self.get_logger().error(
                    f"Planner failed and planner_failure_policy='{self.planner_failure_policy}' "
                    "with handcrafted fallback disabled; not publishing a mission."
                )
                return False
            elif self.allow_handcrafted_fallback or self.planner_failure_policy == 'handcrafted':
                self.get_logger().warn('Using handcrafted fallback mission trajectory.')
                traj = base_traj
            else:
                self.get_logger().error('Planner failed and no fallback path is allowed.')
                return False
        elif self.planning_mode == Command.PLANNING_ONBOARD:
            # Goal-only trajectory convention: air-side mission executor calls local planner.
            traj = Trajectory()
            traj.header = base_traj.header
            traj.frame_id = base_traj.frame_id
            traj.sequence_id = base_traj.sequence_id
            traj.waypoints.append(base_traj.waypoints[-1])
        else:
            traj = base_traj

        self.pub_mission.publish(traj)
        self.get_logger().info(
            f"Published mission ({len(traj.waypoints)} waypoints, planning_mode={self.planning_mode_name})"
        )
        return True

    def _loop_for(self, duration_sec: float, mode: int, arm: bool = False, disarm: bool = False) -> None:
        end = time.time() + duration_sec
        dt = 1.0 / max(self.loop_rate_hz, 1.0)
        while rclpy.ok() and time.time() < end:
            self._publish_cmd(mode, arm=arm, disarm=disarm)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(dt)

    def _wait_for_telemetry(self) -> bool:
        """Block until the first telemetry message arrives, confirming the bridge and drone are up."""
        timeout = self.wait_for_telemetry_sec
        self.get_logger().info(
            f'Waiting for first telemetry (drone + bridge ready check, up to {timeout:.0f}s)...'
        )
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_telemetry is not None:
                self.get_logger().info('Telemetry received – drone and bridge are up, starting demo.')
                return True
        self.get_logger().warn(
            f'No telemetry after {timeout:.0f}s – bridge or drone may not be ready. Proceeding anyway.'
        )
        return False

    def _wait_for_terrain(self) -> bool:
        """Wait until terrain generator has published a sufficiently dense obstacle set."""
        timeout = self.wait_for_terrain_sec
        needed = max(0, self.min_terrain_markers)
        if needed == 0:
            return True
        self.get_logger().info(
            f'Waiting for terrain readiness on {self.terrain_marker_topic} '
            f'(need >= {needed} markers, timeout {timeout:.0f}s)...'
        )
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_terrain_marker_count >= needed:
                self.get_logger().info(
                    f'Terrain ready ({self.last_terrain_marker_count} markers). Starting mission flow.'
                )
                return True
        self.get_logger().warn(
            f'Terrain readiness timeout: got {self.last_terrain_marker_count} markers, '
            f'expected >= {needed}. Proceeding anyway.'
        )
        return False

    def _wait_for_path_markers(self) -> bool:
        timeout = self.wait_for_path_markers_sec
        needed = max(0, self.min_path_markers)
        if needed == 0:
            return True
        self.get_logger().info(
            f'Waiting for planner path markers on {self.path_marker_topic} '
            f'(need >= {needed}, timeout {timeout:.0f}s)...'
        )
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_path_marker_count >= needed:
                self.get_logger().info(
                    f'Path markers ready ({self.last_path_marker_count}).'
                )
                return True
        self.get_logger().warn(
            f'Path marker wait timeout: got {self.last_path_marker_count}, expected >= {needed}.'
        )
        return False

    def run(self) -> None:
        # Wait until the drone and ROS↔Gazebo bridge are confirmed up before issuing any commands.
        # This prevents the demo from racing ahead before Gazebo finishes loading.
        self._wait_for_telemetry()
        # For planner demos, also wait for terrain to be published so takeoff/plan timing is sane.
        self._wait_for_terrain()

        # Send initial command mode so the air unit configures its planning mode
        # before the trajectory request arrives (fixes onboard planner race condition).
        self._publish_cmd(Command.MODE_IDLE, arm=False)

        self.get_logger().info('Demo: planning trajectory...')
        mission_published = self._publish_demo_trajectory()
        if not mission_published:
            self.get_logger().error(
                'Demo: no trajectory available after planning/fallback policy. '
                'Aborting before arm/takeoff.'
            )
            return

        if (
            self.require_path_markers_before_takeoff
            and self.planning_mode == Command.PLANNING_OFFBOARD
            and self.use_planner
        ):
            if not self._wait_for_path_markers():
                self.get_logger().warn(
                    'Planner path markers did not appear in time; continuing because a planned trajectory exists.'
                )

        self.get_logger().info(f'Demo: arm + takeoff (planning_mode={self.planning_mode_name})')
        self._loop_for(self.takeoff_wait_sec, Command.MODE_TAKEOFF, arm=True)

        self.get_logger().info('Demo: hover')
        self._loop_for(self.hover_wait_sec, Command.MODE_HOVER, arm=True)

        # Brief hover after trajectory is published so the drone is stable before it starts moving.
        if self.pre_mission_hover_sec > 0:
            self.get_logger().info(
                f'Demo: holding hover {self.pre_mission_hover_sec:.1f}s before mission starts...'
            )
            self._loop_for(self.pre_mission_hover_sec, Command.MODE_HOVER, arm=True)

        self.get_logger().info('Demo: mission – flying to goal')
        end = time.time() + self.mission_timeout_sec
        dt = 1.0 / max(self.loop_rate_hz, 1.0)
        while mission_published and rclpy.ok() and time.time() < end:
            self._publish_cmd(Command.MODE_MISSION, arm=True)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_mission_status and self.last_mission_status.complete:
                break
            time.sleep(dt)

        self.get_logger().info('Demo: land')
        self._loop_for(self.landing_wait_sec, Command.MODE_LAND, arm=True)
        self.get_logger().info('Demo: disarm')
        self._loop_for(0.5, Command.MODE_IDLE, disarm=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationDemoMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
