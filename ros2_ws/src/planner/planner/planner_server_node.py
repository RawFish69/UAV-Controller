import rclpy
from drone_msgs.msg import MissionStatus, Trajectory, Waypoint
from drone_msgs.srv import PlanPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from uav_algorithms import plan_trajectory_points


class PlannerServerNode(Node):
    def __init__(self) -> None:
        super().__init__('planner_server_node')
        self.declare_parameter('default_planner_type', 'astar')
        self.declare_parameter('default_terrain_profile', 'plains')
        self.declare_parameter('service_name', 'plan_path')
        self.declare_parameter('path_marker_topic', 'planned_path_markers')
        self.declare_parameter('mission_status_topic', '/uav/mission_status')
        self.declare_parameter('default_waypoint_speed_mps', 2.5)
        self.declare_parameter('default_acceptance_radius_m', 1.0)

        self.default_planner_type = self.get_parameter('default_planner_type').value
        self.default_terrain_profile = self.get_parameter('default_terrain_profile').value
        service_name = self.get_parameter('service_name').value
        path_marker_topic = self.get_parameter('path_marker_topic').value
        mission_status_topic = self.get_parameter('mission_status_topic').value
        self.default_waypoint_speed_mps = float(self.get_parameter('default_waypoint_speed_mps').value)
        self.default_acceptance_radius_m = float(self.get_parameter('default_acceptance_radius_m').value)
        self._last_points: list[list[float]] = []
        self._last_total_waypoints = 0

        self.srv = self.create_service(PlanPath, service_name, self._handle_plan_path)
        self.pub_path_markers = self.create_publisher(MarkerArray, path_marker_topic, 10)
        self.create_subscription(MissionStatus, mission_status_topic, self._on_mission_status, 20)
        self.get_logger().info(f'Planner service ready: {self.get_namespace()}/{service_name}')

    def _handle_plan_path(self, request: PlanPath.Request, response: PlanPath.Response) -> PlanPath.Response:
        planner_type = request.planner_type or self.default_planner_type
        terrain_profile = request.terrain_profile or self.default_terrain_profile
        start = [request.start.position.x, request.start.position.y, request.start.position.z]
        goal = [request.goal.position.x, request.goal.position.y, request.goal.position.z]

        try:
            points = plan_trajectory_points(
                start_xyz=start,
                goal_xyz=goal,
                planner_type=planner_type,
                terrain_profile=terrain_profile,
                collision_inflation_m=float(request.collision_inflation_m),
            )
        except Exception as exc:
            response.success = False
            response.message = f'planning failed: {exc}'
            self.get_logger().error(response.message)
            return response

        q = Quaternion()
        q.w = 1.0
        traj = Trajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'map'
        traj.frame_id = 'map'
        traj.sequence_id = 1

        for p in points:
            wp = Waypoint()
            wp.pose.position.x = float(p[0])
            wp.pose.position.y = float(p[1])
            wp.pose.position.z = float(p[2])
            wp.pose.orientation = q
            wp.hold_time_sec = 0.0
            wp.acceptance_radius_m = max(0.2, self.default_acceptance_radius_m)
            wp.desired_speed_mps = max(0.1, self.default_waypoint_speed_mps)
            traj.waypoints.append(wp)

        response.success = True
        response.message = f'planned {len(traj.waypoints)} waypoints using {planner_type}/{terrain_profile}'
        response.trajectory = traj
        self._last_points = [list(map(float, p)) for p in points]
        self._last_total_waypoints = len(points)
        self._publish_path_markers(self._last_points, active_waypoint_index=0, complete=False)
        self.get_logger().info(response.message)
        return response

    def _on_mission_status(self, msg: MissionStatus) -> None:
        if not self._last_points:
            return
        if int(msg.total_waypoints) > 0 and int(msg.total_waypoints) != self._last_total_waypoints:
            return
        self._publish_path_markers(
            self._last_points,
            active_waypoint_index=int(msg.active_waypoint_index),
            complete=bool(msg.complete),
        )

    def _publish_path_markers(
        self,
        points: list[list[float]] | list,
        active_waypoint_index: int = 0,
        complete: bool = False,
    ) -> None:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        header = Header(frame_id='map', stamp=stamp)

        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if not points:
            self.pub_path_markers.publish(marker_array)
            return

        n = len(points)
        if complete:
            active_idx = n
        else:
            active_idx = max(0, min(int(active_waypoint_index), n - 1))

        ros_points: list[Point] = []
        for p in points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            ros_points.append(pt)

        # "Glow" effect in RViz: wide translucent line behind the core path.
        def _make_line(marker_id: int, ns: str, pts: list[Point], width: float, color: ColorRGBA) -> Marker:
            m = Marker()
            m.header = header
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color = color
            m.points = pts
            return m

        completed_pts = ros_points[: max(0, min(active_idx + 1, n))]
        remaining_pts = ros_points[max(0, min(active_idx, n - 1)) :] if n >= 2 else ros_points

        if len(completed_pts) >= 2:
            marker_array.markers.append(
                _make_line(
                    0, 'planned_path_glow_done', completed_pts, 0.95,
                    ColorRGBA(r=0.10, g=0.95, b=0.65, a=0.25),
                )
            )
            marker_array.markers.append(
                _make_line(
                    1, 'planned_path_done', completed_pts, 0.35,
                    ColorRGBA(r=0.05, g=0.95, b=0.55, a=0.95),
                )
            )

        if len(remaining_pts) >= 2:
            marker_array.markers.append(
                _make_line(
                    2, 'planned_path_glow_remaining', remaining_pts, 0.95,
                    ColorRGBA(r=1.00, g=0.75, b=0.05, a=0.22),
                )
            )
            marker_array.markers.append(
                _make_line(
                    3, 'planned_path_remaining', remaining_pts, 0.35,
                    ColorRGBA(r=1.00, g=0.55, b=0.05, a=0.98),
                )
            )

        for i, p in enumerate(points):
            dot = Marker()
            dot.header = header
            dot.ns = 'planned_waypoints'
            dot.id = i + 1
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = float(p[0])
            dot.pose.position.y = float(p[1])
            dot.pose.position.z = float(p[2])
            dot.pose.orientation.w = 1.0
            dot.scale.x = 0.55
            dot.scale.y = 0.55
            dot.scale.z = 0.55
            if i == 0:
                dot.color = ColorRGBA(r=0.15, g=0.9, b=0.2, a=0.95)
            elif i == len(points) - 1:
                dot.color = ColorRGBA(r=0.95, g=0.1, b=0.15, a=0.95)
            elif i < active_idx:
                dot.color = ColorRGBA(r=0.05, g=0.85, b=0.75, a=0.9)
            elif i == active_idx and not complete:
                dot.scale.x = 0.75
                dot.scale.y = 0.75
                dot.scale.z = 0.75
                dot.color = ColorRGBA(r=1.0, g=1.0, b=0.9, a=1.0)
            else:
                dot.color = ColorRGBA(r=1.0, g=0.7, b=0.1, a=0.85)
            marker_array.markers.append(dot)

        self.pub_path_markers.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
