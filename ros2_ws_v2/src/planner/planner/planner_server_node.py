import rclpy
from drone_msgs.msg import Trajectory, Waypoint
from drone_msgs.srv import PlanPath
from geometry_msgs.msg import Quaternion
from rclpy.node import Node

from uav_algorithms import plan_trajectory_points


class PlannerServerNode(Node):
    def __init__(self) -> None:
        super().__init__('planner_server_node')
        self.declare_parameter('default_planner_type', 'rrtstar')
        self.declare_parameter('default_terrain_profile', 'plains')
        self.declare_parameter('service_name', 'plan_path')

        self.default_planner_type = self.get_parameter('default_planner_type').value
        self.default_terrain_profile = self.get_parameter('default_terrain_profile').value
        service_name = self.get_parameter('service_name').value

        self.srv = self.create_service(PlanPath, service_name, self._handle_plan_path)
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
            wp.acceptance_radius_m = 0.5
            wp.desired_speed_mps = 1.0
            traj.waypoints.append(wp)

        response.success = True
        response.message = f'planned {len(traj.waypoints)} waypoints using {planner_type}/{terrain_profile}'
        response.trajectory = traj
        self.get_logger().info(response.message)
        return response


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
