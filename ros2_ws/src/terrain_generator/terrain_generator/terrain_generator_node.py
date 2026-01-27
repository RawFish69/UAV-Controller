"""ROS2 node for terrain generation."""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header
import numpy as np

from .forest_generator import generate_forest, generate_random_forest
from .mountains_generator import generate_mountains, generate_mountain_ridge
from .plains_generator import generate_plains, generate_empty_plains
from .obstacles import CylinderObstacle, BoxObstacle


class TerrainGeneratorNode(Node):
    """ROS2 node that generates and publishes terrain obstacles."""
    
    def __init__(self):
        super().__init__('terrain_generator')
        
        # Declare parameters
        self.declare_parameter('terrain_type', 'forest')  # forest, mountains, plains
        self.declare_parameter('space_dim', [100.0, 100.0, 50.0])  # [x, y, z]
        self.declare_parameter('start_pos', [0.0, 0.0, 0.0])
        
        # Forest parameters
        self.declare_parameter('forest.grid_size', 10)
        self.declare_parameter('forest.radius_range', [0.5, 1.5])
        self.declare_parameter('forest.height_range', [5.0, 15.0])
        self.declare_parameter('forest.density', 0.7)
        
        # Mountains parameters
        self.declare_parameter('mountains.num_peaks', 15)
        self.declare_parameter('mountains.base_size_range', [3.0, 8.0])
        self.declare_parameter('mountains.height_range', [10.0, 30.0])
        self.declare_parameter('mountains.grid_resolution', 10.0)
        self.declare_parameter('mountains.ridge_count', 2)
        self.declare_parameter('mountains.pit_count', 6)
        self.declare_parameter('mountains.pit_depth_range', [5.0, 20.0])
        self.declare_parameter('mountains.steepness', 1.6)
        self.declare_parameter('mountains.min_height', 0.5)
        
        # Plains parameters
        self.declare_parameter('plains.num_obstacles', 10)
        self.declare_parameter('plains.obstacle_types', ['bush', 'rock'])
        
        # Publisher
        self.pub_markers = self.create_publisher(MarkerArray, '/terrain/obstacles', 10)
        
        # Generate terrain
        self.obstacles = []
        self.terrain_type = ''
        self.generate_and_publish()
        
        # Republish terrain periodically (every 5 seconds) to ensure RViz displays it
        self.republish_timer = self.create_timer(5.0, self.republish_terrain)
        
        self.get_logger().info('Terrain generator node started')
    
    def generate_and_publish(self):
        """Generate terrain based on parameters and publish markers."""
        terrain_type = self.get_parameter('terrain_type').get_parameter_value().string_value
        space_dim = self.get_parameter('space_dim').get_parameter_value().double_array_value
        start_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value
        
        space_dim = np.array(space_dim)
        start_pos = np.array(start_pos)
        
        # Store for republishing
        self.terrain_type = terrain_type
        self.space_dim = space_dim
        self.start_pos = start_pos
        
        obstacles = []
        
        if terrain_type == 'forest':
            grid_size = self.get_parameter('forest.grid_size').get_parameter_value().integer_value
            radius_range = tuple(self.get_parameter('forest.radius_range').get_parameter_value().double_array_value)
            height_range = tuple(self.get_parameter('forest.height_range').get_parameter_value().double_array_value)
            density = self.get_parameter('forest.density').get_parameter_value().double_value
            
            obstacles = generate_forest(
                space_dim, grid_size=grid_size,
                radius_range=radius_range, height_range=height_range,
                density=density, start_pos=start_pos
            )
            self.get_logger().info(f'Generated forest with {len(obstacles)} trees')
        
        elif terrain_type == 'mountains':
            num_peaks = self.get_parameter('mountains.num_peaks').get_parameter_value().integer_value
            base_size_range = tuple(self.get_parameter('mountains.base_size_range').get_parameter_value().double_array_value)
            height_range = tuple(self.get_parameter('mountains.height_range').get_parameter_value().double_array_value)
            grid_resolution = self.get_parameter('mountains.grid_resolution').get_parameter_value().double_value
            ridge_count = self.get_parameter('mountains.ridge_count').get_parameter_value().integer_value
            pit_count = self.get_parameter('mountains.pit_count').get_parameter_value().integer_value
            pit_depth_range = tuple(self.get_parameter('mountains.pit_depth_range').get_parameter_value().double_array_value)
            steepness = self.get_parameter('mountains.steepness').get_parameter_value().double_value
            min_height = self.get_parameter('mountains.min_height').get_parameter_value().double_value
            
            obstacles = generate_mountains(
                space_dim, num_peaks=num_peaks,
                base_size_range=base_size_range, height_range=height_range,
                start_pos=start_pos,
                grid_resolution=grid_resolution,
                ridge_count=ridge_count,
                pit_count=pit_count,
                pit_depth_range=pit_depth_range,
                steepness=steepness,
                min_height=min_height,
            )
            self.get_logger().info(f'Generated mountains with {len(obstacles)} obstacles')
        
        elif terrain_type == 'plains':
            num_obstacles = self.get_parameter('plains.num_obstacles').get_parameter_value().integer_value
            obstacle_types = self.get_parameter('plains.obstacle_types').get_parameter_value().string_array_value
            
            obstacles = generate_plains(
                space_dim, num_obstacles=num_obstacles,
                obstacle_types=obstacle_types, start_pos=start_pos
            )
            self.get_logger().info(f'Generated plains with {len(obstacles)} obstacles')
        
        else:
            self.get_logger().warn(f'Unknown terrain type: {terrain_type}')
            obstacles = []
        
        # Store obstacles for republishing
        self.obstacles = obstacles
        
        # Publish markers
        self.publish_markers(obstacles, terrain_type)
    
    def publish_markers(self, obstacles, terrain_type):
        """Publish obstacle markers for visualization."""
        marker_array = MarkerArray()
        
        # Color based on terrain type
        if terrain_type == 'forest':
            color = ColorRGBA(r=0.0, g=0.5, b=0.0, a=0.8)  # Green
        elif terrain_type == 'mountains':
            color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)  # Gray
        else:  # plains
            color = ColorRGBA(r=0.8, g=0.6, b=0.2, a=0.8)  # Brown
        
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.ns = 'terrain_obstacles'
            
            if isinstance(obstacle, CylinderObstacle):
                marker.type = Marker.CYLINDER
                marker.pose.position.x = float(obstacle.center[0])
                marker.pose.position.y = float(obstacle.center[1])
                marker.pose.position.z = float(obstacle.center[2] + obstacle.height / 2.0)
                marker.pose.orientation.w = 1.0
                marker.scale = Vector3(
                    x=float(obstacle.radius * 2),
                    y=float(obstacle.radius * 2),
                    z=float(obstacle.height)
                )
                marker.color = color
            
            elif isinstance(obstacle, BoxObstacle):
                marker.type = Marker.CUBE
                marker.pose.position.x = float(obstacle.center[0])
                marker.pose.position.y = float(obstacle.center[1])
                marker.pose.position.z = float(obstacle.center[2] + obstacle.size[2] / 2.0)
                marker.pose.orientation.w = 1.0
                marker.scale = Vector3(
                    x=float(obstacle.size[0]),
                    y=float(obstacle.size[1]),
                    z=float(obstacle.size[2])
                )
                marker.color = color
            
            marker_array.markers.append(marker)
        
        # Delete marker to clear old obstacles
        delete_marker = Marker()
        delete_marker.header = Header()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, delete_marker)
        
        self.pub_markers.publish(marker_array)
        self.get_logger().info(f'Published {len(obstacles)} obstacle markers')
    
    def republish_terrain(self):
        """Republish terrain markers periodically to ensure RViz displays them."""
        if self.obstacles:
            self.publish_markers(self.obstacles, self.terrain_type)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
