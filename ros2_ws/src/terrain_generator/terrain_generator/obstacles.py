"""Obstacle classes for terrain generation."""
import numpy as np


class CylinderObstacle:
    """Cylindrical obstacle (e.g., tree trunk)."""
    
    def __init__(self, center, height, radius):
        """
        Initialize a cylindrical obstacle.
        
        Args:
            center: Center position [x, y, z]
            height: Height of the cylinder
            radius: Radius of the cylinder
        """
        self.center = np.array(center)
        self.height = height
        self.radius = radius
    
    def is_inside(self, point, inflation=0.0):
        """
        Check if a point is inside the cylinder.
        
        Args:
            point: Point to check (x, y) or (x, y, z)
            inflation: Safety margin added to radius and height
            
        Returns:
            bool: True if point is inside the cylinder
        """
        if len(point) == 2:
            x, y = point
            z = 0
        else:
            x, y, z = point
        
        x0, y0, z0 = self.center
        
        # Check horizontal distance
        horizontal_distance = np.sqrt((x - x0)**2 + (y - y0)**2)
        if horizontal_distance > self.radius + inflation:
            return False
        
        # Check vertical bounds
        if z < z0 or z > z0 + self.height + inflation:
            return False
        
        return True


class BoxObstacle:
    """Box-shaped obstacle (e.g., building, rock)."""
    
    def __init__(self, center, size):
        """
        Initialize a box obstacle.
        
        Args:
            center: Center position [x, y, z]
            size: Size of the box [width, length, height]
        """
        self.center = np.array(center)
        self.size = np.array(size)
        self.half_size = self.size / 2.0
    
    def is_inside(self, point, inflation=0.0):
        """Check if a point is inside the box."""
        if len(point) == 2:
            x, y = point
            z = self.center[2]
        else:
            x, y, z = point
        
        x0, y0, z0 = self.center
        
        if (abs(x - x0) > self.half_size[0] + inflation or
            abs(y - y0) > self.half_size[1] + inflation or
            abs(z - z0) > self.half_size[2] + inflation):
            return False
        
        return True


def is_point_in_collision(point, obstacles, inflation=0.0):
    """
    Check if a point is in collision with any obstacle.
    
    Args:
        point: Point to check (x, y) or (x, y, z)
        obstacles: List of obstacle objects
        inflation: Safety margin
        
    Returns:
        bool: True if in collision
    """
    for obstacle in obstacles:
        if obstacle.is_inside(point, inflation):
            return True
    return False
