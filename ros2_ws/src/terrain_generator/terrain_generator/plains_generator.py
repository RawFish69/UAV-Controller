"""Plains terrain generator."""
import numpy as np
from .obstacles import CylinderObstacle, BoxObstacle


def generate_plains(space_dim, num_obstacles=10, obstacle_types=['bush', 'rock'],
                    start_pos=None, min_start_distance=3.0):
    """
    Generate a plains terrain with sparse obstacles.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_obstacles: Number of obstacles to place
        obstacle_types: List of obstacle types ('bush', 'rock', 'tree')
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        
    Returns:
        list: List of obstacle objects
    """
    obstacles = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    for _ in range(num_obstacles):
        obstacle_type = np.random.choice(obstacle_types)
        
        if obstacle_type == 'bush':
            # Small cylindrical bushes
            radius = np.random.uniform(0.3, 0.8)
            height = np.random.uniform(0.5, 2.0)
            x = np.random.uniform(radius, space_dim[0] - radius)
            y = np.random.uniform(radius, space_dim[1] - radius)
            z = 0
            center = np.array([x, y, z])
            
            if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
                continue
            
            obstacles.append(CylinderObstacle(center=center, height=height, radius=radius))
        
        elif obstacle_type == 'rock':
            # Small box rocks
            size = np.random.uniform(0.5, 2.0, 3)
            size[2] = np.random.uniform(0.3, 1.5)  # Height
            x = np.random.uniform(size[0]/2, space_dim[0] - size[0]/2)
            y = np.random.uniform(size[1]/2, space_dim[1] - size[1]/2)
            z = 0
            center = np.array([x, y, z])
            
            if np.linalg.norm(center[:2] - start_pos[:2]) < (size[0]/2 + min_start_distance):
                continue
            
            obstacles.append(BoxObstacle(center=center, size=size))
        
        elif obstacle_type == 'tree':
            # Occasional small trees
            radius = np.random.uniform(0.4, 1.0)
            height = np.random.uniform(3.0, 8.0)
            x = np.random.uniform(radius, space_dim[0] - radius)
            y = np.random.uniform(radius, space_dim[1] - radius)
            z = 0
            center = np.array([x, y, z])
            
            if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
                continue
            
            obstacles.append(CylinderObstacle(center=center, height=height, radius=radius))
    
    return obstacles


def generate_empty_plains(space_dim):
    """
    Generate empty plains with no obstacles.
    
    Args:
        space_dim: Dimensions of the 3D space
        
    Returns:
        list: Empty list (no obstacles)
    """
    return []
