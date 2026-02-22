"""Plains terrain generator."""
import numpy as np
from .obstacles import CylinderObstacle, BoxObstacle


def generate_plains(space_dim, num_obstacles=10, obstacle_types=['bush', 'rock'],
                    start_pos=None, min_start_distance=3.0,
                    local_cluster_radius=0.0):
    """
    Generate a plains terrain with sparse obstacles.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_obstacles: Number of obstacles to place
        obstacle_types: List of obstacle types ('bush', 'rock', 'tree')
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        local_cluster_radius: If > 0, sample obstacles around start_pos within radius
        
    Returns:
        list: List of obstacle objects
    """
    obstacles = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    cluster_radius = float(max(0.0, local_cluster_radius))

    def _sample_xy(margin_x: float, margin_y: float) -> tuple[float, float]:
        if cluster_radius > 0.0:
            xmin = max(margin_x, float(start_pos[0]) - cluster_radius)
            xmax = min(float(space_dim[0]) - margin_x, float(start_pos[0]) + cluster_radius)
            ymin = max(margin_y, float(start_pos[1]) - cluster_radius)
            ymax = min(float(space_dim[1]) - margin_y, float(start_pos[1]) + cluster_radius)
            if xmin < xmax and ymin < ymax:
                return (
                    float(np.random.uniform(xmin, xmax)),
                    float(np.random.uniform(ymin, ymax)),
                )
        return (
            float(np.random.uniform(margin_x, float(space_dim[0]) - margin_x)),
            float(np.random.uniform(margin_y, float(space_dim[1]) - margin_y)),
        )

    for _ in range(num_obstacles):
        obstacle_type = np.random.choice(obstacle_types)
        
        if obstacle_type == 'bush':
            # Small cylindrical bushes
            radius = np.random.uniform(0.3, 0.8)
            height = np.random.uniform(0.5, 2.0)
            x, y = _sample_xy(radius, radius)
            z = 0
            center = np.array([x, y, z])
            
            if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
                continue
            
            obstacles.append(CylinderObstacle(center=center, height=height, radius=radius))
        
        elif obstacle_type == 'rock':
            # Small box rocks
            size = np.random.uniform(0.5, 2.0, 3)
            size[2] = np.random.uniform(0.3, 1.5)  # Height
            x, y = _sample_xy(float(size[0] / 2), float(size[1] / 2))
            z = 0
            center = np.array([x, y, z])
            
            if np.linalg.norm(center[:2] - start_pos[:2]) < (size[0]/2 + min_start_distance):
                continue
            
            obstacles.append(BoxObstacle(center=center, size=size))
        
        elif obstacle_type == 'tree':
            # Occasional small trees
            radius = np.random.uniform(0.4, 1.0)
            height = np.random.uniform(3.0, 8.0)
            x, y = _sample_xy(radius, radius)
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
