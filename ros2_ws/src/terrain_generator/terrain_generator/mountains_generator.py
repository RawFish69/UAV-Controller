"""Mountains terrain generator."""
import numpy as np
from .obstacles import BoxObstacle, CylinderObstacle


def generate_mountains(space_dim, num_peaks=15, base_size_range=(3.0, 8.0),
                       height_range=(10.0, 30.0), start_pos=None,
                       min_start_distance=5.0):
    """
    Generate a mountainous terrain with peaks and rocks.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_peaks: Number of mountain peaks
        base_size_range: (min, max) base size for peaks
        height_range: (min, max) height for peaks
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        
    Returns:
        list: List of obstacle objects (boxes and cylinders)
    """
    obstacles = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    # Generate mountain peaks (box obstacles)
    for _ in range(num_peaks):
        base_size = np.random.uniform(*base_size_range)
        height = np.random.uniform(*height_range)
        
        # Random position
        x = np.random.uniform(base_size/2, space_dim[0] - base_size/2)
        y = np.random.uniform(base_size/2, space_dim[1] - base_size/2)
        z = 0
        
        center = np.array([x, y, z])
        
        # Check distance from start
        if np.linalg.norm(center[:2] - start_pos[:2]) < (base_size/2 + min_start_distance):
            continue
        
        # Box obstacle for mountain peak
        size = np.array([base_size, base_size, height])
        obstacles.append(BoxObstacle(center=center, size=size))
    
    # Add some smaller rocks (cylinders)
    num_rocks = int(num_peaks * 0.5)
    for _ in range(num_rocks):
        radius = np.random.uniform(0.5, 2.0)
        height = np.random.uniform(2.0, 8.0)
        
        x = np.random.uniform(radius, space_dim[0] - radius)
        y = np.random.uniform(radius, space_dim[1] - radius)
        z = 0
        
        center = np.array([x, y, z])
        
        if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
            continue
        
        obstacles.append(CylinderObstacle(center=center, height=height, radius=radius))
    
    return obstacles


def generate_mountain_ridge(space_dim, ridge_length=50.0, ridge_width=5.0,
                            peak_height_range=(15.0, 25.0), num_peaks=5):
    """
    Generate a continuous mountain ridge.
    
    Args:
        space_dim: Dimensions of the 3D space
        ridge_length: Length of the ridge
        ridge_width: Width of the ridge
        peak_height_range: (min, max) height for peaks along ridge
        num_peaks: Number of peaks along the ridge
        
    Returns:
        list: List of box obstacles forming a ridge
    """
    obstacles = []
    
    # Ridge direction (diagonal across space)
    start_x = space_dim[0] * 0.2
    start_y = space_dim[1] * 0.2
    end_x = space_dim[0] * 0.8
    end_y = space_dim[1] * 0.8
    
    dx = (end_x - start_x) / num_peaks
    dy = (end_y - start_y) / num_peaks
    
    for i in range(num_peaks):
        x = start_x + i * dx
        y = start_y + i * dy
        height = np.random.uniform(*peak_height_range)
        
        center = np.array([x, y, 0])
        size = np.array([ridge_width, ridge_width, height])
        
        obstacles.append(BoxObstacle(center=center, size=size))
    
    return obstacles
