"""Forest terrain generator."""
import numpy as np
from .obstacles import CylinderObstacle


def generate_forest(space_dim, grid_size=10, radius_range=(0.5, 1.5), 
                    height_range=(5.0, 15.0), density=0.7, 
                    start_pos=None, min_start_distance=3.0):
    """
    Generate a forest of cylindrical obstacles (trees) using a grid-based approach.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        grid_size: Number of grid cells along one axis
        radius_range: (min, max) radius for tree cylinders
        height_range: (min, max) height for tree cylinders
        density: Probability of placing a tree in each grid cell (0-1)
        start_pos: Start position (x, y, z) to avoid trees near start
        min_start_distance: Minimum distance from start position
        
    Returns:
        list: List of CylinderObstacle objects
    """
    trees = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    # Calculate grid cell size
    grid_cell_size_x = space_dim[0] / grid_size
    grid_cell_size_y = space_dim[1] / grid_size
    
    # Iterate over grid cells
    for i in range(grid_size):
        for j in range(grid_size):
            # Skip based on density
            if np.random.rand() > density:
                continue
            
            # Compute cell bounds
            cell_min_x = i * grid_cell_size_x
            cell_max_x = (i + 1) * grid_cell_size_x
            cell_min_y = j * grid_cell_size_y
            cell_max_y = (j + 1) * grid_cell_size_y
            
            # Randomly place a tree within the grid cell
            radius = np.random.uniform(*radius_range)
            height = np.random.uniform(*height_range)
            
            # Ensure tree fits within cell
            x = np.random.uniform(cell_min_x + radius, cell_max_x - radius)
            y = np.random.uniform(cell_min_y + radius, cell_max_y - radius)
            z = 0
            
            center = np.array([x, y, z])
            
            # Check if too close to start position
            if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
                continue
            
            trees.append(CylinderObstacle(center=center, height=height, radius=radius))
    
    return trees


def generate_random_forest(space_dim, num_trees=50, radius_range=(0.5, 1.5),
                            height_range=(5.0, 15.0), start_pos=None,
                            min_start_distance=3.0):
    """
    Generate a random forest without grid structure.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_trees: Number of trees to generate
        radius_range: (min, max) radius for tree cylinders
        height_range: (min, max) height for tree cylinders
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        
    Returns:
        list: List of CylinderObstacle objects
    """
    trees = []
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    for _ in range(num_trees):
        radius = np.random.uniform(*radius_range)
        height = np.random.uniform(*height_range)
        
        # Random position
        x = np.random.uniform(radius, space_dim[0] - radius)
        y = np.random.uniform(radius, space_dim[1] - radius)
        z = 0
        
        center = np.array([x, y, z])
        
        # Check distance from start
        if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
            continue
        
        # Check collision with existing trees
        collision = False
        for tree in trees:
            if np.linalg.norm(center[:2] - tree.center[:2]) < (radius + tree.radius + 0.5):
                collision = True
                break
        
        if not collision:
            trees.append(CylinderObstacle(center=center, height=height, radius=radius))
    
    return trees
