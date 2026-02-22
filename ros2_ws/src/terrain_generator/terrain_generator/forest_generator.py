"""Forest terrain generator."""
import numpy as np
from .obstacles import CylinderObstacle


def generate_forest(space_dim, grid_size=10, radius_range=(0.5, 1.5),
                    height_range=(5.0, 15.0), density=0.7,
                    start_pos=None, min_start_distance=3.0,
                    local_cluster_radius=0.0,
                    local_cluster_num_trees=0):
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
        local_cluster_radius: If > 0, bias generation near start_pos
        local_cluster_num_trees: Extra random trees spawned inside local cluster radius
        
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
    
    cluster_radius = float(max(0.0, local_cluster_radius))

    # Iterate over grid cells
    for i in range(grid_size):
        for j in range(grid_size):
            if cluster_radius > 0.0:
                cell_center = np.array([
                    (i + 0.5) * grid_cell_size_x,
                    (j + 0.5) * grid_cell_size_y,
                ])
                if np.linalg.norm(cell_center - start_pos[:2]) > cluster_radius:
                    continue
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
    
    # Add an optional dense local patch independent of the coarse global grid.
    extra_local = int(max(0, local_cluster_num_trees))
    if cluster_radius > 0.0 and extra_local > 0:
        trees.extend(
            generate_random_forest(
                space_dim=space_dim,
                num_trees=extra_local,
                radius_range=radius_range,
                height_range=height_range,
                start_pos=start_pos,
                min_start_distance=min_start_distance,
                sample_center_xy=start_pos[:2],
                sample_radius=cluster_radius,
                existing_trees=trees,
            )
        )

    return trees


def generate_random_forest(space_dim, num_trees=50, radius_range=(0.5, 1.5),
                            height_range=(5.0, 15.0), start_pos=None,
                            min_start_distance=3.0,
                            sample_center_xy=None,
                            sample_radius=0.0,
                            existing_trees=None):
    """
    Generate a random forest without grid structure.
    
    Args:
        space_dim: Dimensions of the 3D space (x, y, z)
        num_trees: Number of trees to generate
        radius_range: (min, max) radius for tree cylinders
        height_range: (min, max) height for tree cylinders
        start_pos: Start position to avoid
        min_start_distance: Minimum distance from start
        sample_center_xy: Optional XY center for local sampling window
        sample_radius: Optional local sampling radius around sample_center_xy
        existing_trees: Existing trees to avoid colliding with
        
    Returns:
        list: List of CylinderObstacle objects
    """
    trees = []
    existing_trees = list(existing_trees or [])
    
    if start_pos is None:
        start_pos = np.array([0, 0, 0])
    else:
        start_pos = np.array(start_pos)
    
    center_xy = None if sample_center_xy is None else np.asarray(sample_center_xy, dtype=float).reshape(2)
    sample_radius = float(max(0.0, sample_radius))

    for _ in range(num_trees):
        radius = np.random.uniform(*radius_range)
        height = np.random.uniform(*height_range)

        # Random position (global or local patch)
        if center_xy is not None and sample_radius > 0.0:
            # Rejection sample inside a disk around the requested center.
            placed = False
            for _attempt in range(20):
                theta = np.random.uniform(0.0, 2.0 * np.pi)
                rho = sample_radius * np.sqrt(np.random.uniform(0.0, 1.0))
                x = center_xy[0] + rho * np.cos(theta)
                y = center_xy[1] + rho * np.sin(theta)
                if radius <= x <= (space_dim[0] - radius) and radius <= y <= (space_dim[1] - radius):
                    placed = True
                    break
            if not placed:
                continue
        else:
            x = np.random.uniform(radius, space_dim[0] - radius)
            y = np.random.uniform(radius, space_dim[1] - radius)
        z = 0
        
        center = np.array([x, y, z])
        
        # Check distance from start
        if np.linalg.norm(center[:2] - start_pos[:2]) < (radius + min_start_distance):
            continue
        
        # Check collision with existing trees
        collision = False
        for tree in [*existing_trees, *trees]:
            if np.linalg.norm(center[:2] - tree.center[:2]) < (radius + tree.radius + 0.5):
                collision = True
                break
        
        if not collision:
            trees.append(CylinderObstacle(center=center, height=height, radius=radius))
    
    return trees
