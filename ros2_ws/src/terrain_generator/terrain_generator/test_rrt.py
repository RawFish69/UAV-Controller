import numpy as np
import sys
import math
sys.path.append('.') # Add terrain_generator mod to path
from obstacles import CylinderObstacle, is_point_in_collision

tree = CylinderObstacle(center=[10.0, 10.0, 5.0], height=10.0, radius=0.2)
obstacles = [tree]
inflation = 1.0 # Larger inflation avoids overshoot but what about normal settings?
step_size = 5.0 # Max dist between RRT nodes

p_from = np.array([5.0, 10.0])
p_to = np.array([12.5, 10.0])

def collision_free_fix(p_from: np.ndarray, p_to: np.ndarray) -> bool:
    dist = np.linalg.norm(p_to - p_from)
    # If we only check every step_size/2, we might jump OVER small objects.
    # What if the step granularity is based on inflation/radius instead of step_size?
    check_dist = 0.5  # Check every 0.5m
    steps = max(2, int(np.ceil(dist / check_dist)))
    
    print(f"Testing {steps} steps.")
    for s in np.linspace(0.0, 1.0, steps):
        p = (1.0 - s) * p_from + s * p_to
        if is_point_in_collision(p, obstacles, inflation=inflation):
            print(f"Collision at {p}")
            return False
        else:
            print(f"Clear at {p}")
    return True

print("Checking points:")
result = collision_free_fix(p_from, p_to)
print(f"\nFinal result: {result}")

