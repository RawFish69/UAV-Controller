import numpy as np
import sys
sys.path.append('.') # Add terrain_generator mod to path
from obstacles import CylinderObstacle, BoxObstacle

tree = CylinderObstacle(center=[10.0, 10.0, 5.0], height=10.0, radius=2.0)
print(f"Tree bounds: x=[8,12] y=[8,12] z=[5,15]")
# 2D Point right inside the trunk (x=10, y=10)
p_in = np.array([10.0, 10.0])
print(p_in, "->", tree.is_inside(p_in, inflation=0.0))

# 3D Point in the trunk (x=10, y=10, z=10)
p_in_3d = np.array([10.0, 10.0, 10.0])
print(p_in_3d, "->", tree.is_inside(p_in_3d, inflation=0.0))

# 2D Point outside (x=20, y=20)
p_out = np.array([20.0, 20.0])
print(p_out, "->", tree.is_inside(p_out, inflation=0.0))

box = BoxObstacle(center=[20.0, 20.0, 5.0], size=[4.0, 4.0, 10.0])
print("\nBox bounds: x=[18,22] y=[18,22] z=[0,10]")
p_in_box = np.array([20.0, 20.0])
print(p_in_box, "->", box.is_inside(p_in_box, inflation=0.0))

p_out_box = np.array([30.0, 30.0])
print(p_out_box, "->", box.is_inside(p_out_box, inflation=0.0))

