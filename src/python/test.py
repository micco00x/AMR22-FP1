from planner import R1_feasibility
from multi_level_surface_map import MultiLevelSurfaceMap
import matplotlib.pyplot as plt
import numpy as np

pi = np.pi


map = MultiLevelSurfaceMap( 'data/world_of_stairs.json', 0.2)
print(type(map.mlsm))

f = (-10, 2, 8, 0)
print(R1_feasibility(f, map))
