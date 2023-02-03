from planner import R1_feasibility
from multi_level_surface_map import MultiLevelSurfaceMap
import matplotlib.pyplot as plt
import numpy as np

pi = np.pi


Map = MultiLevelSurfaceMap( 'data/world.json', 0.2)
print(type(Map.mlsm))

f = (-10, 2, 8, 0)
print(R1_feasibility(f, Map))
