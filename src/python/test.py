from planner import r1_feasibility
from multi_level_surface_map import MultiLevelSurfaceMap
import matplotlib.pyplot as plt
import numpy as np
from numpy import transpose,  cos ,sin

pi = np.pi

from planner import RRT


map = MultiLevelSurfaceMap( 'data/world_of_stairs.json', 0.2)

f_swg = [0.45, -2.3, 0.00205, 0] 
f_sup = [0.65, -2.3, 0.00205, 0]
initial_stance = [f_sup, f_sup]
g = map.world2map_coordinates(1.85, 1.5)
f = map.world2map_coordinates(0.45, -2.3)
goal_region = [ [g[0], g[1], 0.00205], [41, -31, 0.00205],[60, -141, 0.00205],[60, -141, 0.00205],[45, -70, 0.00205], ] #FALSE GOAL REGION, IT NEVER ENDS
print('goal region:', goal_region, 'Start', f)
time_max = 1000

RRT(initial_stance, goal_region, map, time_max)

# f = (-10, 2, 8, 0)
# print(r1_feasibility(f, map))
#print(map.mlsm[1][200])
