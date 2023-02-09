from planner import r1_feasibility
from multi_level_surface_map import MultiLevelSurfaceMap
import matplotlib.pyplot as plt
import numpy as np
from numpy import transpose,  cos ,sin

pi = np.pi

from planner import RRT


map = MultiLevelSurfaceMap( 'data/world_of_stairs.json', 0.2)
f_swg = (0.4, 0.4, 0.00205, 0) 
f_sup = (0, 0, 0.00205, 0)
initial_stance = [f_sup, f_sup]
goal_region = [ [map.world2map_coordinates(777777, 7777777)[0], map.world2map_coordinates(0, 0)[1], 0.00205] ] #FALSE GOAL REGION, IT NEVER ENDS
#print('goal region:', goal_region)
time_max = 1000

RRT(initial_stance, goal_region, map, time_max)

# f = (-10, 2, 8, 0)
# print(r1_feasibility(f, map))
#print(map.mlsm[1][200])
