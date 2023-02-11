import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import json
import argparse
from pathlib import Path
from math import cos, sin
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Our imports
from src.python.utils import json2dict, get_world_meshes, get_3d_cuboid_coordinates, get_2d_rectangle_coordinates
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.planner import RRT


def main(world_json, resolution):
    fig = make_subplots(
    cols=2,
    specs=[[{"type": "scene"}, {"type": "scene"}]],
    )

    # MultiLevelSurfaceMap
    map = MultiLevelSurfaceMap(world_json, resolution)
    fig.add_trace(map.as_showable(), row=1, col=1)
    
    
    # World of stairs
    scene = json2dict(world_json)['boxes']
    world_meshes = get_world_meshes(scene)
    for mesh in world_meshes:
        fig.add_trace(mesh, row=1, col=2)
    
    #RRT
    f_swg = [0.45, -2.3, 0.00205, 0] 
    f_sup = [0.65, -2.3, 0.00205, 0]
    initial_stance = (f_swg, f_sup)
    g = map.world2map_coordinates(1.85, 1.5)
    f = map.world2map_coordinates(0.45, -2.3)
    goal_region = [ [g[0], g[1], 0.00205], [41, -31, 0.00205],[60, -141, 0.00205],[60, -141, 0.00205],[45, -70, 0.00205], ] #FALSE GOAL REGION, IT NEVER ENDS
    print('goal region:', goal_region, 'Start', f)
    time_max = 1000

    # Show goal regions
    
    # Planning    
    steps = RRT(initial_stance, goal_region, map, time_max)
    
    # Show footsteps 
    for step in steps:
        foot = step[0]
        foot_id = step[1]
        vertices = get_2d_rectangle_coordinates((foot[0],foot[1]), (0.15, 0.25), foot[3])
        x = vertices[:,0]
        y = vertices[:,1]
        z = np.ones(4, dtype=np.float64)
        z = z.dot(foot[2]+0.001)
        foot_mesh = go.Mesh3d(x=x, y=y, z=z, color='cyan' if foot_id == 'Right' else 'red')
        fig.add_trace(foot_mesh, row=1, col=2)
        
        
    
    fig.update_layout(margin=dict(l=0, r=0, b=0, t=0), showlegend=False)
    fig.show()



def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-json', type=Path, default='data/world_of_stairs.json', help='json file containing the information about the boxes contained in the world')
    parser.add_argument('--resolution', type=float, default=0.02, help='Set the map resolution')
    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    # old_main(**vars(opt))
    main(**vars(opt))



