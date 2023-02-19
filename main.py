#!/usr/bin/env python3

import numpy as np
import argparse
from pathlib import Path
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Our imports
from src.python.utils import json2dict, get_world_meshes, get_2d_rectangle_coordinates, get_2d_circle_coordinates
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.planner import RRT
from src.python.parameters import *


def main(world_json, resolution, time_max, print_steps):
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
    
    # Define initial stance and goal region
    if(str(world_json) == 'data/world_of_stairs.json'):
        f_swg = [1.85, 2, 0.00020, np.pi/2] 
        f_sup = [1.65, 2, 0.00020, np.pi/2]
        goal_region = (1.85, 2, 3.005, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/easy.json'):
        f_swg = [-2.2, 1.1, 0.000205, np.pi/2] 
        f_sup = [-2.4, 1.1, 0.000205, np.pi/2]
        goal_region = (-2.3, 10, 0.728, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/tunnel.json'): # TODO correct initial configuration
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
        # goal_region = [[-3.1, -0.5, 0.00205], [-4, -0.5, 0.00205],[-3.1, 0.4, 0.00205],[-4, 0.4, 0.00205]]  #tunnel map
    elif(str(world_json) == 'data/world_of_stairs2.json'):
        f_swg = [-2.725, 0.47, 0.000205, np.pi/2] 
        f_sup = [-2.925, 0.47, 0.000205, np.pi/2]
        goal_region = (-2.825, 0.47, 1.903, 0.5) # (x, y, z, radius)
    else:
        print('Initial stance and goal region not specified!')
        display_results(fig, map)
        return
    
    initial_stance = (f_swg, f_sup)
    
    # Draw the goal region:
    goal_vertices = get_2d_circle_coordinates((goal_region[0], goal_region[1]), goal_region[3])
    goal_x = goal_vertices[:,0]
    goal_y = goal_vertices[:,1]
    goal_z = np.ones(len(goal_vertices), dtype=np.float64).dot(goal_region[2]+0.01)
    goal_mesh = go.Mesh3d(name='Goal region', x=goal_x, y=goal_y, z=goal_z, color='yellow')
    fig.add_trace(goal_mesh, row=1, col=2)

    # Planning    
    steps = RRT(initial_stance, goal_region, map, time_max)
    
    # Show footsteps 
    for i, step in enumerate(steps):
        foot = step[0]
        foot_id = step[1]
        if(print_steps): print('Footprint '+ str(i) +':\t', foot, '\t', foot_id)
        vertices = get_2d_rectangle_coordinates((foot[0],foot[1]), ROBOT_FOOT_SIZE, foot[3])
        x = vertices[:,0]
        y = vertices[:,1]
        z = np.ones(4, dtype=np.float64)
        z_delta = 0.01 if i not in [0,1] else 0.1 # little vertical shift to avoid overlappings with objects surfaces and show clearly the starting position
        z = z.dot(foot[2] + z_delta) 
        footprint_name = 'Step'+str(i-1) if i not in [0,1] else 'Start'+foot_id
        footprint_color = 'blue' if foot_id == 'Right' and i not in [0,1] else 'red' if i not in [0,1] else 'cyan' if foot_id == 'Right' else 'magenta'
        footprint_mesh = go.Mesh3d(name=footprint_name, x=x, y=y, z=z, color=footprint_color)
        fig.add_trace(footprint_mesh, row=1, col=2)
    display_results(fig, map)


def display_results(fig, map):
    fig.update_layout(margin=dict(l=0, r=0, b=0, t=0), showlegend=False)
    visualization_range = [min(map.world_dimensions[0][0], map.world_dimensions[1][0], map.world_dimensions[2][0]), max(map.world_dimensions[0][1], map.world_dimensions[1][1], map.world_dimensions[2][1])+map.resolution]
    nticks = int(visualization_range[1] - visualization_range[0])*2
    fig.update_layout(scene = dict(xaxis = dict(nticks=nticks, range=visualization_range), yaxis = dict(nticks=nticks, range=visualization_range), zaxis = dict(nticks=nticks, range=visualization_range)))
    fig.update_layout(scene2 = dict(xaxis = dict(nticks=nticks, range=visualization_range), yaxis = dict(nticks=nticks, range=visualization_range), zaxis = dict(nticks=nticks, range=visualization_range)))
    fig.update_layout(scene_aspectmode='cube')
    fig.update_layout(scene2_aspectmode='cube')
    fig.show()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-json', type=Path, default='data/world_of_stairs2.json', help='json file containing the information about the boxes contained in the world')
    parser.add_argument('--resolution', type=float, default=0.02, help='Set the map resolution')
    parser.add_argument('--time-max', type=int, default=1000, help='Set the map resolution')
    parser.add_argument('--print-steps', action='store_true', help='Print the returned steps information on the terminal')
    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    main(**vars(opt))



