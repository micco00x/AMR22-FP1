#!/usr/bin/env python3

import os
import numpy as np
import argparse
from pathlib import Path
import datetime
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Our imports
from src.python.utils import json2dict, get_world_meshes, get_2d_rectangle_coordinates, get_2d_circle_coordinates, retrieve_steps, retrieve_all_steps, save_tree_results_on_tsv, save_goal_results_on_tsv, get_number_of_nodes
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.planner import RRT
from src.python.parameters import *

# import plotly.express as px

def main(world_json, resolution, time_max, no_display, override):
    run_info = {'world': world_json.stem, 'time_max': str(time_max), 'datetime': str(datetime.datetime.now()).split('.')[0].replace(' ', '_' ), 'resolution': resolution}
    output_dir = 'outputs/' + run_info['world'] + '_' + run_info['time_max'] + '_' + run_info['datetime']
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    else: # Just to be sure
        print('The output directory already exists!')
        return None
    
    fig = make_subplots(
        cols = 2,
        rows = 2,
        column_widths=[0.5, 0.5],
        row_heights=[0.5, 0.5],
        vertical_spacing=0.01,
        horizontal_spacing=0.01,
        specs=[[{"type": "scene"}, {"type": "scene", 'rowspan': 2}],
               [{"type": "scene"}, None]]
    )

    # MultiLevelSurfaceMap
    map = MultiLevelSurfaceMap(world_json, resolution)
    
    # World of stairs
    scene = json2dict(world_json)['boxes']
    world_meshes = get_world_meshes(scene)
    for mesh in world_meshes:
        fig.add_trace(mesh, row=1, col=2)
        mesh.opacity = 0.3
        fig.add_trace(mesh, row=2, col=1)
    
    # Define initial stance and goal region
    if(str(world_json) == 'data/world_of_stairs.json'):
        f_swg = [1.85, 2, 0.00020, np.pi/2] 
        f_sup = [1.65, 2, 0.00020, np.pi/2]
        goal_region = (1.85, 2, 3.005, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/world_of_stairs2.json'):
        f_swg = [-2.525, 1.675, 0.000205, 0] 
        f_sup = [-2.525, 1.875, 0.000205, 0]
        goal_region = (-2.825, 0.47, 1.903, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/world_of_stairs3.json'):
        f_swg = [-2.525, 1.675, 0.000205, 0] 
        f_sup = [-2.525, 1.875, 0.000205, 0]
        goal_region = (-2.825, 0.47, 1.903, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/easy.json'):
        f_swg = [-2.2, 1.1, 0.000205, np.pi/2] 
        f_sup = [-2.4, 1.1, 0.000205, np.pi/2]
        goal_region = (-2.3, 10, 0.728, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/easy2.json'):
        f_swg = [-2.4, 2, 0.000205, np.pi/2] 
        f_sup = [-2.6, 2, 0.000205, np.pi/2]
        goal_region = (-1.8, 14.8, 1.00205, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/tunnel.json'):
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
        # goal_region = [[-3.1, -0.5, 0.00205], [-4, -0.5, 0.00205],[-3.1, 0.4, 0.00205],[-4, 0.4, 0.00205]]  #tunnel map
    elif(str(world_json) == 'data/tunnel2.json'):
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
    elif(str(world_json) == 'data/tunnel3.json'):
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
    elif(override):
        f_swg = CUSTOM_INITIAL_F_SWG
        f_sup = CUSTOM_INITIAL_F_SUP
        goal_region = CUSTOM_GOAL_REGION
    else:
        print('Initial stance and goal region not specified!')
        return
    initial_stance = (f_swg, f_sup)
    run_info['initial_f_sup'] = f_sup
    run_info['initial_f_wg'] = f_swg
    run_info['goal_region'] = goal_region
    
    # Draw the goal region:
    goal_vertices = get_2d_circle_coordinates((goal_region[0], goal_region[1]), goal_region[3])
    goal_x = goal_vertices[:,0]
    goal_y = goal_vertices[:,1]
    goal_z = np.ones(len(goal_vertices), dtype=np.float64).dot(goal_region[2]+0.01)
    goal_mesh = go.Mesh3d(name='Goal region', x=goal_x, y=goal_y, z=goal_z, color='yellow')
    fig.add_trace(goal_mesh, row=1, col=2)

    # Planning    
    rrt_root, goal_node = RRT(initial_stance, goal_region, map, time_max)
    if not rrt_root: return None
    nr_rrt_nodes = get_number_of_nodes(rrt_root)
    run_info['nr_rrt_nodes'] = nr_rrt_nodes
    print('Number of nodes: ', nr_rrt_nodes)
    
    # Save and rrt data as tsv
    save_tree_results_on_tsv(rrt_root, output_dir)
    save_goal_results_on_tsv(goal_node, output_dir) if goal_node else save_goal_results_on_tsv(rrt_root, output_dir)
    
    # Save info about the run
    with open(output_dir + '/run_info.txt', 'w') as f:
        for item in run_info.items():
            f.write(item[0] + ':\t' + str(item[1]) + '\n')
            
    # MLSM visualization
    map_showable = map.as_showable()
    fig.add_trace(map_showable[0], row=1, col=1)
    for line in map_showable[1]: fig.add_trace(line, row=1, col=1)
    
    # RRT visualization
    rrt_showable = rrt_root.as_showable()
    fig.add_trace(rrt_showable[0], row=2, col=1)
    # for line in rrt_showable[1]: fig.add_trace(line, row=2, col=1)
    
    # Plan visualization:
    steps = retrieve_steps(goal_node) if goal_node else retrieve_steps(rrt_root)
    for i, step in enumerate(steps):
        foot = step[0]
        foot_id = step[1]
        vertices = get_2d_rectangle_coordinates((foot[0],foot[1]), ROBOT_FOOT_SIZE, foot[3])
        x = vertices[:,0]
        y = vertices[:,1]
        z = np.ones(4, dtype=np.float64)
        z_delta = 0.001 if i not in [0,1] else 0.005 # little vertical shift to avoid overlappings with objects surfaces and show clearly the starting position
        z = z.dot(foot[2] + z_delta) 
        footprint_name = 'Step'+str(i-1) if i not in [0,1] else 'Start'+foot_id
        footprint_color = 'blue' if foot_id == 'Right' and i not in [0,1] else 'red' if i not in [0,1] else 'cyan' if foot_id == 'Right' else 'magenta'
        footprint_mesh = go.Mesh3d(name=footprint_name, x=x, y=y, z=z, color=footprint_color)
        fig.add_trace(footprint_mesh, row=1, col=2)
    
    # Save the interactive plot
    fig = update_figure_layout(fig, map)             
    print('Saving the plot...')
    fig.write_html(output_dir + '/plot.html')
    print('Done!')
    
    # Display
    if not no_display: fig.show()



def update_figure_layout(fig, map):
    fig.update_layout(margin=dict(l=0, r=0, b=0, t=0), showlegend=False)
    visualization_range = [min(map.world_dimensions[0][0], map.world_dimensions[1][0], map.world_dimensions[2][0]), max(map.world_dimensions[0][1], map.world_dimensions[1][1], map.world_dimensions[2][1])+map.resolution]
    nticks = int(visualization_range[1] - visualization_range[0])*2
    fig.update_layout(scene = dict(xaxis = dict(nticks=nticks, range=visualization_range), yaxis = dict(nticks=nticks, range=visualization_range), zaxis = dict(nticks=nticks, range=visualization_range)))
    fig.update_layout(scene2 = dict(xaxis = dict(nticks=nticks, range=visualization_range), yaxis = dict(nticks=nticks, range=visualization_range), zaxis = dict(nticks=nticks, range=visualization_range)))
    fig.update_layout(scene3 = dict(xaxis = dict(nticks=nticks, range=visualization_range), yaxis = dict(nticks=nticks, range=visualization_range), zaxis = dict(nticks=nticks, range=visualization_range)))
    fig.update_layout(scene_aspectmode='cube')
    fig.update_layout(scene2_aspectmode='cube')
    fig.update_layout(scene3_aspectmode='cube')
    return fig


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-json', type=Path, default='data/world_of_stairs2.json', help='json file containing the information about the boxes contained in the world')
    parser.add_argument('--resolution', type=float, default=MLSM_RESOLUTION, help='Set the map resolution')
    parser.add_argument('--time-max', type=int, default=500, help='Set the map resolution')
    parser.add_argument('--no-display', action='store_true', help='Do not show the returned steps information in a 3D scene')
    parser.add_argument('--override', action='store_true', help='Specify directly [IN PARAMETERS.PY, lines 2,3,4] the initial pose and the goal region.')
    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    main(**vars(opt))



