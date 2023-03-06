#!/usr/bin/env python3

import os
import numpy as np
import argparse
from pathlib import Path
from plotly.subplots import make_subplots
import plotly.graph_objects as go

import time

# Our imports
from src.python.utils import json2dict, get_world_meshes, get_2d_rectangle_coordinates, get_2d_circle_coordinates
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.planner import RRT
from src.python.parameters import *


def main(world_json, resolution, time_max, print_steps, no_display_steps, no_out, out, out2):
    #sum of time's variables
    tot_1 = 0
    tot_2 = 0
    tot_3 = 0
    tot_trajectory = 0
    tot_v_near = 0


    fig = make_subplots(
        cols=2,
        specs=[[{"type": "scene"}, {"type": "scene"}]],
        column_widths=[0.4, 0.6]
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
    elif(str(world_json) == 'data/world_of_stairs2.json'):
        f_swg = [-2.525, 1.675, 0.000205, 0] 
        f_sup = [-2.525, 1.875, 0.000205, 0]
        goal_region = (-2.825, 0.47, 1.903, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/easy.json'):
        f_swg = [-2.2, 1.1, 0.000205, np.pi/2] 
        f_sup = [-2.4, 1.1, 0.000205, np.pi/2]
        goal_region = (-2.3, 10, 0.728, 0.5) # (x, y, z, radius)
    elif(str(world_json) == 'data/tunnel.json'): # TODO correct initial configuration
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
        # goal_region = [[-3.1, -0.5, 0.00205], [-4, -0.5, 0.00205],[-3.1, 0.4, 0.00205],[-4, 0.4, 0.00205]]  #tunnel map
    elif(str(world_json) == 'data/tunnel2.json'): # TODO correct initial configuration
        f_swg = [3.3, 0.1, 0.000205, np.pi]
        f_sup = [3.3, -0.1, 0.000205, np.pi]
        goal_region = (-3.5, 0, 0.00205, 0.4) # (x, y, z, radius)
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
    steps_to_goal, all_steps, all_time, time_for = RRT(initial_stance, goal_region, map, time_max)
    
    # Output file creation
    if(not no_out):
        f = open(out, 'w')
        labels = 'NR\tswg_id\tpos_x\tpos:y\tpos_z\torientation\ttrajectory_params\ttime_step_1\ttime_step_2\ttime_step_3\ttime_generate_trajectory\ttime_v_near_generation\n'
        f.write(labels)
        if steps_to_goal is not None:
            labels_goal = 'NR\tswg_id\tpos_x\tpos:y\tpos_z\torientation\ttrajectory_params\n'
            f2 = open(out2, 'w')
            f2.write(labels_goal)
    
    if steps_to_goal is not None:
        #write the goal path to the file out2 (and show it)
        for i, step in enumerate(steps_to_goal):
            foot = step[0]
            foot_id = step[1]
            traj_params = step[2]
            if(print_steps): print('Footprint '+ str(i) +':\t', foot, '\t', foot_id)
            # Steps info in output file:
            if(not no_out and i not in [0,1]):
                info = str(i-1) + '\t' + foot_id + '\t' + str(foot[0]) + '\t' + str(foot[1]) + '\t' + str(foot[2]) + '\t' + str(foot[3]) + '\t' + str(traj_params) + '\n' 
                f2.write(info)
            # Steps visualization:
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

        #write all the steps to the file out
        for i, step in enumerate(all_steps):
            foot = step[0]
            foot_id = step[1]
            traj_params = step[2]
            # Steps info in output file:
            if(not no_out and i not in [0,1]):
                info = str(i-1) + '\t' + foot_id + '\t' + str(foot[0]) + '\t' + str(foot[1]) + '\t' + str(foot[2]) + '\t' + str(foot[3]) + '\t' + str(traj_params) + '\t' + str(all_time[i-2][1]) + '\t' + str(all_time[i-2][3]) + '\t' + str(all_time[i-2][4]) + '\t' + str(all_time[i-2][2]) + '\t' + str(all_time[i-2][0]) + '\n' 
                f.write(info)
                tot_1 += all_time[i-2][1]
                tot_2 += all_time[i-2][3]
                tot_3 += all_time[i-2][4]
                tot_trajectory += all_time[i-2][2]
                tot_v_near += all_time[i-2][0]

    else:
        #write all the steps to the file out (and show it)
        for i, step in enumerate(all_steps):
            foot = step[0]
            foot_id = step[1]
            traj_params = step[2]
            if(print_steps): print('Footprint '+ str(i) +':\t', foot, '\t', foot_id)
            # Steps info in output file:
            if(not no_out and i not in [0,1]):
                info = str(i-1) + '\t' + foot_id + '\t' + str(foot[0]) + '\t' + str(foot[1]) + '\t' + str(foot[2]) + '\t' + str(foot[3]) + '\t' + str(traj_params) + '\t' + str(all_time[i-2][1]) + '\t' + str(all_time[i-2][3]) + '\t' + str(all_time[i-2][4]) + '\t' + str(all_time[i-2][2]) + '\t' + str(all_time[i-2][0]) + '\n' 
                f.write(info)
                tot_1 += all_time[i-2][1]
                tot_2 += all_time[i-2][3]
                tot_3 += all_time[i-2][4]
                tot_trajectory += all_time[i-2][2]
                tot_v_near += all_time[i-2][0]
            # Steps visualization:
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
    
    # Show footsteps 
    if not no_display_steps: display_results(fig, map)
    
    if(not no_out):
        time_1 = "Mean time step 1:" + '\t' + str(round(tot_1/(len(all_steps)-2), 7)) + '\n' 
        time_2 = "Mean time step 2:" + '\t' + str(round(tot_2/(len(all_steps)-2), 7)) + '\n' 
        time_3 = "Mean time step 3:" + '\t' + str(round(tot_3/(len(all_steps)-2), 7)) + '\n' 
        time_traj = "Mean time trajectory generation:" + '\t' + str(round(tot_trajectory/(len(all_steps)-2), 7)) + '\n' 
        time_v_near = "Mean time v_near selection:" + '\t' + str(round(tot_v_near/(len(all_steps)-2), 7)) + '\n' 
        totale_1 = "Total time step 1:" + '\t' + str(round(tot_1, 7)) + '\n' 
        totale_2 = "Total time step 2:" + '\t' + str(round(tot_2, 7)) + '\n' 
        totale_3 = "Total time step 3:" + '\t' + str(round(tot_3, 7)) + '\n' 
        totale_traj = "Total time trajectory generation:" + '\t' + str(round(tot_trajectory, 7)) + '\n' 
        totale_v_near = "total time v_near selection:" + '\t' + str(round(tot_v_near, 7)) + '\n' 
        totale_for = "total time for cycle:" + '\t' + str(round(time_for, 7)) + '\n' 
        f.write("\n")
        f.write(time_1)
        f.write(time_2)
        f.write(time_3)
        f.write(time_traj)
        f.write(time_v_near)
        f.write("\n")
        f.write(totale_1)
        f.write(totale_2)
        f.write(totale_3)
        f.write(totale_traj)
        f.write(totale_v_near)
        f.write(totale_for)
        f.close()
        if steps_to_goal is not None:
            f2.close()
        


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
    parser.add_argument('--resolution', type=float, default=MLSM_RESOLUTION, help='Set the map resolution')
    parser.add_argument('--time-max', type=int, default=500, help='Set the map resolution')
    parser.add_argument('--print-steps', action='store_true', help='Print the returned steps information on the terminal')
    parser.add_argument('--no-display-steps', action='store_true', help='Do not show the returned steps information in a 3D scene')
    parser.add_argument('--no-out', action='store_true', help='Don\'t save the final output in a file.')
    parser.add_argument('--out', type=Path, default='outputs/out_steps.tsv', help='Path to the tsv file where to store the output.')
    parser.add_argument('--out2', type=Path, default='outputs/out_goal.tsv', help='Path to the tsv file where to store the output.')
    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    main(**vars(opt))



