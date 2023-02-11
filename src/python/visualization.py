import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import json
import argparse
from pathlib import Path
from math import cos, sin
import plotly.graph_objects as go

# Our imports
from src.python.utils import json2dict, rpy2rotation_matrix, get_z_rotation_matrix, get_3d_cuboid_coordinates
from src.python.multi_level_surface_map import MultiLevelSurfaceMap


def get_world_meshes(scene):
    '''scene: a dictionary containing all the info about a scene. For each object it is specified the position of its centroind, its size and its orientation in a RPY-format.
    This function will return a list of go.Mesh3d that can be showed through plotly.'''
    meshes = []
    for object in scene:
        size = scene[object]['size']
        orientation = scene[object]['orientation']
        position = scene[object]['position'] # position of the centroid of the box
        cuboid = get_3d_cuboid_coordinates(position, size, orientation)
        i = 0
        x = np.array(cuboid[:,0])
        y = np.array(cuboid[:,1])
        z = np.array(cuboid[:,2])
        meshes.append(go.Mesh3d(x=x, y=y, z=z, alphahull=0, name=object))
        
    return meshes


def main(world_json, resolution):

    # Multilevel map plot
    map = MultiLevelSurfaceMap(world_json, resolution)
    x, y, z = map.as_numpy(stride=1)
    
    # TODO add subplot of the world made by cubes
    
    fig = go.Figure(data=[go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode='markers',
        marker=dict(
            size=10,#0.02/map.resolution,
            color=z,                # set color to an array/list of desired values
            colorscale='Viridis',   # choose a colorscale
            opacity=0.8
        )
    )])

    # tight layout
    fig.update_layout(margin=dict(l=0, r=0, b=0, t=0))
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



