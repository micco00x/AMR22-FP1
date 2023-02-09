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
from utils import json2dict, rpy2rotation_matrix, get_z_rotation_matrix, get_3d_cuboid_coordinates
from multi_level_surface_map import MultiLevelSurfaceMap


def get_poly_collection(scene):
    '''scene: a dictionary containing all the info about a scene. For each object it is specified the position of its centroind, its size and its orientation in a RPY-format.
    This function will return a list of poligons that can be showed through matplotlib.'''
    # TODO: separate boxes and flat surfaces
    collection = []
    for object in scene:
        obj_size = scene[object]['size']
        obj_orientation = scene[object]['orientation']
        obj_position = scene[object]['position']
        block = get_3d_cuboid_coordinates(obj_position, obj_size, obj_orientation)
        collection.append( block )
    collection = list(np.concatenate(collection))
    return Poly3DCollection(collection, edgecolor=[0.5, 0.5, 0.5])
    # return Poly3DCollection(collection, shade=True) # TODO: better but works only with matplotlib v.3.7 and above


def old_main(world_json, resolution):

    # Multilevel map plot
    map = MultiLevelSurfaceMap(world_json, resolution)
    map.plot()

    # World plot
    world_figure = plt.figure('Wold of cubes')
    ax = plt.subplot(projection='3d')
    scene = json2dict(world_json)['boxes']
    boxes = get_poly_collection(scene)
    ax.add_collection(boxes)

    # Axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Visualization boundaries:
    ax.set_xlim([map.world_dimensions[0][0], map.world_dimensions[0][1]])
    ax.set_ylim([map.world_dimensions[1][0], map.world_dimensions[1][1]])
    ax.set_zlim([map.world_dimensions[2][0], map.world_dimensions[2][1]])

    plt.show()


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



