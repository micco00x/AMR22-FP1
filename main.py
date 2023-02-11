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
from src.python.utils import json2dict, rpy2rotation_matrix, get_z_rotation_matrix, get_3d_cuboid_coordinates
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.visualization import get_world_meshes


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
    
    # Show footsteps
    
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



