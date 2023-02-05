import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import json
import argparse
from pathlib import Path
from math import cos, sin

from utils import json2dict, rpy2rotation_matrix, get_z_rotation_matrix
from multi_level_surface_map import MultiLevelSurfaceMap

def get_coordinates(position, size, orientation):
    X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
         [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
         [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
         [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
         [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
         [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]] # a list of vertices for each face of a cuboid
    X = np.array(X).astype(float)

    # Scaling based on the size
    for i in range(3):
        if i == 2: X[:,:,i] *= -size[i] # for convenience wrt the depth definition
        else: X[:,:,i] *= size[i]

    # Rotation based on the orientation
    rotation_matrix = rpy2rotation_matrix(orientation)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            X[i,j,:] = rotation_matrix.dot(X[i,j,:])

    # Calculation of the position of the 'reference vertex' given the centroid of the cuboid
    delta = np.array([-size[0]/2, -size[1]/2, size[2]/2]).astype(float)
    delta = rotation_matrix.dot( delta )
    position = np.array(position) + delta

    # Translation based on the 'reference vertex'
    X += position

    return X


def get_poly_collection(scene):
    # TODO: separate boxes and flat surfaces
    collection = []
    for object in scene:
        obj_size = scene[object]['size']
        obj_orientation = scene[object]['orientation']
        obj_position = scene[object]['position']
        block = get_coordinates(obj_position, obj_size, obj_orientation)
        collection.append( block )
    collection = list(np.concatenate(collection))
    return Poly3DCollection(collection, edgecolor=[0.5, 0.5, 0.5])
    # return Poly3DCollection(collection, shade=True) # TODO: better but works only with matplotlib v.3.7 and above


def main(world_json, resolution):

    # Multilevel map plot
    map = MultiLevelSurfaceMap(world_json, resolution)
    #map.plot()

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
    # ax.set_xlim([map.world_dimensions[0][0], map.world_dimensions[0][1]])
    # ax.set_ylim([map.world_dimensions[1][0], map.world_dimensions[1][1]])
    # ax.set_zlim([map.world_dimensions[2][0], map.world_dimensions[2][1]])
    ax.set_xlim(-4, 6)
    ax.set_ylim(-4, 6)
    ax.set_zlim(-4, 6)

    plt.show()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-json', type=Path, default='data/world_of_stairs.json', help='json file containing the information about the boxes contained in the world')
    parser.add_argument('--resolution', type=float, default=0.02, help='Set the map resolution')
    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    main(**vars(opt))