import json
from math import cos, sin
import numpy as np
import plotly.graph_objects as go

def json2dict(json_file):
    dict = {}
    with open(json_file) as file:
        dict = json.load(file)
    return dict

def get_x_rotation_matrix(alpha):
    cos_alpha = cos(alpha)
    sin_alpha = sin(alpha)
    rot_matrix_x = np.array( [[1,    0,             0         ],
                              [0,    cos_alpha,     -sin_alpha],
                              [0,    sin_alpha,     cos_alpha ]] ).astype(float)
    return rot_matrix_x


def get_y_rotation_matrix(alpha):
    cos_alpha = cos(alpha)
    sin_alpha = sin(alpha)
    rot_matrix_y = np.array( [[cos_alpha,    0,    sin_alpha],
                              [0,            1,    0        ],
                              [-sin_alpha,   0,    cos_alpha]] ).astype(float)
    return rot_matrix_y


def get_z_rotation_matrix(alpha):
    cosAlpha = cos(alpha)
    sinAlpha = sin(alpha)
    rot_matrix_z = np.array( [[cosAlpha,    -sinAlpha,  0],
                              [sinAlpha,    cosAlpha,   0],
                              [0,           0,          1]] ).astype(float)
    return rot_matrix_z


def rpy2rotation_matrix(orientation):
    alpha, beta, gamma = orientation[0], orientation[1], orientation[2]
    rot_matrix_x = get_x_rotation_matrix(alpha)
    rot_matrix_y = get_y_rotation_matrix(beta)
    rot_matrix_z = get_z_rotation_matrix(gamma)
    
    return rot_matrix_z.dot( rot_matrix_y.dot( rot_matrix_x ) ) # Rz(gamma)*Rz(beta)*Rx(alpha)

def get_z_rotation_matrix_2d(alpha):
    cosAlpha = cos(alpha)
    sinAlpha = sin(alpha)
    rot_matrix_z = np.array( [[cosAlpha,    -sinAlpha],
                              [sinAlpha,    cosAlpha ]] ).astype(float)
    return rot_matrix_z


def get_2d_rectangle_coordinates(position, size, orientation):
    '''Given the position, the size and the orientation of a rectangle, this function will return the list of its vertices.'''
    vertices = [[0, 1], [0, 0], [1, 0], [1, 1]]
    vertices = np.array(vertices).astype(float)

    # Scaling based on the size
    for i in range(2): vertices[:,i] *= size[i]

    # Rotation based on the orientation
    rotation_matrix = get_z_rotation_matrix_2d(orientation)
    for i in range(vertices.shape[0]):
        vertices[i,:] = rotation_matrix.dot(vertices[i,:])

    # Calculation of the position of the 'reference vertex' given the centroid of the rectangle
    delta = np.array([-size[0]/2, -size[1]/2]).astype(float)
    delta = rotation_matrix.dot( delta )
    position = np.array(position) + delta

    # Translation based on the 'reference vertex'
    vertices += position

    return vertices


def get_3d_cuboid_coordinates(position, size, orientation):
    '''Given the position of the centroid (x,y,z), the size (width, height, depth) and the orientation of a cuboid (alpha, beta, gamma), this function will return the list of the 8 vertices of the corresponding cubiod.'''
    vertices = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [0, 1, 1], [1, 0, 1], [1, 1, 1]] # the list of vertices of a cube centered at the origin and with l=1
    vertices = np.array(vertices, dtype=np.float64)

    # Scaling based on the size
    for i in range(3):
        if i == 2: vertices[:,i] *= -size[i] # for convenience wrt the depth definition
        else: vertices[:,i] *= size[i]

    # Rotation based on the orientation
    rotation_matrix = rpy2rotation_matrix(orientation)
    for i in range(vertices.shape[0]):
            vertices[i,:] = rotation_matrix.dot(vertices[i,:])

    # Calculation of the position of the 'reference vertex' given the centroid of the cuboid
    delta = np.array([-size[0]/2, -size[1]/2, size[2]/2]).astype(float)
    delta = rotation_matrix.dot( delta )
    position = np.array(position) + delta

    # Translation based on the 'reference vertex'
    vertices += position

    return vertices


def calculate_world_dimensions(world_dict):
        x_range = [0, 0] # MIN and MAX value on the x axis
        y_range = [0, 0] # MIN and MAX value on the y axis
        z_range = [0, 0] # MIN and MAX value on the z axis
        
        boxes = world_dict['boxes']
        for obj in boxes:
            pos = boxes[obj]['position']
            size = boxes[obj]['size']
            orientation = boxes[obj]['orientation']
            vertices = get_2d_rectangle_coordinates(pos[:-1], size[:-1], orientation[2])
            
            for v in vertices:
                if v[0] < x_range[0]: x_range[0] = v[0] # new minimum found on the x axis
                if v[0] > x_range[1]: x_range[1] = v[0] # new maximum found on the x axis
                
                if v[1] < y_range[0]: y_range[0] = v[1] # new minimum found on the y axis
                if v[1] > y_range[1]: y_range[1] = v[1] # new maximum found on the y axis
            
            if pos[2] - size[2] < z_range[0]: z_range[0] = pos[2] - size[2] # new minimum found on the z axis
            if pos[2] > z_range[1]: z_range[1] = pos[2] # new maximum found on the z axis
        
        return (x_range, y_range, z_range)
    
    
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
        x = cuboid[:,0]
        y = cuboid[:,1]
        z = cuboid[:,2]
        meshes.append(go.Mesh3d(x=x, y=y, z=z, alphahull=0, name=object))
        
    return meshes