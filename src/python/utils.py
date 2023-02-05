import json
from math import cos, sin
import numpy as np

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


def calculate_world_dimensions(world_dict):
        x_range = [0, 0] # MIN and MAX value on the x axis
        y_range = [0, 0] # MIN and MAX value on the y axis
        z_range = [0, 0] # MIN and MAX value on the z axis
        
        boxes = world_dict['boxes']
        for obj in boxes:
            pos = boxes[obj]['position']
            size = boxes[obj]['size']
            orientation = boxes[obj]['orientation'][2]
            
            # TODO take orientation into account 
            if pos[0] < x_range[0]: x_range[0] = pos[0] # new minimum found on the x axis
            if pos[0] + size[0] > x_range[1]: x_range[1] = pos[0] + size[0] # new maximum found on the x axis
            
            if pos[1] < y_range[0]: y_range[0] = pos[1] # new minimum found on the y axis
            if pos[1] + size[1] > y_range[1]: y_range[1] = pos[1] + size[1] # new maximum found on the y axis
            
            if pos[2] - size[2] < z_range[0]: z_range[0] = pos[2] - size[2] # new minimum found on the z axis
            if pos[2] > z_range[1]: z_range[1] = pos[2] # new maximum found on the z axis
        
        return (x_range, y_range, z_range)