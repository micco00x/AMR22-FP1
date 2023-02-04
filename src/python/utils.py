import json
from math import cos, sin
import numpy as np

def json2dict(json_file):
    dict = {}
    with open(json_file) as file:
        dict = json.load(file)
    return dict

def rpy2rotation_matrix(orientation):
    alpha, beta, gamma = orientation[0], orientation[1], orientation[2]
    c_alpha = cos(alpha)
    s_alpha = sin(alpha)
    c_beta = cos(beta)
    s_beta = sin(beta)
    c_gamma = cos(gamma)
    s_gamma = sin(gamma)
    
    Rx = np.array( [[1,    0,           0      ],
                    [0,    c_alpha,    -s_alpha],
                    [0,    s_alpha,    c_alpha ]] ).astype(float)
    
    Ry = np.array( [[c_beta,    0,    s_beta],
                    [0,         1,    0     ],
                    [-s_beta,   0,    c_beta]] ).astype(float)
    
    Rz = np.array( [[c_gamma,   -s_gamma,   0],
                    [s_gamma,   c_gamma,    0],
                    [0,         0,          1]] ).astype(float)
    
    R = Rz.dot( Ry.dot( Rx ) )
    # R = np.array( [[c_gamma*c_beta,   c_gamma*s_beta*s_alpha - s_gamma*c_alpha,   c_gamma*s_beta*c_alpha + s_gamma*s_alpha],
    #                [s_gamma*c_beta,   s_gamma*s_beta*s_alpha + c_gamma*c_alpha,   s_gamma*s_beta*c_alpha - c_gamma*s_alpha],
    #                [-s_beta,          c_beta*s_alpha,                             c_beta*c_alpha                         ]] ).astype(float)
    return R

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