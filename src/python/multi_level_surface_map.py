import numpy as np
import math
import matplotlib.pyplot as plt

from utils import json2dict

# considering only positive values for the position of the boxes
class MultiLevelSurfaceMap():
    def __init__(self, world_json, resolution):
        self.resolution = resolution
        
        self.world_dimensions = self.calculate_world_dimensions(world_json)
        rows = math.ceil((self.world_dimensions[0][1] - self.world_dimensions[0][0])/self.resolution)
        columns = math.ceil((self.world_dimensions[1][1] - self.world_dimensions[1][0])/self.resolution)
        self.discrete_size = (rows, columns)
        
        self.mlsm = [[ None ]*columns for _ in range(rows) ]
        self.build(world_json)
    
    def calculate_world_dimensions(self, world_json):
        x_range = [0, 0] # MIN and MAX value on the x axis
        y_range = [0, 0] # MIN and MAX value on the y axis
        z_range = [0, 0] # MIN and MAX value on the z axis
        
        world = json2dict(world_json)['boxes']
        for obj in world:
            pos = world[obj]['position']
            size = world[obj]['size']
            if pos[0] < x_range[0]: x_range[0] = pos[0] # new minimum found on the x axis
            if pos[0] + size[0] > x_range[1]: x_range[1] = pos[0] + size[0] # new maximum found on the x axis
            
            if pos[1] < y_range[0]: y_range[0] = pos[1] # new minimum found on the y axis
            if pos[1] + size[1] > y_range[1]: y_range[1] = pos[1] + size[1] # new maximum found on the y axis
            
            if pos[2] - size[2] < z_range[0]: z_range[0] = pos[2] - size[2] # new minimum found on the z axis
            if pos[2] > z_range[1]: z_range[1] = pos[2] # new maximum found on the z axis
        
        return (x_range, y_range, z_range)
        
    def build(self, world_json):
        world = json2dict(world_json)['boxes']
        for obj in world:
            position = world[obj]['position']
            size = world[obj]['size']
            map_position_x = math.ceil((position[0] - self.world_dimensions[0][0]) / self.resolution)
            map_position_y = math.ceil((position[1] - self.world_dimensions[1][0]) / self.resolution)
            
            for i in range(map_position_x, map_position_x + int(size[0]//self.resolution) ):
                for j in range(map_position_y, map_position_y + int(size[1]//self.resolution) ):
                    if not self.mlsm[i][j]: self.mlsm[i][j] = []
                    self.mlsm[i][j].append( (position[2], size[2]) )
                    
    def plot(self):
        plt.figure('Multi Level Surface Map')
        ax = plt.subplot(projection='3d')
        
         # Axes labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # Visualization boundaries:
        ax.set_xlim([self.world_dimensions[0][0], self.world_dimensions[0][1]])
        ax.set_ylim([self.world_dimensions[1][0], self.world_dimensions[1][1]])
        ax.set_zlim([self.world_dimensions[2][0], self.world_dimensions[2][1]])
        
        for i in range(self.discrete_size[0]):
            for j in range(self.discrete_size[1]):
                if (self.mlsm[i][j]):
                    for level in self.mlsm[i][j]:
                        # point_coords = (i*self.resolution, j*self.resolution, level[0])
                        # ax.scatter(point_coords[0], point_coords[1], point_coords[2], c='b', s=10)
                        # ax.scatter(point_coords[0], point_coords[1], point_coords[2] - level[1], marker='x', c='r', s=10)
                        x = [i*self.resolution + self.world_dimensions[0][0], i*self.resolution + self.world_dimensions[0][0]]
                        y = [j*self.resolution + self.world_dimensions[1][0], j*self.resolution + self.world_dimensions[1][0]]
                        z = [level[0], level[0] - level[1]]
                        ax.scatter(x, y, z, c='r', s=10)
                        ax.plot(x, y, z, c='b')
            # ax.scatter(points_x, points_y, points_z, c='r', s=10)
            # ax.plot(points_x, points_z, color='b')

if __name__ == '__main__':
    map = MultiLevelSurfaceMap((15,15), 'data/world.json', 1)
    map.plot()

    plt.show()