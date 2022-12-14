import numpy as np
import math
import matplotlib.pyplot as plt

from utils import json2dict

# considering only positive values for the position of the boxes
class MultiLevelSurfaceMap():
    def __init__(self, size, world_json, resolution):
        assert size[0] > 0 and size[1] > 1 and resolution > 0
        self.size = size
        self.resolution = resolution
        rows = math.ceil(self.size[0]/self.resolution)
        columns = math.ceil(self.size[1]/self.resolution)
        self.size_discretized = (rows, columns)
        self.mlsm = [[ None ]*columns for _ in range(rows) ]
        self.build(world_json)
    
    def build(self, world_json):
        world = json2dict(world_json)['boxes']
        for obj in world:
            position = world[obj]['position']
            size = world[obj]['size']
            map_position_x = math.ceil(position[0] / self.resolution)
            map_position_y = math.ceil(position[1] / self.resolution)
            
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
        ax.set_xlim([0,self.size[0]])
        ax.set_ylim([0,self.size[0]])
        ax.set_zlim([0,self.size[0]])
        
        for i in range(self.size_discretized[0]):
            for j in range(self.size_discretized[1]):
                if (self.mlsm[i][j]):
                    for level in self.mlsm[i][j]:
                        # point_coords = (i*self.resolution, j*self.resolution, level[0])
                        # ax.scatter(point_coords[0], point_coords[1], point_coords[2], c='b', s=10)
                        # ax.scatter(point_coords[0], point_coords[1], point_coords[2] - level[1], marker='x', c='r', s=10)
                        x = [i*self.resolution, i*self.resolution]
                        y = [j*self.resolution, j*self.resolution]
                        z = [level[0], level[0] - level[1]]
                        ax.scatter(x, y, z, c='r', s=10)
                        ax.plot(x, y, z, c='b')
            # ax.scatter(points_x, points_y, points_z, c='r', s=10)
            # ax.plot(points_x, points_z, color='b')

if __name__ == '__main__':
    map = MultiLevelSurfaceMap((15,15), 'data/world.json', 1)
    map.plot()

    plt.show()