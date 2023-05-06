import numpy as np
import math
import matplotlib.pyplot as plt
import plotly.graph_objects as go

from src.python.utils import json2dict, calculate_world_dimensions, get_z_rotation_matrix_2d, get_2d_rectangle_coordinates

# considering only positive values for the position of the boxes
class MultiLevelSurfaceMap():
    def __init__(self, world_json, resolution):
        self.resolution = resolution
        
        world_dict = json2dict(world_json)
        self.world_dimensions = calculate_world_dimensions(world_dict)
        
        rows = math.ceil((self.world_dimensions[0][1] - self.world_dimensions[0][0])/self.resolution)
        columns = math.ceil((self.world_dimensions[1][1] - self.world_dimensions[1][0])/self.resolution)
        self.discrete_size = (rows, columns)
        
        self.mlsm = [[ None ]*columns for _ in range(rows)]
        self.build(world_dict)
    
    def build(self, world_dict):
        boxes = world_dict['boxes']
        for obj in boxes:
            size = boxes[obj]['size']
            orientation = boxes[obj]['orientation']
            position = boxes[obj]['position'] # position of the centroid of the box
            position[2] += size[2]/2 # translation to have the reference point on the surface of the box
            
            rotation_matrix = get_z_rotation_matrix_2d(orientation[2])
            position_2d = np.array([position[0], position[1]], dtype=np.float64)
            delta_x = 0
            while(delta_x < size[0]/2):
                delta_y = 0
                while(delta_y < size[1]/2):
                    for i in [-1, 1]:
                        for j in [-1, 1]:
                            point = np.array([i*delta_x, j*delta_y], dtype=np.float64) # scaling
                            point = rotation_matrix.dot( point ) # rotation
                            point = point + position_2d # translation
                            map_position_x, map_position_y = self.world2map_coordinates(point[0], point[1]) # coordinates discretization
                            
                            # MLSM population
                            if not self.mlsm[map_position_x][map_position_y]: self.mlsm[map_position_x][map_position_y] = []
                            if((position[2], size[2]) not in self.mlsm[map_position_x][map_position_y]): self.mlsm[map_position_x][map_position_y].append( (position[2], size[2]) )
                    
                    delta_y += self.resolution*0.9
                delta_x += self.resolution*0.9
        
        # Sort elements
        for i in range(self.discrete_size[0]):
            for j in range(self.discrete_size[1]):
                cell = self.mlsm[i][j]
                if cell and len(cell)>1:
                    self.mlsm[i][j].sort(key=lambda cell: cell[0], reverse=True)
            
    def world2map_coordinates(self, x, y):
        '''Given the continous coordinates of a point, returns the indexes of the mlsm cell that contains that point'''
        if(x < self.world_dimensions[0][0] or x >= self.world_dimensions[0][1] or y < self.world_dimensions[1][0] or y >= self.world_dimensions[1][1]): # out of boundaries
            return None
        map_position_x = math.floor((x - self.world_dimensions[0][0]) / self.resolution)
        map_position_y = math.floor((y - self.world_dimensions[1][0]) / self.resolution)
        return (map_position_x, map_position_y)
    
    
    def query(self, x, y): 
        '''Given the continous coordinates of a point, returns the content of the mlsm cell that contains that point'''
        if(x < self.world_dimensions[0][0] or x >= self.world_dimensions[0][1] or y < self.world_dimensions[1][0] or y >= self.world_dimensions[1][1]): # out of boundaries
            return None
        map_position_x, map_position_y = self.world2map_coordinates(x, y)
        return self.mlsm[map_position_x][map_position_y]
    
    def check_collision(self, x, y, z, z_depth=0):
        ''' Checks if a segment (or a point) is in collision with an object in the map.
        x, y, z: point position
        z_depth: lenght of the segment. Zero if you have to check a point
        
        Returns True if there is no collision, False otherwise.
        '''
        levels = self.query(x, y)
        if(not levels): return True
        for level in levels:
            if(z_depth == 0):
                if(z > level[0] - level[1]) and (z < level[0]): 
                    return False
            else:
                if(z >= level[0] and (z - z_depth < level[0] or z - z_depth < level[0] - level[1])) or (z <= level[0] and z >= level[0] - level[1]):
                    return False
        return True
    
    
    def as_numpy(self, stride=1):
        '''Returns the map as numpy array.
        If you want to decrease the resolution of the map change the stride to an higher value.'''
        assert stride >= 1
        x, y, z, depth = [], [], [], []
        for i in range(0, self.discrete_size[0], stride):
            for j in range(0, self.discrete_size[1], stride):
                if (self.mlsm[i][j]):
                    for level in self.mlsm[i][j]:
                        x.append(i*self.resolution + self.world_dimensions[0][0])
                        y.append(j*self.resolution + self.world_dimensions[1][0])
                        z.append(level[0])
                        depth.append(level[1])
        x = np.array(x, dtype=np.float64)
        y = np.array(y, dtype=np.float64)
        z = np.array(z, dtype=np.float64)
        depth = np.array(depth, dtype=np.float64)
        return (x, y, z, depth)
    
    
    def as_showable(self):
        '''for visualization using plotly'''
        x, y, z, depth = self.as_numpy(stride=10)
        scatter = go.Scatter3d( x=x, y=y, z=z, mode='markers', marker=dict(size = 1, color = z, colorscale='Viridis') )
        lines = []
        for i in range(len(depth)):
            if depth[i] < 0.01: continue
            lines.append(go.Scatter3d( x=[x[i],x[i]], y=[y[i],y[i]], z=[z[i],z[i]-depth[i]], mode='lines', marker=dict(size = 1, color = z, colorscale='Viridis') ))
        return (scatter, lines)
    
    
    def plot(self):
        '''[DEPRECATED] for visualization using matplotlib'''
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
        
        for i in range(0, self.discrete_size[0], 1):
            for j in range(0, self.discrete_size[1], 1):
                if (self.mlsm[i][j]):
                    for level in self.mlsm[i][j]:
                        x = [i*self.resolution + self.world_dimensions[0][0], i*self.resolution + self.world_dimensions[0][0]]
                        y = [j*self.resolution + self.world_dimensions[1][0], j*self.resolution + self.world_dimensions[1][0]]
                        z = [level[0], level[0] - level[1]]
                        ax.scatter(x, y, z, c='r', s=10)
                        ax.plot(x, y, z, c='b')