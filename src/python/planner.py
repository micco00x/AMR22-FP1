import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy import transpose,  cos ,sin


"""
Stance = (f_swing, f_support)
Footstep f = (X_f, Y_f, Z_f, Theta_f)

For all the task we will have different reference frames. To go from the robot frame ( or its feet frames) to
the word (immutable) frame it will be used a rotation matrix [[cos(a) -sin(a) 0],[sin(a) cos(a) 0], [0 0 1]] 
where a is the angle that describe the saggital axis, computed as the mean of the yawn angle of the two feet.
The yawn angle of each foot is the 4th element of a footstep f, and is computed with respect the starting position
of the robot. In fact the initial stance of the robot is always with both feet along the x axis of word frame,
therefore both Theta_f = 0
"""
pi = np.pi
resolution = 0.5

class Tree():
    def __init__(self, f_swg_ini, f_sup_ini):
        self.root = Node(f_swg_ini,f_sup_ini)
    


class Node():
    """
    Each node v of the tree is a stance v = (f_swg, f_sup)
    """
    def __init__(self, f_swg, f_sup):
        self.f_swg = f_swg
        self.f_sup = f_sup
        self.parent = None
        self.children = []
        self.f_swg_id = 'Right' # It always starts by moving right foot first
    
    # def expand(self, new_f_swg, new_f_sup):
    #     new_node = Node(new_f_swg, new_f_sup)
    #     new_node.parent = self
    #     if self.f_swg_id == 'Right':
    #         new_node.f_swg_id = 'Left'
    #     if self.f_swg_id == 'Left':
    #         new_node.f_swg_id ='Right'

    #     self.children = [self.children, new_node]

    def Is_Child(self, parent_node):
        parent_node.children.append(self)
        if parent_node.f_swg_id == 'Right':
            self.f_swg_id = 'Left'
        if parent_node.f_swg_id == 'Left':
            self.f_swg_id = 'Right'


    
    def Is_parent(self, child_node):
        child_node.parent = self







def RRT(Initial_Stance, Goal, Map):
    """
    Initial_Stance = a list of 2 elements [f_swg_ini, f_sup_ini]
    Goal = a final goal region [an (x,y) area of the map]
    Map = multilevel surface map in 3D (x,y,z)
    """
    rrt_tree = Tree(Initial_Stance[0], Initial_Stance[1])
    x_range, y_range, z_range = Map.calculate_world_dimesions()
    stop_condition = Goal_Check()

    """ Step 1) Selecting a vertex for expansion"""
    while stop_condition == False: #Untill the goal is reached
        p_rand = [randint(x_range[0], x_range[1]), randint(y_range[0],y_range[1])] # random point in (x,y)
        if rrt_tree.children() == []:
            v_near = rrt_tree
        else:
            v_near = None
            distance = None
            for child in rrt_tree.childern():
                dist = Vertex_to_Point_Distance(child, p_rand)
                if distance == None:
                    distance = dist
                    v_near = child
                else:
                    if dist < distance:
                        distance = dist
                        v_near = child
                    else:
                        pass
        #Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        f_near_swg = v_near[0]
        f_near_sup = v_near[1]



            






    pass


def V_near_Generation(tree, p_rand): 
    v_near = tree
    distance = None
    if tree.children() != []:
        for child in tree.childern():
            v_near = None
            distance, v_near = V_near_Generation(child, p_rand)
    else:
        dist = Vertex_to_Point_Distance(tree, p_rand)
        if v_near == None:
            distance = dist
            v_near = child
        else:
            if dist < distance:
                distance = dist
                v_near = child
            else:
                pass    
            return distance, v_near
            
    return distance, v_near
    # if rrt_tree.children() == []:
    #     v_near = rrt_tree




def Vertex_to_Point_Distance(vertex, point, k = 3):
    
    mid_point = [abs((vertex.f_swg[0] + vertex.f_sup[0])/2), abs((vertex.f_swg[1] + vertex.f_sup[1])/2)]
    Saggital_axis = (vertex.f_swg[3] + vertex.f_sup[3]) / 2 # Saggital axis expressed as an angle with respect the Xo axis of the originpytoh
    joining_vector = [abs(mid_point[0] - point[0]), abs(mid_point[1] - point[1])]
    phi = np.arctan2(joining_vector[1], joining_vector[0])
    dist = norm((mid_point - point)) + k*abs(Saggital_axis - phi)
    
    return dist


def Motion_Primitives(vertex):
    """
    Set of primitives U: 20 possible swing foot landing w.r.t the actual support foot
    For now lets assume 5 possible forward distances, 2 side distances and 2 possible angles
    The 2 possible angles are: the saggital_axis angle and (the sggital_axis + 30°)
    The 2 possible side distances are: 12 units and 24 units from f_sup along y axis of f_sup, if f_sup is 'Right'
                                      -12 units and -24 units  if f_sup is 'Left
    The 5 possible forward distances are: -10,0,10,20,30 units along x axis of f_sup
    U_r = primitives for f_sup = 'Right'
    U_l = primitives for f_sup = 'Left'
    (Recall: foot dimensions are 12x7 units)
    """
    swing = vertex.f_swg
    support = vertex.f_sup 
    X_sup, Y_sup, Z_sup, Theta_sup = support
    Saggital_axis = (vertex.f_swg[3] + vertex.f_sup[3]) / 2 
    s = Saggital_axis

    U_r = [[X_sup + 30, Y_sup + 12, s], [X_sup + 20, Y_sup + 12, s], [X_sup + 10, Y_sup + 12, s], [X_sup , Y_sup + 12, s], [X_sup -10, Y_sup + 12, s],
           [X_sup + 30  , Y_sup + 12, s + (pi/6)], [X_sup + 20, Y_sup + 12, s + (pi/6)], [X_sup + 10, Y_sup + 12, s + (pi/6)], [X_sup , Y_sup + 12, s + (pi/6)], [X_sup -10, Y_sup - 12, s + (pi/6)],
           [X_sup + 30, Y_sup - 24, s], [X_sup + 20, Y_sup - 24, s], [X_sup + 10, Y_sup - 24, s], [X_sup , Y_sup - 24, s], [X_sup -10, Y_sup - 24, s],
           [X_sup + 30  , Y_sup - 24, s + (pi/6)], [X_sup + 20, Y_sup - 24, s + (pi/6)], [X_sup + 10, Y_sup - 24, s + (pi/6)], [X_sup , Y_sup - 24, s + (pi/6)], [X_sup -10, Y_sup - 24, s + (pi/6)]
            ]

    U_l = [[X_sup + 30, Y_sup - 12, s], [X_sup + 20, Y_sup - 12, s], [X_sup + 10, Y_sup - 12, s], [X_sup , Y_sup - 12, s], [X_sup -10, Y_sup - 12, s],
           [X_sup + 30  , Y_sup - 12, s + (pi/6)], [X_sup + 20, Y_sup - 12, s + (pi/6)], [X_sup + 10, Y_sup - 12, s + (pi/6)], [X_sup , Y_sup - 12, s + (pi/6)], [X_sup -10, Y_sup - 12, s + (pi/6)],
           [X_sup + 30, Y_sup - 24, s], [X_sup + 20, Y_sup - 24, s], [X_sup + 10, Y_sup - 24, s], [X_sup , Y_sup - 24, s], [X_sup -10, Y_sup - 24, s],
           [X_sup + 30  , Y_sup - 24, s + (pi/6)], [X_sup + 20, Y_sup - 24, s + (pi/6)], [X_sup + 10, Y_sup - 24, s + (pi/6)], [X_sup , Y_sup - 24, s + (pi/6)], [X_sup -10, Y_sup - 24, s + (pi/6)]
            ]
                    

    if vertex.f_swg_id == 'Right':
        pass
    if vertex.f_swg_id == 'Left':
        pass
    pass


def Goal_Check(info):
    
    pass

def Feasibility_check(f):
    #Conditions for feasibility
    #R1: f fully in contact
    #R2: stance feasibility, f is kinematically admissible from previous footstep 
    #R3: 
    pass

def R1_feasibility(f, Map):
    """
    This verifies that the footstep f is fully in contact within a single horizontal patch.
    To guarantee this, each cell of the map belonging to or overlapping with the footprint
    must have the same height Z. The foorprint is 12x7 units , but let's use 13x9 to overcome
    small errors in postion
    """
    Saggital_axis = (f[3] + f[3]) / 2
    a = Saggital_axis
    rotation_matrix = np.array(([cos(a),-sin(a)], [sin(a), cos(a)]))
    x_range = list(range(-6, +6+1))
    y_range = list(range(-4, +4+1))
    footstep = np.zeros([2,(len(x_range)*len(y_range))])
    counter = -1
    for j in x_range:
        for k in y_range:
            counter += 1
            footstep[:,counter] = np.array([j,k])

    height = None
    for i in range(0,footstep.shape[1]):
        position = np.rint(np.matmul(rotation_matrix, footstep[:,i]))
        position[0] = position[0] + f[0]
        position[1] = position[1] + f[1]
        print(footstep[:,i], position)
        if height == None:
            height = Map.mlsm[position[0].astype(int)][position[1].astype(int)]
        else:
            if Map.mlsm[position[0].astype(int)][position[1].astype(int)] == height:
                pass
            else:
                R1_feasibility = 'Not_Feasible'
                return R1_feasibility 
    R1_feasibility = 'Feasible'
    return R1_feasibility


def R2_feasibility(f):
    pass