import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm

# Stance = (f_swing, f_support)
#Footsetp f = (X_f, Y_f, Z_f, Theta_f)

class Tree():
    def __init__(self, f_swg_ini, f_sup_ini):
        self.root = Node(f_swg_ini,f_sup_ini)
    
    def expand(self,data):
        pass



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
    while stop_condition == False:
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
    For now lets assume 5 possible forward distances, 2 side distances and 2 possible angle 
    U_r = primitives for right foot
    U_l = primitives for left foot
    """
    swing = vertex.f_swg
    support = vertex.f_sup 
    Saggital_axis = (vertex.f_swg[3] + vertex.f_sup[3]) / 2 # PER DEFINIRE IL SAGGITAL AXIS DEVO TOGLIERMI UN DUBBIO SU COME È  MESSO IL SISTEMA DI RIFERIMENTO

    U_r = [[ggg]]

    U_l = []

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

