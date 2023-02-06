import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy import transpose,  cos ,sin
import random


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
res = 0.5

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

    def is_Child(self, parent_node):
        parent_node.children.append(self)
        if parent_node.f_swg_id == 'Right':
            self.f_swg_id = 'Left'
        if parent_node.f_swg_id == 'Left':
            self.f_swg_id = 'Right'

    
    def is_parent(self, child_node):
        child_node.parent = self







def RRT(initial_Stance, goal, map, time_max):
    """
    Initial_Stance = a list of 2 elements [f_swg_ini, f_sup_ini]
    Goal = a final goal region [an (x,y) area of the map]
    Map = multilevel surface map in 3D (x,y,z)
    """
    rrt_tree = Tree(initial_Stance[0], initial_Stance[1])
    x_range, y_range, z_range = map.calculate_world_dimesions()
    #AGGIUNGERE CHECK SU initial_Stance PER VERIFICARE CHE SIA NEI LIMITI DELLA MAPPA
    

    """ Step 1) Selecting a vertex for expansion"""
    for i in range(time_max):
        if goal_Check():
            return # TODO PATH la lista di passi da fare
        
        p_rand = [randint(x_range[0], x_range[1]), randint(y_range[0],y_range[1])] # random point in (x,y)
        distance, v_near = v_near_Generation(rrt_tree.root, p_rand)

        """ Step 2) Generating a candidate vertex"""
        #Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        candidate_swg_f, candidate_sup_f = motion_Primitive_selector(v_near)
        candidate_sup_f[2] = assign_height(candidate_sup_f, v_near.f_swg, map)
        #Before creating the vertex( a node) we need first to check R1 and R2 for candidate support foot

        r1_check =r1_feasibility(candidate_sup_f, map)
        r2_check = r2_feasibility(candidate_sup_f, v_near.f_swg)
        if r1_check == 'Not_Feasible':
            pass # The current expansion attempt is aborted and a new iteration is started
        if r2_check == 'Not_Feasible':
            pass

        """ Step 3) Choosing a parent"""
        # DEVO: DEFINIRE LA FUNZIONE NEIGHBORHOOD CHE DEFINISCE I NODI VICINI AD UN NODO,
        #       DEFINIRE LA FUNZIONE DI COSTO DI UN VERTEX





        
            







def v_near_Generation(node, p_rand):
    best_distance = vertex_to_Point_Distance(node, p_rand)
    v_near = node
    if len(node.children) == 0:
        return best_distance, v_near
    
    for child in node.children:
        child_distance, child_v_near = v_near_Generation(child, p_rand)
        if child_distance < best_distance:
            best_distance = child_distance
            v_near = child_v_near
            
    return best_distance, v_near


def vertex_to_Point_Distance(vertex, point, k = 3):
    
    mid_point = [abs((vertex.f_swg[0] + vertex.f_sup[0])/2), abs((vertex.f_swg[1] + vertex.f_sup[1])/2)]
    Saggital_axis = (vertex.f_swg[3] + vertex.f_sup[3]) / 2 # Saggital axis expressed as an angle with respect the Xo axis of the originpytoh
    joining_vector = [abs(mid_point[0] - point[0]), abs(mid_point[1] - point[1])]
    phi = np.arctan2(joining_vector[1], joining_vector[0])
    dist = norm((mid_point - point)) + k*abs(Saggital_axis - phi)
    
    return dist


def motion_Primitive_selector(node):
    """
    Set of primitives U: 20 possible swing foot landing w.r.t the actual support foot
    For now lets assume 5 possible forward distances, 2 side distances and 2 possible angles
    The 2 possible angles are: the saggital_axis angle and (the sggital_axis + 30°)
    The 2 possible side distances are: 12 units and 24 units from f_sup along y axis of f_sup, if f_sup is 'Right'
                                      -12 units and -24 units  if f_sup is 'Left
    The 5 possible forward distances are: -10,0,10,20,30 units along x axis of f_sup
    U_l = primitives for f_swg_id = 'Left'
    U_r = primitives for f_swg_id = 'Right
    (Recall: foot dimensions are 12x7 units)
    It return
    """
    swing = node.f_swg
    support = node.f_sup 
    X_sup, Y_sup, Z_sup, Theta_sup = support
    Saggital_axis = (swing[3] + support[3]) / 2 
    s = Saggital_axis

    U_l = [[X_sup + 30, Y_sup + 12, 0, s], [X_sup + 20, Y_sup + 12, 0, s], [X_sup + 10, Y_sup + 12, 0, s], [X_sup , Y_sup + 12, 0, s], [X_sup -10, Y_sup + 12, 0, s],
           [X_sup + 30, Y_sup + 12, 0, s + (pi/6)], [X_sup + 20, Y_sup + 12, 0, s + (pi/6)], [X_sup + 10, Y_sup + 12, 0, s + (pi/6)], [X_sup , Y_sup + 12, 0, s + (pi/6)], [X_sup -10, Y_sup - 12, 0, s + (pi/6)],
           [X_sup + 30, Y_sup - 24, 0, s], [X_sup + 20, Y_sup - 24, 0, s], [X_sup + 10, Y_sup - 24, 0, s], [X_sup , Y_sup - 24, 0, s], [X_sup -10, Y_sup - 24, 0, s],
           [X_sup + 30, Y_sup - 24, 0, s + (pi/6)], [X_sup + 20, Y_sup - 24, 0, s + (pi/6)], [X_sup + 10, Y_sup - 24, 0, s + (pi/6)], [X_sup , Y_sup - 24, 0, s + (pi/6)], [X_sup -10, Y_sup - 24, 0, s + (pi/6)]
            ]

    U_r = [[X_sup + 30, Y_sup - 12, 0, s], [X_sup + 20, Y_sup - 12, 0, s], [X_sup + 10, Y_sup - 12, 0, s], [X_sup , Y_sup - 12, 0, s], [X_sup -10, Y_sup - 12, 0, s],
           [X_sup + 30, Y_sup - 12, 0, s + (pi/6)], [X_sup + 20, Y_sup - 12, 0, s + (pi/6)], [X_sup + 10, Y_sup - 12, 0, s + (pi/6)], [X_sup , Y_sup - 12, 0, s + (pi/6)], [X_sup -10, Y_sup - 12, 0, s + (pi/6)],
           [X_sup + 30, Y_sup - 24, 0, s], [X_sup + 20, Y_sup - 24, 0, s], [X_sup + 10, Y_sup - 24, 0, s], [X_sup , Y_sup - 24, 0, s], [X_sup -10, Y_sup - 24, 0, s],
           [X_sup + 30, Y_sup - 24, 0, s + (pi/6)], [X_sup + 20, Y_sup - 24, 0, s + (pi/6)], [X_sup + 10, Y_sup - 24, 0, s + (pi/6)], [X_sup , Y_sup - 24, 0, s + (pi/6)], [X_sup -10, Y_sup - 24, 0, s + (pi/6)]
            ]
                    

    if node.f_swg_id == 'Left':
        new_support_foot = random.choice(U_l)
        pass
    if node.f_swg_id == 'Right':
        new_support_foot = random.choice(U_r )
        pass

    new_swing_foot = support
    
    return new_swing_foot, new_support_foot


def goal_Check(info):
    
    pass

def assign_height(actual_footprint, previous_footprint, map):
    h_prev = previous_footprint[2]
    tuples = map.mlsm[actual_footprint[0]][actual_footprint[1]]
    h_actual = tuples[0][0]
    for v in tuples: # choose the smallest height difference for actual_footprint position in the map
        if abs(v[0] - h_prev) < abs(h_actual - h_prev):
            h_actual = v[0]
    return h_actual



    pass


def Feasibility_check(f):
    #Conditions for feasibility
    #R1: f fully in contact
    #R2: stance feasibility, f is kinematically admissible from previous footstep 
    #R3: 
    pass

def r1_feasibility(f, map):
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
            height = map.mlsm[position[0].astype(int)][position[1].astype(int)][0]
        else:
            if map.mlsm[position[0].astype(int)][position[1].astype(int)][0] == height:
                pass
            else:
                R1_feasibility = 'Not_Feasible'
                return R1_feasibility 
    R1_feasibility = 'Feasible'
    return R1_feasibility


def r2_feasibility(f, f_prev):
    """
    This verifies that the footstep f is kinematically admissibile from the previous footstep f_prev
    The constraints on the 4 dimensions of a footstep( R3 x SO(2)) are chosen by construction, based on 
    dimensions of the robot and the environment. Deltas are the maximum increments acceptable.
    """
    delta_x_min = 1/res
    delta_x_max = 1/res
    delta_y_min = 1/res
    delta_y_max = 1/res
    delta_z_min = 1/res
    delta_z_max = 1/res
    delta_theta_min = 1/res
    delta_theta_max = 1 /res
    l = 1/res #NON MI È CHIARO CHE PARAMETRO SIA. FORSE LA DISTANZA STANDARD LUNGO L'ASSE Y TRA IL PIEDE DESTRO E SINISTRO. CHIEDERE  MICHELE

    rotation_matrix = np.array(([cos(f_prev[3]),-sin(f_prev[3])], [sin(f_prev[3]), cos(f_prev[3])]))
    vars= np.array([f[0] - f_prev[0], [f(1) - f_prev(1)]]) # 2x1 matrix with x and y values
    xy_pos = np.matmul(rotation_matrix, vars) + np.array([[0], [+l]]) # positive l
    xy_neg = np.matmul(rotation_matrix, vars) + np.array([[0], [-l]]) #negative l
    z = f[2] - f_prev[2]
    theta = f[3] - f_prev[3]

    if ((xy_pos[0] < - delta_x_min) or (xy_pos > delta_x_max)): # For x the posutive case is enough since it has 0 in the summed vector
        return 'Not_Feasible'
    if ((xy_pos[1] < -delta_y_min) or (xy_pos[1] > delta_y_max)):
        return 'Not_Feasible'
    if ((xy_neg[1] < -delta_y_min) or (xy_neg[1] > delta_y_max)):
        return 'Not_Feasible'
    if ((z < -delta_z_min) or (z > delta_z_max)):
        return 'Not_Feasible'
    if ((theta < -delta_theta_min) or (theta > delta_theta_max)):
        return 'Not_Feasible'
    else:
        return 'Feasible'