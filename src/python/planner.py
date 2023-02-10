import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy import transpose,  cos ,sin
import random
from multi_level_surface_map import MultiLevelSurfaceMap
from utils import get_2d_rectangle_coordinates

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
    def __init__(self, f_swg, f_sup, parent = None, trajectory = []):
        self.f_swg = f_swg
        self.f_sup = f_sup
        self.parent = parent
        self.children = []
        self.f_swg_id = 'Right' # It always starts by moving right foot first
        self.trajectory = trajectory # Trajectory is the trajectory that , starting from the parent, brings to the actual node
    
    # def expand(self, new_f_swg, new_f_sup):
    #     new_node = Node(new_f_swg, new_f_sup)
    #     new_node.parent = self
    #     if self.f_swg_id == 'Right':
    #         new_node.f_swg_id = 'Left'
    #     if self.f_swg_id == 'Left':
    #         new_node.f_swg_id ='Right'

    #     self.children = [self.children, new_node]

    # def is_Child(self, parent_node):
    #     parent_node.children.append(self)
    #     if parent_node.f_swg_id == 'Right':
    #         self.f_swg_id = 'Left'
    #     if parent_node.f_swg_id == 'Left':
    #         self.f_swg_id = 'Right'

    
    # def is_parent(self, child_node):
    #     child_node.parent = self


    def __eq__(self, other):
        if (self.f_swg == other.f_swg) and (self.f_sup == other.f_sup):
            return True
        if (self.f_swg == other.f_sup) and (self.f_sup == other.f_swg):
            return True
        return False






def RRT(initial_Stance, goal, map, time_max):
    """
    Initial_Stance = a list of 2 elements [f_swg_ini, f_sup_ini]
    Goal = a final goal region [an (x,y) area of the map]
    Map = multilevel surface map in 3D (x,y,z)
    """
    rrt_tree = Tree(initial_Stance[0], initial_Stance[1])
    x_range = map.rows
    y_range = map.columns
    #AGGIUNGERE CHECK SU initial_Stance PER VERIFICARE CHE SIA NEI LIMITI DELLA MAPPA
    if goal_Check(rrt_tree.root, goal, map) is True:
        print('PATH FOUND')
        return rrt_tree
    
    
    for i in range(time_max):
        """ Step 1) Selecting a vertex for expansion"""
        p_rand = [randint(0, x_range), randint(0,y_range)] # random point in (x,y)
        distance, v_near = v_near_Generation(rrt_tree.root, p_rand)

        """ Step 2) Generating a candidate vertex"""
        #Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        candidate_swg_f, candidate_sup_f = motion_Primitive_selector(v_near)
        #print('candidate_sup-fot type:', type(candidate_sup_f), candidate_sup_f)
        candidate_sup_f[2] = assign_height(candidate_sup_f, v_near.f_swg, map)
        if candidate_sup_f[2] == False: # there isn't any object or surface in this point, so discard
            pass
        #Before creating the vertex( a node) we need first to check R1 and R2 for candidate support foot

        r1_check = r1_feasibility(candidate_sup_f, map)
        r2_check = r2_feasibility(candidate_sup_f, v_near.f_swg)
        if r1_check == False:
            pass # The current expansion attempt is aborted and a new iteration is started
        if r2_check == False:
            pass

        """ Step 3) Choosing a parent"""
        # DEVO: DEFINIRE LA FUNZIONE NEIGHBORHOOD CHE DEFINISCE I NODI VICINI AD UN NODO,
        #       DEFINIRE LA FUNZIONE DI COSTO DI UN VERTEX
    
    print('ok')
        # if goal_Check(f_sup, goal, mlsm):
        #     return # TODO PATH la lista di passi da fare


        
            







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
    
    mid_point = np.array([abs((vertex.f_swg[0] + vertex.f_sup[0])/2), abs((vertex.f_swg[1] + vertex.f_sup[1])/2)])
    Saggital_axis = np.array((vertex.f_swg[3] + vertex.f_sup[3]) / 2) # Saggital axis expressed as an angle with respect the Xo axis of the originpytoh
    joining_vector =np.array([abs(mid_point[0] - point[0]), abs(mid_point[1] - point[1])])
    phi = np.arctan2(joining_vector[1], joining_vector[0])
    dist = norm((mid_point - point)) + k*abs(Saggital_axis - phi)
    
    return dist # Result is in FOOT COORDS

def footstep_to_footstep_metric(f1, f2, kj = 0.4):
    p = np.array([f1[0] - f2[0], f1[1] - f2[1], f1[2] - f2[2]])
    angle = np.array([f1[3] - f2[3]])
    metric = norm(p) + kj*norm(angle)
    return metric # Result is in FOOT COORDS

def neighborhood(vertex,node, r_neigh = 3):
    # neighbors is a list with all the node of the tree that have a footstep_to_footstep metric < r_neigh w.r.t VERTEX
    neighbors = []
    if len(node.children) == 0:
        metric = footstep_to_footstep_metric(vertex.f_sup, node.f_sup)
        if metric < r_neigh:
            neighbors.append(node)
        return neighbors
    for child in node.children:
        list_of_neighbors = neighborhood(vertex, child)
        for neigh in list_of_neighbors:
            neighbors.append(neigh)
    return neighbors

def cost_of_a_vertex():
    pass



    



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
    
    """
    swing = node.f_swg
    support = node.f_sup 
    X_sup, Y_sup, Z_sup, Theta_sup = support
    Saggital_axis = (swing[3] + support[3]) / 2 
    s = Saggital_axis

    U_l = [[X_sup + 0.30, Y_sup + 0.12, 0, s], [X_sup + 0.20, Y_sup + 0.12, 0, s], [X_sup + 0.10, Y_sup + 0.12, 0, s], [X_sup , Y_sup + 0.12, 0, s], [X_sup -0.10, Y_sup + 0.12, 0, s],
           [X_sup + 0.30, Y_sup + 0.12, 0, s + (pi/6)], [X_sup + 0.20, Y_sup + 0.12, 0, s + (pi/6)], [X_sup + 0.10, Y_sup + 0.12, 0, s + (pi/6)], [X_sup , Y_sup + 0.12, 0, s + (pi/6)], [X_sup -0.10, Y_sup - 0.12, 0, s + (pi/6)],
           [X_sup + 0.30, Y_sup - 0.24, 0, s], [X_sup + 0.20, Y_sup - 0.24, 0, s], [X_sup + 0.10, Y_sup - 0.24, 0, s], [X_sup , Y_sup - 0.24, 0, s], [X_sup -0.10, Y_sup - 0.24, 0, s],
           [X_sup + 0.30, Y_sup - 0.24, 0, s + (pi/6)], [X_sup + 0.20, Y_sup - 0.24, 0, s + (pi/6)], [X_sup + 0.10, Y_sup - 0.24, 0, s + (pi/6)], [X_sup , Y_sup - 0.24, 0, s + (pi/6)], [X_sup -0.10, Y_sup - 0.24, 0, s + (pi/6)]
            ]

    U_r = [[X_sup + 0.30, Y_sup - 0.12, 0, s], [X_sup + 0.20, Y_sup - 0.12, 0, s], [X_sup + 0.10, Y_sup - 0.12, 0, s], [X_sup , Y_sup - 0.12, 0, s], [X_sup -0.10, Y_sup - 0.12, 0, s],
           [X_sup + 0.30, Y_sup - 0.12, 0, s + (pi/6)], [X_sup + 0.20, Y_sup - 0.12, 0, s + (pi/6)], [X_sup + 0.10, Y_sup - 0.12, 0, s + (pi/6)], [X_sup , Y_sup - 0.12, 0, s + (pi/6)], [X_sup -0.10, Y_sup - 0.12, 0, s + (pi/6)],
           [X_sup + 0.30, Y_sup - 0.24, 0, s], [X_sup + 0.20, Y_sup - 0.24, 0, s], [X_sup + 0.10, Y_sup - 0.24, 0, s], [X_sup , Y_sup - 0.24, 0, s], [X_sup -0.10, Y_sup - 0.24, 0, s],
           [X_sup + 0.30, Y_sup - 0.24, 0, s + (pi/6)], [X_sup + 0.20, Y_sup - 0.24, 0, s + (pi/6)], [X_sup + 0.10, Y_sup - 0.24, 0, s + (pi/6)], [X_sup , Y_sup - 0.24, 0, s + (pi/6)], [X_sup -0.10, Y_sup - 0.24, 0, s + (pi/6)]
            ]
                    

    if node.f_swg_id == 'Left':
        new_support_foot = random.choice(U_l)
        #print('new_support_foot:', type(new_support_foot), '  ',new_support_foot)
        pass
    if node.f_swg_id == 'Right':
        new_support_foot = random.choice(U_r )
        #print('new_support_foot:', type(new_support_foot),'  ',new_support_foot)
        pass

    new_swing_foot = support
    
    return new_swing_foot, new_support_foot #Result is in FOOT COORDS


def goal_Check(node, goal, map):
    f = node.f_sup
    f_x, f_y = map.world2map_coordinates(f[0], f[1])
    for goal_point in goal:
        if f_x == goal_point[0] and f_y == goal_point[1]: # and f[2] == goal[2]:
            return True
    return False # Result is in MAP COORDS



    
    
    

def assign_height(actual_footprint, previous_footprint, map):
    h_prev = previous_footprint[2]
    #print('h_prev :', type(h_prev))
    cell = map.query(actual_footprint[0],actual_footprint[1])
    #print('cell type:', type(cell), cell)
    if cell == None:# DA CAMBIARE EVNTUALMENTE IN multi_levele_surface_map.py , I NONE ROMPONO LE SCATOLE
        return False
    h_actual = cell[0][0]
    #print('h_actual type:', type(h_actual))
    for v in cell: # choose the smallest height difference for actual_footprint position in the map
        #print('v[0] type:', type(v[0]), v[0])
        # print('h_prev type:' , type(h_prev))
        # print('h_actual type', type(h_actual))

        if abs(v[0] - h_prev) < abs(h_actual - h_prev):
            h_actual = v[0]
    return h_actual 



def r1_feasibility(f, map):###DA CAMBIAREEEEE: PRIMA CALCOLO DOVE STA IL PIEDE NEL MONDO, POI CAMBIO LE COORDS E VADO NEL
    #SISTEMA DI RIFERIMENTO DELLA MAPPA E SOLO LA VALUTO QUALE QUADRATINI OCCUPA E POI FACCIO IL CHECK
    """
    This verifies that the footstep f is fully in contact within a single horizontal patch.
    To guarantee this, each cell of the map belonging to or overlapping with the footprint
    must have the same height Z. The foorprint is 12x7 units , but let's use 13x9 to overcome
    small errors in postion
    """

    x = f[0]
    y = f[1]
    orientation = f[2]
    size = [0.26, 0.14] #Foot size of the robot
    vertices = get_2d_rectangle_coordinates([x,y], size, orientation)

    for ver in vertices:
        cell = map.query(ver[0], ver[1])
        for obj in cell:
            if obj[0] == f[3]:
                continue
            else:
                return False
    return  True


    # sa = f[3] # Saggital axis
    # rotation_matrix = np.array(([cos(sa),-sin(sa)], [sin(sa), cos(sa)]))
    # x_range = list(range(-6, +6+1))
    # y_range = list(range(-4, +4+1))
    # footstep = np.zeros([2,(len(x_range)*len(y_range))])
    # counter = -1
    # for j in x_range:
    #     for k in y_range:
    #         counter += 1
    #         footstep[:,counter] = np.array([j,k])

    # height = 'start'
    # for i in range(0,footstep.shape[1]):
    #     position = np.rint(np.matmul(rotation_matrix, footstep[:,i]))
    #     position[0] = position[0] + f[0]
    #     position[1] = position[1] + f[1]
    #     #print(footstep[:,i], position)
    #     if height == 'start':
    #         if map.mlsm[position[0].astype(int)][position[1].astype(int)] == None: #SEI FUORI DALLA MAPPA CALPESTABILE , NESSUN OGGETTO
    #             return False
    #         height = map.mlsm[position[0].astype(int)][position[1].astype(int)][0]
    #     else:
    #         if map.mlsm[position[0].astype(int)][position[1].astype(int)] == None: #SEI FUORI DALLA MAPPA CALPESTABILE , NESSUN OGGETTO
    #             return False
    #         if map.mlsm[position[0].astype(int)][position[1].astype(int)][0] == height:
    #             pass
    #         else:
    #             return False
    # return True

def r2_feasibility(f, f_prev):
    """
    This verifies that the footstep f is kinematically admissibile from the previous footstep f_prev
    The constraints on the 4 dimensions of a footstep( R3 x SO(2)) are chosen by construction, based on 
    dimensions of the robot and the environment. Deltas are the maximum increments acceptable.
    """
    delta_x_min = 1
    delta_x_max = 1
    delta_y_min = 1
    delta_y_max = 1
    delta_z_min = 1
    delta_z_max = 1
    delta_theta_min = 1
    delta_theta_max = 1 
    l = 1 #NON MI È CHIARO CHE PARAMETRO SIA. FORSE LA DISTANZA STANDARD LUNGO L'ASSE Y TRA IL PIEDE DESTRO E SINISTRO. CHIEDERE  MICHELE

    rotation_matrix = np.array(([cos(f_prev[3]),-sin(f_prev[3])], [sin(f_prev[3]), cos(f_prev[3])]))
    vars= np.array([[f[0] - f_prev[0]], [f[0] - f_prev[1]]]) # 2x1 matrix with x and y values
    xy_pos = np.matmul(rotation_matrix, vars) + np.array([[0], [+l]]) # positive l
    #print('xy_pos type:',type(xy_pos), xy_pos[0], xy_pos[1])
    xy_neg = np.matmul(rotation_matrix, vars) + np.array([[0], [-l]]) #negative l
    #print('f type', type(f[2]), '     ', 'f_prev type:', type(f_prev[2]) )
    z = f[2] - f_prev[2]
    theta = f[3] - f_prev[3]

    if ((xy_pos[0] < - delta_x_min) or (xy_pos[0] > delta_x_max)): # For x the posutive case is enough since it has 0 in the summed vector
        return False
    if ((xy_pos[1] < -delta_y_min) or (xy_pos[1] > delta_y_max)):
        return False
    if ((xy_neg[1] < -delta_y_min) or (xy_neg[1] > delta_y_max)):
        return False
    if ((z < -delta_z_min) or (z > delta_z_max)):
        return False
    if ((theta < -delta_theta_min) or (theta > delta_theta_max)):
        return False
    else:
        return True