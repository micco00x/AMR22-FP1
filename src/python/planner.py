import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy import transpose,  cos ,sin
import random
from src.python.multi_level_surface_map import MultiLevelSurfaceMap
from src.python.utils import get_2d_rectangle_coordinates, get_z_rotation_matrix

"""
Stance = (f_swing, f_support)
Footstep f = [X_f, Y_f, Z_f, Theta_f]

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
    def __init__(self, f_swg, f_sup, cost = 0, parent = None, trajectory = []):
        self.f_swg = f_swg
        self.f_sup = f_sup
        self.parent = parent
        self.children = []
        self.cost = cost
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
    x_range, y_range = map.discrete_size
    # #AGGIUNGERE CHECK SU initial_Stance PER VERIFICARE CHE SIA NEI LIMITI DELLA MAPPA
    if goal_Check(rrt_tree.root, goal, map) is True:
        print('PATH FOUND')
        return rrt_tree
    

    for i in range(time_max):
        print('Iteration: ', i)
        """ Step 1) Selecting a vertex for expansion"""
        p_rand = [randint(0, x_range), randint(0,y_range)] # random point in (x,y)
        distance, v_near = v_near_Generation(rrt_tree.root, p_rand)

        # print(v_near.f_sup)
        # print(v_near.f_swg)
        # print(v_near.f_swg_id)
        
        """ Step 2) Generating a candidate vertex"""
        #Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        candidate_swg_f, candidate_sup_f, candidate_id = motion_Primitive_selector(v_near)
        #print('candidate_sup-fot type:', type(candidate_sup_f), candidate_sup_f)
        candidate_sup_f[2] = assign_height( v_near.f_swg, candidate_sup_f, map)
        if candidate_sup_f[2] == None:
            continue
        print(candidate_sup_f[2])
        #Before creating the vertex( a node) we need first to check R1 and R2 for candidate support foot

        r1_check = r1_feasibility(candidate_sup_f, map)
        r2_check = r2_feasibility(v_near.f_swg, candidate_sup_f)
        if r1_check == False:
           print('r1_check fail')
           continue # The current expansion attempt is aborted and a new iteration is started
        if r2_check == False:
            print('r2:check fail')
            continue
        v_candidate = Node(candidate_swg_f, candidate_sup_f)

        """ Step 3) Choosing a parent"""
        print('VABENE')
        neighbors = neighborhood(v_candidate, rrt_tree.root)
        candidate_parent = v_near
        candidate_cost = cost_of_a_new_vertex(v_candidate, candidate_parent) ### PER ORA IL COSTO PER PASSARE DA UN NODO AD UN ALTRO È 1, VA CAMBIATO, COSÌ È NAIVE
        for neigh in neighbors:
            if r2_feasibility( neigh.f_swg, candidate_sup_f): ###HERE WE MUST ADD ALSO R3 FEASIBILITY
                if (cost_of_a_new_vertex(v_candidate, neigh))  < candidate_cost:
                    candidate_parent = neigh
                    candidate_cost = cost_of_a_new_vertex(v_candidate, neigh)
        #now let's add the node to the tree, POI QUESTO PASSAGGIO VA FATTO DOPO IL PUNTO 4, ORA È UN TEST
        v_candidate.parent = candidate_parent
        v_candidate.cost = candidate_parent.cost + 1 #candidate_cost
        v_candidate.f_swg_id = candidate_id
        candidate_parent.children.append(v_candidate)
        print( 'NEW FOOTSTEP',map.world2map_coordinates(v_candidate.f_sup[0], v_candidate.f_sup[1]), v_candidate.f_sup[3])
        print('FOOTSTEP ID:', v_candidate.f_swg_id )


        if goal_Check(v_candidate, goal, map) is True:
            print('PATH FOUND')
            return rrt_tree
                      

    print('ok')
        # if goal_Check(f_sup, goal, mlsm):
        #     return # TODO PATH la lista di passi da fare

    return retrieve_steps(rrt_tree.root, [(rrt_tree.root.f_sup, 'Left' if rrt_tree.root.f_swg_id == 'Right' else 'Right') ] )



def retrieve_steps(node, steps):
    steps.append( (node.f_swg, node.f_swg_id) )
    for child in node.children:
        steps = retrieve_steps(child, steps)
    return steps


def goal_Check(node, goal, map):
    f = node.f_sup
    f_x, f_y = map.world2map_coordinates(f[0], f[1])
    for goal_point in goal:
        if f_x == goal_point[0] and f_y == goal_point[1]: # and f[2] == goal[2]:
            return True
    return False # Result is in MAP COORDS            







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

def neighborhood(vertex, tree, r_neigh = 3):
    # neighbors is a list with all the node of the tree that have a footstep_to_footstep metric < r_neigh w.r.t VERTEX
    neighbors = []
    if len(tree.children) == 0:
        metric = footstep_to_footstep_metric(vertex.f_sup, tree.f_sup)
        if metric < r_neigh:
            neighbors.append(tree)
        return neighbors
    for child in tree.children:
        list_of_neighbors = neighborhood(vertex, child)
        for neigh in list_of_neighbors:
            neighbors.append(neigh)
    return neighbors

def cost_of_a_new_vertex(vertex, candidate_parent):
    """ Returns an integer >= 0 as a cost to reach the vertex from the root"""
    cost = candidate_parent.cost + 1 # Da inserire metrica tra vertex e parent AL POSTO DI 1
    return cost



    



def motion_Primitive_selector(node):
    """
    Set of primitives U: 20 possible swing foot landing w.r.t the actual support foot
    For now lets assume 5 possible forward distances, 2 side distances and 2 possible angles
    The 2 possible angles are: the saggital_axis angle and (the sggital_axis + 30°)
    The 2 possible side distances are: -12 units and -24 units from f_sup along y axis of f_sup, if f_swg is 'Right'
                                      12 cm and 24 cm  if f_swg is 'Left
    The 5 possible forward distances are: -10,0,10,20,30 cm along x axis of f_sup
    U_l = primitives for f_swg_id = 'Left'
    U_r = primitives for f_swg_id = 'Right
    (Recall: foot dimensions are 12x7 units)
    
    """
    swing = node.f_swg
    support = node.f_sup
    saggital_axis = (swing[3] + support[3]) / 2 
    s = saggital_axis
    rot = get_z_rotation_matrix(saggital_axis)
    swing[:-1] = rot.dot(swing[:-1])
    support[:-1] = rot.dot(support[:-1])
    
    x_sup, y_sup, z_sup, theta_sup = support

    # x = 1 if node.f_swg_id == 'Left' else -1

    new_id = 'Right' if node.f_swg_id == 'Left' else 'Left'

    p_x = [-0.10, 0, 0.10, 0.20, 0.30, 0.40]
    p_y = [ -0.24, -0.12, 0, 0.12, 0.24]
    p_th = [-(pi/6), 0, +(pi/6)]

    new_support_foot = [ x_sup + random.choice(p_x), y_sup + random.choice(p_y), 0, theta_sup + random.choice(p_th)]


    inverse_rot = np.linalg.inv(rot)
    new_support_foot[:-1] = inverse_rot.dot(new_support_foot[:-1])
    for i in range(0, 4):
        new_support_foot[i] = round(new_support_foot[i], 5)
    

    # if node.f_swg_id == 'Left':
    #     new_support_foot = random.choice(U_l)
    #     new_id = 'Right'
    #     #print('new_support_foot:', type(new_support_foot), '  ',new_support_foot)

    # if node.f_swg_id == 'Right':
    #     new_support_foot = random.choice(U_r )
    #     new_id = 'Left'
    #     #print('new_support_foot:', type(new_support_foot),'  ',new_support_foot)


    new_swing_foot = node.f_sup
    
    return new_swing_foot, new_support_foot, new_id #Result is in FOOT COORDS

    

def assign_height(previous_footprint, actual_footprint, map):
    h_prev = previous_footprint[2]
    #print('h_prev', h_prev)
    actual_cell = map.query(actual_footprint[0], actual_footprint[1]) #This contains the tuples of obejcts heights
    if actual_cell == None:
        return None
    #print(actual_cell)
    for i,tuple in enumerate(actual_cell):
        #print('VALORE ASSOLUTO ALTEZZA: ',abs(h_prev - tuple[0]))
        if abs(h_prev - tuple[0]) < 0.25: #Se trovs un'altezza fattibilr la assegna ad h_actual
            h_actual = tuple[0]
            #print('ALYEZZA ASSEGNATA: ', actual_cell[i])
            for k in range(i+1, len(actual_cell)): #Dopo averla ssegnata prova con quelle rimanenti per vedere se sono meglio
                if abs(h_prev - actual_cell[k][0]) < 0.30:
                    if actual_cell[k][0] > h_actual:
                        #print('ALLELUJA') #DA CAMBIARE IL SEGNO(metterlo minore) SE IL NOSTRO GOAL È PIÙ IN BASSO rispetto a dove partiamo
                        h_actual = actual_cell[k][0]
            return h_actual
    return None # questo caso è una sorta di r2 check solo sull' altezza, vuol dire che non cè nessun altezza possbiile da assegnare



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
    orientation = f[3]
    size = [0.26, 0.14] #Foot size of the robot
    vertices = get_2d_rectangle_coordinates([x,y], size, orientation)
    #print('Vetices: ', vertices)

    for ver in vertices:
        cell = map.query(ver[0], ver[1])
        if cell == None:
            return False
        if  any((obj[0] >= (f[2]) and obj[0] <= (f[2]) ) for obj in cell):
            continue
        else:
            return False
    #print('Ce la fa')
    return  True


def r2_feasibility( f_prev, f_actual):
    """
    This verifies that the footstep f is kinematically admissibile from the previous footstep f_prev
    The constraints on the 4 dimensions of a footstep( R3 x SO(2)) are chosen by construction, based on 
    dimensions of the robot and the environment. Deltas are the maximum increments acceptable.
    """
    delta_x_neg = 0.45
    delta_x_pos = 0.45
    delta_y_neg = 100.30
    delta_y_pos = 100.30
    delta_z_neg = 0.25
    delta_z_pos = 0.25
    delta_theta_neg = 3
    delta_theta_pos = 3
    l = 0.1 #NON MI È CHIARO CHE PARAMETRO SIA. FORSE LA DISTANZA STANDARD LUNGO L'ASSE Y TRA IL PIEDE DESTRO E SINISTRO. CHIEDERE  MICHELE

    rotation_matrix = np.array(([cos(f_prev[3]),-sin(f_prev[3])], [sin(f_prev[3]), cos(f_prev[3])]))
    vars= np.array([[f_actual[0] - f_prev[0]], [f_actual[1] - f_prev[1]]]) # 2x1 matrix with x and y values
    xy_pos = np.matmul(rotation_matrix, vars) + np.array([[0], [+l]]) # positive l
    #print('xy_pos type:',type(xy_pos), xy_pos[0], xy_pos[1])
    xy_neg = np.matmul(rotation_matrix, vars) + np.array([[0], [-l]]) #negative l
    #print('f type', type(f[2]), '     ', 'f_prev type:', type(f_prev[2]) )
    z = f_actual[2] - f_prev[2]
    theta = f_actual[3] - f_prev[3]
    #print(type(xy_pos[1]), xy_pos[1])

    if ((xy_pos[0] < - delta_x_neg) or (xy_pos[0] > delta_x_pos)): # For x the posutive case is enough since it has 0 in the summed vector
        #print('X fail, difference is',xy_pos[0]  )
        return False
    if ((xy_pos[1] < (-1*delta_y_neg)) or (xy_pos[1] > delta_y_pos)):
        #print('Y fail_PLUS, difference is', xy_pos[1],  'delta_y_neg', delta_y_neg, 'delta_y_pos', delta_y_pos )
        return False
    if ((xy_neg[1] < (-1*delta_y_neg)) or (xy_neg[1] > delta_y_pos)):
        #print('Y fail_NEG, difference is', xy_pos[1] , 'delta_y_neg', delta_y_neg, 'delta_y_pos', delta_y_pos)
        return False
    if ((z < (-1*delta_z_neg)) or (z > delta_z_pos)):
       # print('Z fail')
        return False
    if ((theta < (-1*delta_theta_neg)) or (theta > delta_theta_pos)):
        #print('Theta fail')
        return False
    else:
        return True