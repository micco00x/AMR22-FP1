import numpy as np
from numpy import random
from numpy.linalg import norm
import random
import math
import time
from tqdm import tqdm

from src.python.utils import get_2d_rectangle_coordinates, get_z_rotation_matrix, get_z_rotation_matrix_2d
from src.python.parameters import *

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
all_time = []

class Node():
    """
    Each node v of the tree is a stance v = (f_swg, f_sup)
    """
    def __init__(self, f_swg, f_sup, f_swg_id = None, cost = 0, parent = None, trajectory = []):
        self.f_swg = f_swg
        self.f_sup = f_sup
        self.parent = parent
        self.children = []
        self.cost = cost
        self.f_swg_id = f_swg_id # It always starts by moving right foot first
        self.trajectory = trajectory # Trajectory is the trajectory that , starting from the parent, brings to the actual node
    
    def __eq__(self, other):
        if (self.f_swg == other.f_swg) and (self.f_sup == other.f_sup):
            return True
        if (self.f_swg == other.f_sup) and (self.f_sup == other.f_swg):
            return True
        return False




def RRT(initial_Stance, goal_region, map, time_max):
    start_rrt = time.time()
    #time_tot = 0
    """
    Initial_Stance = a list of 2 elements [f_swg_ini, f_sup_ini]
    Goal = a final goal region [an (x,y) area of the map]
    Map = multilevel surface map in 3D (x,y,z)
    """
    rrt_root = Node(initial_Stance[0], initial_Stance[1], f_swg_id='Right')
    x_range, y_range, z_range = map.world_dimensions
    
    if not r2_feasibility(rrt_root.f_sup, rrt_root.f_swg, rrt_root.f_swg_id):
        print('Initial stance NOT feasible!\n')
        return []
            
    if goal_check(rrt_root, goal_region):
        print('Already in the goal region! No steps needed')
        return retrieve_all_steps(rrt_root)
    
    goal_nodes = []
    v_candidate = None
    for i in tqdm(range(time_max)):
        time_step = []
        start_1 = time.time()


        """ Step 1) Selecting a vertex for expansion"""
        if i % 10 == 0:
            p_rand = goal_region[:-1]
        else:
            x_rand = random.random()*abs(x_range[1] - x_range[0]) + x_range[0]
            y_rand = random.random()*abs(y_range[1] - y_range[0]) + y_range[0]
            z_rand = random.random()*abs(z_range[1] - z_range[0]) + z_range[0] # 1.5m added on top and at the bottom to have a better distribution
            p_rand = [x_rand, y_rand, z_rand] # random point in (x,y,z)
        
        start_v_near = time.time()

        distance, v_near = v_near_selection(rrt_root, p_rand, z_range) 
        #v_near = rrt_root if i==0 or not v_candidate else v_candidate
        #print('BEST_DISTANCE:',distance)

        time_step.append(round(time.time()-start_v_near, 6)) #append time for v_near selection
        #time_step.append(round(end_v_near-start_v_near, 6)) #append time for v_near selection


        time_step.append(round(time.time()-start_1, 6)) #append time for step 1
        #time_step.append(round(end-start, 6)) #append time for step 1
        
        start_2 = time.time()    
        """ Step 2) Generating a candidate vertex"""
        # Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        candidate_swg_f, candidate_sup_f, candidate_id = motion_primitive_selector(v_near) #candidate_id è il piede di swing del nodo candiate
        candidate_sup_f[2] = assign_height(v_near.f_sup, candidate_sup_f, map)
        if candidate_sup_f[2] == None:
            time_step.append(0)
            time_step.append(round(time.time() - start_2, 6))
            time_step.append(0)
            continue
        # Before creating the vertex( a node) we need first to check R1 and R2 for candidate support foot

        # Step feasibility checks:
        if not r1_feasibility(candidate_sup_f, map):
            # print('r1_check fail')
            time_step.append(0)
            time_step.append(round(time.time() - start_2, 6))
            time_step.append(0)
            continue # The current expansion attempt is aborted and a new iteration is started
        if not r2_feasibility(v_near.f_sup, candidate_sup_f, v_near.f_swg_id): # Redundant check. It has to be guaranteed by the primitive selection.
            print('R2 Fail: Check primitive selection!')
            time_step.append(0)
            time_step.append(round(time.time() - start_2, 6))
            time_step.append(0)
            continue
        
        start_trajectory = time.time()

        candidate_trajectory = r3_feasibility(v_near.f_swg, candidate_sup_f, map)

        time_step.append(round(time.time()-start_trajectory, 6)) #append time for generate trajectory
        #time_step.append(round(end_trajectory-start_trajectory, 6)) #append time for generate trajectory

        if not candidate_trajectory:
            # print('r3:check fail')
            time_step.append(round(time.time() - start_2, 6))
            time_step.append(0)
            continue
        
        v_candidate = Node(candidate_swg_f, candidate_sup_f, f_swg_id=candidate_id, trajectory=candidate_trajectory)
        v_candidate.parent = v_near
        v_near.children.append(v_candidate)


        time_step.append(round(time.time()-start_2, 6)) #append time for step 2
        #time_step.append(round(end-start, 6)) #append time for step 2


        start_3 = time.time()

        """ Step 3) Choosing a parent"""
        # neighbors = neighborhood(v_candidate, rrt_root)

        '''print("current node: ", v_candidate.f_sup, v_candidate.f_swg)
        print("vicini")
        for i in neighbors:
            print(i.f_sup, i.f_swg)
        print("------------------")'''

        candidate_parent = v_near
        # # candidate_cost = cost_of_a_node(v_candidate, candidate_parent, z_range) ### PER ORA IL COSTO PER PASSARE DA UN NODO AD UN ALTRO È 1, VA CAMBIATO, COSÌ È NAIVE
        # # for neigh in neighbors:
        # #     if r2_feasibility( neigh.f_sup, candidate_sup_f, neigh.f_swg_id): ###HERE WE MUST ADD ALSO R3 FEASIBILITY
        # #         if r3_feasibility(neigh.f_swg, candidate_sup_f, map ):
        # #             if (cost_of_a_node(v_candidate, neigh, z_range)) < candidate_cost:
        # #                 candidate_parent = neigh
        # #                 candidate_cost = cost_of_a_node(v_candidate, neigh, z_range)

        # # #change swing foot if the parents is not v_near 
        # # if candidate_parent != v_near:
        # #     v_candidate.f_swg = candidate_parent.f_sup

        # Now let's add the node to the tree
        # # if not v_candidate.parent.parent:
        if candidate_parent != v_near:
            if v_candidate not in candidate_parent.children:
                v_candidate.parent = candidate_parent
                candidate_parent.children.append(v_candidate)
                v_candidate.cost = cost_of_a_node(v_candidate, candidate_parent, z_range) #candidate_cost
                v_near.children.remove(v_candidate)
                v_candidate.f_swg_id = candidate_id
        
        if goal_check(v_candidate, goal_region):
            goal_nodes.append(v_candidate)

            time_step.append(round(time.time()-start_3, 6)) #append time for step 3
            #time_step.append(round(end-start, 6)) #append time for step 3
            all_time.append(time_step)

            break


        time_step.append(round(time.time()-start_3, 6)) #append time for step 3
        #time_step.append(round(end-start, 6)) #append time for step 3

        all_time.append(time_step)
        #time_tot += time_step[1] + time_step[3] + time_step[4]

        

        '''Step 4): Rewiring '''
        # # rewiring(v_candidate, rrt_root, map, z_range)
                      
    print('\n### End RRT search ###')
    tot_for = time.time() - start_rrt
    #print("diff: ", tot_for)
    #print("time tot: ", time_tot)

    try:
        best_goal_node = min(goal_nodes, key=lambda goal_node: goal_node.cost)
    except: # Goal_nodes is an empty list
        print('Path to the goal NOT FOUND!\n')
        return None, retrieve_all_steps(rrt_root), all_time, tot_for
    print('Path to the goal FOUND!\n')
    return retrieve_steps(best_goal_node), retrieve_all_steps(rrt_root), all_time, tot_for


def retrieve_steps(node):
    '''Start from a node of the tree. Returns all the steps to reach the node from the root of the tree ## In the returned list there are also the footprints of the initial stance.'''
    # print('#\nsup:\t', node.f_sup, '\nswg:\t', node.f_swg)
    steps=[]
    while(node.parent):
        steps.insert(0, (node.f_sup, 'Left' if node.f_swg_id == 'Right' else 'Right', node.trajectory))
        node = node.parent
    steps.insert(0, (node.f_swg, node.f_swg_id, node.trajectory))
    steps.insert(0, (node.f_sup, 'Left' if node.f_swg_id == 'Right' else 'Right', node.trajectory))
    return steps


def retrieve_all_steps(node):
    # print('#\nsup:\t', node.f_sup, '\nswg:\t', node.f_swg)
    steps = [(node.f_swg, node.f_swg_id, node.trajectory)]
    queue = [node]
    while len(queue): # finchè la coda non è vuota
        node = queue.pop()
        steps.append( (node.f_sup, 'Left' if node.f_swg_id == 'Right' else 'Right', node.trajectory) )
        for child in node.children:
            queue.append(child)
    return steps


def goal_check(node, goal):
    if abs(goal[2] - node.f_sup[2]) < 0.01:
        distance_to_goal = goal[3] - math.sqrt( (goal[0] - node.f_sup[0])**2 + (goal[1] - node.f_sup[1])**2 )
        if distance_to_goal >= 0: return True
    return False # Result is in MAP COORDS            


def v_near_selection(rrt_root, p_rand, z_range):
    best_distance = node_to_point_distance(rrt_root, p_rand, z_range)
    v_near = rrt_root
    
    # BFS on the RRT tree
    queue = rrt_root.children.copy() 
    while len(queue):
        node = queue.pop(0)
        distance = node_to_point_distance(node, p_rand, z_range)
        if distance < best_distance:
            best_distance = distance
            v_near = node
        for child in node.children:
            queue.append(child)
            
    return best_distance, v_near 


def node_to_point_distance(node, point, z_range):
    # if point[2] > 1:
    #     f = 1
    # else:
    #     f = -1
    mid_point = np.array([(node.f_swg[0] + node.f_sup[0])/2, (node.f_swg[1] + node.f_sup[1])/2, (node.f_swg[2] + node.f_sup[2])/2])
    saggital_axis = np.array((node.f_swg[3] + node.f_sup[3]) / 2)
    joining_vector =np.array([(point[0] - mid_point[0]), (point[1] - mid_point[1]), (point[2] - mid_point[2])])
    phi = np.arctan2(joining_vector[1], joining_vector[0])
    # mean_height = np.array([(node.f_swg[2] + node.f_sup[2]) / 2],  dtype=np.float64) #altezza media del nodo
    # if f == 1: # second floor
    #     distance = norm((mid_point - point)) + k_mu*abs(saggital_axis - phi) + 1000*norm((z_range[1] - mean_height)) # When the node is at the second floor, this malus is zero
    # if f == -1: # ground floor
    #     distance = norm((mid_point - point)) + k_mu*abs(saggital_axis - phi) + 1000*norm((z_range[0] - mean_height)) #When the node is ate groudn floor. this malus is zero
    distance = norm((mid_point - point)) + K_MU*abs(saggital_axis - phi) + K_HEIGHT*(point[2] - mid_point[2]) 
    # distance = norm((mid_point - point)) + k_mu*abs(saggital_axis - phi)  # original cost function
    return distance # Result is in FOOT COORDS


def footstep_to_footstep_distance_metric(f1, f2, k_gamma = K_GAMMA):
    p = np.array([f1[0] - f2[0], f1[1] - f2[1], f1[2] - f2[2]])
    angle = np.array([f1[3] - f2[3]])
    metric = norm(p) + k_gamma*norm(angle)
    return metric # Result is in FOOT COORDS


def neighborhood(vertex, tree_root, r_neigh = 2): # TODO check this part: maybe it considers all the leaf nodes as neighbour
    # Returns a list containing all the nodes of the tree in that have a footstep_to_footstep metric < r_neigh w.r.t VERTEX
    neighbors = []
    if len(tree_root.children) == 0:
        metric = footstep_to_footstep_distance_metric(vertex.f_sup, tree_root.f_sup)
        if metric < r_neigh:
            neighbors.append(tree_root)
        return neighbors
    for child in tree_root.children:
        list_of_neighbors = neighborhood(vertex, child)
        for neigh in list_of_neighbors:
            neighbors.append(neigh)
    return neighbors


def cost_of_a_node(vertex, candidate_parent, z_range): # TODO add an heuristic
    """ Returns an integer >= 0 as a cost to reach the vertex from the root"""
    kj = (vertex.f_swg[2] + vertex.f_sup[2])/2 # Altezza media del nodo
    mid_point = np.array([(vertex.f_swg[0] + vertex.f_sup[0])/2, (vertex.f_swg[1] +vertex.f_sup[1])/2, (vertex.f_swg[2] + vertex.f_sup[2])/2])
    #d = norm((goal_region[0] - mid_point[0]) + 10*(goal_region[1] - mid_point[1]))
    cost = candidate_parent.cost + 100*(z_range[1]-kj) + 1 # Il costo è minore all' aumenatre dell'altezza media del nodo
    return cost



    



def motion_primitive_selector(node):
    """
    Set of primitives U: 20 possible swing foot landing w.r.t the actual support foot
    For now lets assume 5 possible forward distances, 2 side distances and 2 possible angles
    The 2 possible angles are: the saggital_axis angle and (the sggital_axis + 30°)
    The 2 possible side distances are: -12 cm and -24 cm from f_sup along y axis of f_sup, if f_swg is 'Right'
                                      12 cm and 24 cm  if f_swg is 'Left
    The 5 possible forward distances are: -10,0,10,20,30 cm along x axis of f_sup
    U_l = primitives for f_swg_id = 'Left'
    U_r = primitives for f_swg_id = 'Right
    (Recall: foot dimensions are 12x7 units)
    
    """
    # swing = node.f_swg
    support = node.f_sup
    # sagital_axis = (swing[3] + support[3]) / 2 
    # x_swg, y_swg, z_swg, theta_swg = swing
    x_sup, y_sup, z_sup, theta_sup = support
    y_direction = 1 if node.f_swg_id == 'Left' else -1
    new_id = 'Right' if node.f_swg_id == 'Left' else 'Left'

    # p_x = [-0.10, 0, 0.10, 0.20, 0.30, 0.40]
    # p_y = [ -0.24, -0.12, 0, 0.12, 0.24]
    # p_th = [-(pi/5),-(pi/6), 0, +(pi/6), +(pi/5)]
    p_x = PRIMITIVES_X
    p_y = PRIMITIVES_Y
    p_th = PRIMITIVES_THETA
    
    rot = get_z_rotation_matrix(support[3])
    delta = [ random.choice(p_x), y_direction*(random.choice(p_y) + L), 0, y_direction*random.choice(p_th)]
    delta[:-1] = rot.dot(delta[:-1])
    # new_support_foot = [ x_swg + delta[0], y_swg + delta[1], z_swg, theta_swg + delta[3]]
    new_support_foot = [ x_sup + delta[0], y_sup + delta[1], z_sup, theta_sup + delta[3]]
    
    new_swing_foot = node.f_sup
    
    return new_swing_foot, new_support_foot, new_id


def assign_height(support_foot, new_swing_foot_position, map):
    h_prev = support_foot[2]
    cell = map.query(new_swing_foot_position[0], new_swing_foot_position[1]) #This contains the tuples of obejcts heights
    if cell == None: return None
    higher_limit = None
    for i, object in enumerate(cell): 
        surface_height = object[0]
        if (surface_height - h_prev) < DELTA_Z_POS and (surface_height - h_prev) > -DELTA_Z_NEG and (higher_limit == None or object[0] < higher_limit): 
            return surface_height
        higher_limit = min(higher_limit, object[0] - object[1]) if higher_limit!=None else object[0] - object[1]
    return None # questo caso è una sorta di r2 check solo sull' altezza, vuol dire che non cè nessun altezza possbiile da assegnare



def r1_feasibility(f, map):###DA CAMBIAREEEEE: PRIMA CALCOLO DOVE STA IL PIEDE NEL MONDO, POI CAMBIO LE COORDS E VADO NEL
    #SISTEMA DI RIFERIMENTO DELLA MAPPA E SOLO LA VALUTO QUALE QUADRATINI OCCUPA E POI FACCIO IL CHECK
    """
    This verifies that the footstep f is fully in contact within a single horizontal patch.
    To guarantee this, each cell of the map belonging to or overlapping with the footprint
    must have the same height Z. The foorprint is 24x14 cm , but let's use 13x9 to overcome
    small errors in postion
    """

    x = f[0]
    y = f[1]
    z = f[2]
    orientation = f[3]
    size = ROBOT_FOOT_SIZE
    vertices = get_2d_rectangle_coordinates([x,y], size, orientation)
    #print('Vetices: ', vertices)
    
    for ver in vertices:
        cell = map.query(ver[0], ver[1])
        if cell == None or not map.check_collision(x, y, z): 
            return False
        if any( (obj[0] == z) for obj in cell ) and map.check_collision(ver[0], ver[1], z):
            continue
        else:
            return False
    return True

def r2_feasibility( f_prev, f_actual, swg_id):
    """
    Stance feasibility:
    f_prev: è il piede di support del vecchio nodo 
    f_actual: è il piede di supporto del nuovo nodo
    swg_id: swing_id del vecchio nodo. Specifica rispetto a quale piede calcolare la feasibility
    This verifies that the footstep f is kinematically admissibile from the previous footstep f_prev
    The constraints on the 4 dimensions of a footstep( R3 x SO(2)) are chosen by construction, based on 
    dimensions of the robot and the environment. Deltas are the maximum increments acceptable.
    """
    theta_rot = f_prev[3] # ANGOLO DI ROTAZIONE , PARI ALL'ANGOLO DEL PIEDE DI PARTENZA
    rot_matrix = get_z_rotation_matrix(theta_rot)
    rot_matrix = rot_matrix.transpose()
    
    xy_vector = np.array([f_actual[0] - f_prev[0], f_actual[1] - f_prev[1], 0])
    xy = rot_matrix.dot(xy_vector) #- np.array([[0], [L], [0]])
    z = f_actual[2] - f_prev[2]
    theta = f_actual[3] - f_prev[3]

    # xy_vector = np.array([[f_actual[0] - f_prev[0]], [f_actual[1] - f_prev[1]], [0]])
    #xy_pos = rot_matrix.dot(xy_vector) + np.array([[0], [l], [0]])
    #    xy_neg = rot_matrix.dot(xy_vector) + np.array([[0], [-l], [0]])
    # if candiate_swg_id == 'Right':
    #     xy = rot_matrix.dot(xy_vector) + np.array([[0], [l], [0]])
    # else:
    #     xy = rot_matrix.dot(xy_vector) + np.array([[0], [-l], [0]])

    if ((xy[0] < -DELTA_X_NEG) or (xy[0] > DELTA_X_POS)):
        print('X fail, difference is',xy[0], '\n')
        return False
    
    if swg_id == 'Left' and ((xy[1] < L - DELTA_Y_NEG) or (xy[1] > L + DELTA_Y_POS)):
        print("Y_ERROR_Left pos: ", L - DELTA_Y_NEG, '#', xy[1], '#', L + DELTA_Y_POS,'\n')
        return False
    elif swg_id == 'Right' and ((xy[1] > -L + DELTA_Y_NEG) or (xy[1] < -L - DELTA_Y_POS)):
        print("Y_ERROR_Right pos: ", -L + DELTA_Y_NEG, '#', xy[1], '#', -L - DELTA_Y_POS,'\n')
        return False
    
    if ((z < -DELTA_Z_NEG) or (z > DELTA_Z_POS)):
        print("z_ERROR: ", z, '\n')
        return False
    
    if ((theta < -DELTA_THETA_NEG) or (theta > DELTA_THETA_POS )):
        print("THETA_ERROR: ", theta, '\n')
        return False
    return True









#####################################################################################
#PARTE DI MATTEO
def r3_feasibility(f_prev, f_actual, map):# Bisogna passare f_pre_swg e f_actual_sup
    """
    This verifies that the footstep from j-2 to j is collision free.
    To do so, we have to check that:
        - the body doesn't collide with anything (considering a cilinder a little bit larger than the body, to avoid little measurement errors)
        - the foot trajectory doesn't collide with obstacles
    
    f = jth footstep
    f_sup = support foot of the arrival position (j-1th footstep)
    f_prev = j-2th footstep
    v = vertex (mid point)
    """

    x_mean = (f_actual[0] + f_prev[0])/2
    y_mean = (f_actual[1] + f_prev[1])/2
    z_mean = (f_actual[2] + f_prev[2])/2
    z_range = HEIGHT_ROBOT - ZB

    range_x = np.linspace((x_mean-R), (x_mean+R), num=10, endpoint=True)
    range_y = np.linspace((y_mean-R), (y_mean+R), num=10, endpoint=True)
    for x in range_x:
        for y in range_y:
            if (not (map.check_collision(x, y, HEIGHT_ROBOT+z_mean, z_depth=z_range))):
                return False
    

    #FOOT TRAJECTORY CHECK
    return generate_trajectory(f_prev, f_actual, map)





def generate_trajectory(f_prev, f_current, map):
    h_min = 0.02 #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = 0.24 #maximum height for the step (iperparametro da definire) [30 cm = ?]

    #calcolate ipotenusa
    distance = math.sqrt(((f_current[0]-f_prev[0])**2) + ((f_current[1]-f_prev[1])**2))
    if not distance:
        return [0, 0] #
    #generate trajectory with all h feasible until one is acceptable
    interval = np.linspace(0, distance, num=5, endpoint=True) #num = number of value to generate in the interval
    theta = math.atan2((f_current[1]-f_prev[1]), (f_current[0]-f_prev[0])) #angle to calcolate x,y on 3d world
    rotation_matrix = get_z_rotation_matrix_2d(theta)
    for h_max_set in np.linspace(h_min, h_max, 6, endpoint= True):
        #check if it can exists
        if h_max_set + f_prev[2] < f_current[2]:
            continue

        a_par = 2 * ((f_current[2]-f_prev[2]) - (2 * h_max_set)) / (distance)**2
        b_par = ((4 * h_max_set) - (f_current[2]-f_prev[2])) / distance

        trajectory_params = [round(a_par, 6), round(b_par, 6), round(h_max_set, 6)]

        collision = False
        interval = np.linspace(0, distance, num=5, endpoint=True) #num = number of value to generate in the interval
        theta = math.atan2((f_current[1]-f_prev[1]), (f_current[0]-f_prev[0])) #angle to calcolate x,y on 3d world
        rotation_matrix = get_z_rotation_matrix_2d(theta)
        for delta in interval:
            #reconstruction  of the 3d coordinates
            z = trajectory_params[0] * delta**2 + trajectory_params[1]* delta + f_prev[2]
            delta = np.array([delta * math.cos(theta), delta * math.sin(theta)], dtype=np.float64)
            #delta = rotation_matrix.dot(delta) + f_prev[:1]
            delta = delta + f_prev[:1]
            x = delta[0] + f_prev[0]
            y = delta[1] + f_prev[1]

            #check dimension foot
            #orientation of the foot (?)
            foot_size = [0.26, 0.15]
            for vertex in get_2d_rectangle_coordinates([x,y], foot_size, f_prev[3]):
                if (not (map.check_collision(vertex[0], vertex[1], z))):
                    collision = True
                    break
            if collision:
                #print("ERRRORRRRRRR")
                trajectory_params = None
                break
        #if collision: continue
        if not collision: break
    return trajectory_params #doesn't exist any feasible trajectory











def rewiring(v_new, tree, map, z_range): #v_new è il nodo appena aggiunto8 CHE NOI CHIAMIAMO v_candidate
    # print("nodo: ", tree.children)
    # print("v_new: ", v_new)
    if v_new == tree:# Se v_new è la radice dopo non puoi fare la verifica sul suo parent
        return 
    neighbors = neighborhood(v_new, tree) #define neighbors
    for neighbor in neighbors:
        if neighbor.parent is None:
            continue

        if neighbor == v_new.parent:  #if parents skip
            continue

        if (r2_feasibility(v_new.f_sup, neighbor.f_sup, neighbor.f_swg_id) and (r3_feasibility(v_new.f_swg, neighbor.f_sup, map))): #check r2, r3
            if (cost_of_a_node(neighbor, v_new, z_range)) < neighbor.cost: #check if cost is less
                # if neighbor.parent.parent is None:
                #     continue
                neighbor.parent.children.remove(neighbor)
                neighbor.f_swg = v_new.f_sup    #change f_swg of the new child
                v_new.children.append(neighbor)   #set new child of v_new
                neighbor.parent = v_new
                if v_new.f_swg_id == 'Right':
                    neighbor.f_swg_id = 'Left'
                else:
                    neighbor.f_swg_id = 'Right'

                t_new = generate_trajectory(v_new.f_swg, neighbor.f_sup, map) #calcolate new trajectory
                #neighbor.trajectory = t_new
                #set new trajectory on tree

                
            
        
'''new_neighbors = neighborhood(neighbor, tree)
    for j in new_neighbors:
    t_new = generate_trajectory(neighbor.f_swg, j.f_sup) #calcolate new trajectory
     #set new trajectory on tree'''




'''def edge_cost(v_a, v_b):
    c1 = 1  #cost1

    c2 = abs(v_b.f_sup - v_a.f_swg) #cost2

    c3 = 1 / (v_b.f_sup) #miss clearance (?)'''





