import numpy as np
from numpy import random
from numpy.linalg import norm
import random
import math
from tqdm import tqdm

from src.python.utils import get_2d_rectangle_coordinates, get_z_rotation_matrix, get_z_rotation_matrix_2d, wrap_angle, retrieve_steps, retrieve_all_steps, get_number_of_nodes
from src.python.parameters import *
import plotly.graph_objects as go

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


class Node():
    """
    Each node v of the tree is a stance v = (f_swg, f_sup)
    """
    def __init__(self, f_swg, f_sup, f_swg_id = None, cost = 0, parent = None, traj_h = None):
        self.f_swg = f_swg
        self.f_sup = f_sup
        self.parent = parent
        self.children = []
        self.cost = cost
        self.f_swg_id = f_swg_id 
        self.traj_h = traj_h 
        self.primitive_catalogue = self.build_primitive_catalogue()
    
    def __eq__(self, other):
        delta_swg = [ abs(self.f_swg[i]-other.f_swg[i]) < 0.001 if i<3 else abs((self.f_swg[i]-other.f_swg[i]+math.pi)%(2*math.pi)-math.pi) < 0.001 for i in range(len(self.f_swg)) ]
        delta_sup = [ abs(self.f_sup[i]-other.f_sup[i]) < 0.001 if i<3 else abs((self.f_sup[i]-other.f_sup[i]+math.pi)%(2*math.pi)-math.pi) < 0.001 for i in range(len(self.f_swg)) ]
        if all(delta_swg) and all(delta_sup): return True
        
        delta_swg = [ abs(self.f_swg[i]-other.f_sup[i]) < 0.001 if i<3 else abs((self.f_swg[i]-other.f_sup[i]+math.pi)%(2*math.pi)-math.pi) < 0.001 for i in range(len(self.f_swg)) ]
        delta_sup = [ abs(self.f_sup[i]-other.f_swg[i]) < 0.001 if i<3 else abs((self.f_sup[i]-other.f_swg[i]+math.pi)%(2*math.pi)-math.pi) < 0.001 for i in range(len(self.f_swg)) ]
        if all(delta_swg) and all(delta_sup): return True
        
        return False
    
    def build_primitive_catalogue(self):
        catalogue = []
        y_direction = 1 if self.f_swg_id == 'Left' else -1
        rot = get_z_rotation_matrix(self.f_sup[3])
        for dx in PRIMITIVES_X:
            for dy in PRIMITIVES_Y:
                for dtheta in PRIMITIVES_THETA:
                    delta = [ dx, y_direction*(dy + L), 0, y_direction*dtheta]
                    delta[:-1] = rot.dot(delta[:-1])
                    primitive = [ self.f_sup[0] + delta[0], self.f_sup[1] + delta[1], self.f_sup[2], self.f_sup[3] + delta[3]]
                    catalogue.append(primitive)
        return catalogue
    
    def get_new_pose(self):
        if len(self.primitive_catalogue) == 0: return None
        new_swing_foot = self.f_sup
        new_support_foot = self.primitive_catalogue.pop(random.choice(range(len(self.primitive_catalogue))))
        new_id = 'Right' if self.f_swg_id == 'Left' else 'Left'
        return new_swing_foot, new_support_foot, new_id


    def as_showable(self):
        '''Return all the tree that has the node as root in a plottable format.'''
        x, y, z, = [], [], []
        lines = []
        queue = [self]
        while len(queue):
            node = queue.pop(0)
            midpoint = ( (node.f_sup[0]+node.f_swg[0])/2, (node.f_sup[1]+node.f_swg[1])/2, (node.f_sup[2]+node.f_swg[2])/2 )
            x.append( midpoint[0] )
            y.append( midpoint[1] )
            z.append( midpoint[2] )
            if node.parent:
                parent_midpoint = ( (node.parent.f_sup[0]+node.parent.f_swg[0])/2, (node.parent.f_sup[1]+node.parent.f_swg[1])/2, (node.parent.f_sup[2]+node.parent.f_swg[2])/2 )
                lines.append(go.Scatter3d( x=[midpoint[0],parent_midpoint[0]], y=[midpoint[1],parent_midpoint[1]], z=[midpoint[2],parent_midpoint[2]], mode='lines', marker=dict(size = 1, color = z, colorscale='Viridis') ))
            for child in node.children: queue.append(child)
        scatter = go.Scatter3d( x=x, y=y, z=z, mode='markers', marker=dict( size = 1.2, color=z, colorscale='Viridis' ) )
        return (scatter, lines)
            

def RRT(initial_Stance, goal_region, map, time_max):
    """
    Initial_Stance = a list of 2 elements [f_swg_ini, f_sup_ini]
    Goal = a final goal region [an (x,y) area of the map]
    Map = multilevel surface map in 3D (x,y,z)
    """
    rrt_root = Node(initial_Stance[0], initial_Stance[1], f_swg_id='Right')
    x_range, y_range, z_range = map.world_dimensions
    
    if not r2_feasibility(rrt_root.f_sup, rrt_root.f_swg, rrt_root.f_swg_id):
        print('Initial stance NOT feasible!\n')
        return (rrt_root, None)
            
    if goal_check(rrt_root, goal_region):
        print('Already in the goal region! No steps needed')
        return (rrt_root, rrt_root)
    
    goal_nodes = []
    v_candidate = None
    for i in tqdm(range(time_max)):
        """ Step 1) Selecting a vertex for expansion"""
        if random.random() < 0.05: # Select the center of the Goal region as p_rand with a probability of 5%
            p_rand = goal_region[:-1]
        else:
            x_rand = random.random()*abs(x_range[1] - x_range[0]) + x_range[0]
            y_rand = random.random()*abs(y_range[1] - y_range[0]) + y_range[0]
            z_rand = random.random()*abs(z_range[1] - z_range[0]) + z_range[0] # 1.5m added on top and at the bottom to have a better distribution
            p_rand = [x_rand, y_rand, z_rand] # random point in (x,y,z)
        
        distance, v_near = v_near_selection(rrt_root, p_rand) 
        #v_near = rrt_root if i==0 or not v_candidate else v_candidate
        #print('BEST_DISTANCE:',distance)
        
               
        """ Step 2) Generating a candidate vertex"""
        # Now let's generate  a candidate vertex. we need a set U of primitives i.e. a set of landings for the swg foot with respect to the support foot. 
        # candidate_swg_f, candidate_sup_f, candidate_id = motion_primitive_selector(v_near) #candidate_id è il piede di swing del nodo candiate
        selected_new_pose = v_near.get_new_pose()
        if selected_new_pose == None: continue # TODO maybe is better to break
        candidate_swg_f, candidate_sup_f, candidate_id = selected_new_pose
        candidate_sup_f[2] = assign_height(v_near.f_sup, candidate_sup_f, map)
        if candidate_sup_f[2] == None: continue
        # Before creating the vertex( a node) we need first to check R1 and R2 for candidate support foot

        # Step feasibility checks:
        if not r1_feasibility(candidate_sup_f, map):
            # print('r1_check fail')
            continue # The current expansion attempt is aborted and a new iteration is started
        if not r2_feasibility(v_near.f_sup, candidate_sup_f, v_near.f_swg_id): # Redundant check. It has to be guaranteed by the primitive selection.
            print('R2 Fail: Check primitive selection!')
            continue
        candidate_trajectory_height = r3_feasibility(v_near.f_swg, candidate_sup_f, map)
        if not candidate_trajectory_height:
            # print('r3:check fail')
            continue
        
        v_candidate = Node(candidate_swg_f, candidate_sup_f, f_swg_id=candidate_id, traj_h=candidate_trajectory_height)
        v_candidate.parent = v_near
        v_near.children.append(v_candidate)

        """ Step 3) Choosing a parent """
        # neighbors = neighborhood(v_candidate, rrt_root)

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
        if candidate_parent != v_near:
            if v_candidate not in candidate_parent.children:
                v_candidate.parent = candidate_parent
                candidate_parent.children.append(v_candidate)
                v_candidate.cost = cost_of_a_node(v_candidate, candidate_parent, z_range) #candidate_cost
                v_near.children.remove(v_candidate)
                v_candidate.f_swg_id = candidate_id
        
        if goal_check(v_candidate, goal_region):
            goal_nodes.append(v_candidate)
            break


        """Step 4): Rewiring """
        # # rewiring(v_candidate, rrt_root, map, z_range)
                      
    print('\n### End RRT search ###')
    try:
        best_goal_node = min(goal_nodes, key=lambda goal_node: goal_node.cost)
    except: # Goal_nodes is an empty list
        print('Path to the goal NOT FOUND!\n')
        # return retrieve_all_steps(rrt_root)
        return (rrt_root, None)
    print('Path to the goal FOUND!\n')
    # return retrieve_steps(best_goal_node)
    return (rrt_root, best_goal_node)


def goal_check(node, goal):
    if abs(goal[2] - node.f_sup[2]) < 0.01:
        distance_to_goal = goal[3] - math.sqrt( (goal[0] - node.f_sup[0])**2 + (goal[1] - node.f_sup[1])**2 )
        if distance_to_goal >= 0: return True
    return False


def v_near_selection(rrt_root, p_rand):
    best_distance = node_to_point_distance(rrt_root, p_rand)
    v_near = rrt_root
    
    # BFS on the RRT tree
    queue = rrt_root.children.copy() 
    while len(queue):
        node = queue.pop(0)
        for child in node.children:
            queue.append(child)
        if(len(node.primitive_catalogue) == 0): continue
        distance = node_to_point_distance(node, p_rand)
        if distance < best_distance:
            best_distance = distance
            v_near = node
            
    return best_distance, v_near 


def node_to_point_distance(node, point):
    mid_point = np.array([(node.f_swg[0] + node.f_sup[0])/2, (node.f_swg[1] + node.f_sup[1])/2, (node.f_swg[2] + node.f_sup[2])/2])
    saggital_axis = np.array((node.f_swg[3] + node.f_sup[3]) / 2)
    joining_vector =np.array([(point[0] - mid_point[0]), (point[1] - mid_point[1]), (point[2] - mid_point[2])])
    phi = np.arctan2(joining_vector[1], joining_vector[0])
    distance = norm((mid_point - point)) + K_MU*abs(wrap_angle(saggital_axis - phi))  # TODO Add a weight on the z value of the norm
    return distance # Result is in FOOT COORDS


def footstep_to_footstep_distance_metric(f1, f2, k_gamma = K_GAMMA):
    p = np.array([f1[0] - f2[0], f1[1] - f2[1], f1[2] - f2[2]])
    angle = np.array([f1[3] - f2[3]])
    metric = norm(p) + k_gamma*norm(angle)
    return metric # Result is in FOOT COORDS


def neighborhood(vertex, tree_root, r_neigh = 2):
    '''Returns a list containing all the nodes of the tree in that have a footstep_to_footstep metric < r_neigh w.r.t VERTEX'''
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


def assign_height(support_foot, new_swing_foot_position, map):
    h_prev = support_foot[2]
    cell = map.query(new_swing_foot_position[0], new_swing_foot_position[1]) # cell contains the tuples of obejcts heights
    if cell == None: return None
    higher_limit = None
    for i, object in enumerate(cell): 
        surface_height = object[0]
        if (surface_height - h_prev) < DELTA_Z_POS and (surface_height - h_prev) > -DELTA_Z_NEG and (higher_limit == None or object[0] < higher_limit): 
            return surface_height
        higher_limit = min(higher_limit, object[0] - object[1]) if higher_limit!=None else object[0] - object[1]
    return None


def r1_feasibility(f, map):
    """
    Verifies that the footstep f is fully in contact within a single horizontal patch.
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
        if any( abs(obj[0]-z) < 0.001 for obj in cell ) and map.check_collision(ver[0], ver[1], z): 
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
    Verifies that the footstep f is kinematically admissibile from the previous footstep f_prev
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

    if ((xy[0] < -DELTA_X_NEG) or (xy[0] > DELTA_X_POS)):
        # print('X fail, difference is',xy[0], '\n')
        return False
    
    if swg_id == 'Left' and ((xy[1] < L - DELTA_Y_NEG) or (xy[1] > L + DELTA_Y_POS)):
        # print("Y_ERROR_Left pos: ", L - DELTA_Y_NEG, '#', xy[1], '#', L + DELTA_Y_POS,'\n')
        return False
    elif swg_id == 'Right' and ((xy[1] > -L + DELTA_Y_NEG) or (xy[1] < -L - DELTA_Y_POS)):
        # print("Y_ERROR_Right pos: ", -L + DELTA_Y_NEG, '#', xy[1], '#', -L - DELTA_Y_POS,'\n')
        return False
    
    if ((z < -DELTA_Z_NEG) or (z > DELTA_Z_POS)):
        # print("z_ERROR: ", z, '\n')
        return False
    
    if ((theta < -DELTA_THETA_NEG) or (theta > DELTA_THETA_POS )):
        # print("THETA_ERROR: ", theta, '\n')
        return False
    return True


def r3_feasibility(f_prev, f_actual, map):# Bisogna passare f_pre_swg e f_actual_sup
    """
    Verifies that the footstep from j-2 to j is collision free.
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
    # return [0,0,0] # Added to test if generate_trajectory works well (It doesn't for the moment)


def generate_trajectory(f_prev, f_current, map):
    h_min = H_MIN #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = H_MAX #maximum height for the step (iperparametro da definire) [30 cm = ?]

    interval = np.linspace(0, 1, num=5, endpoint=True) #num = number of value to generate in the interval
    distance = math.sqrt(((f_current[0]-f_prev[0])**2) + ((f_current[1]-f_prev[1])**2))

    if not distance:
        return h_min
    
    for h in np.linspace(h_min, h_max, 6, endpoint= True):
        #check if it can exists
        if h + f_prev[2] < f_current[2]:
            continue

        c = f_prev[2]
        b = (4 * h) - f_current[2] + f_prev[2]
        a = (2 * f_current[2]) - (2 * f_prev[2]) - (4 * h)
        foot_size = [0.26, 0.15]
        collision = False
        for s in interval:
            x_s = f_prev[0] + (f_current[0] - f_prev[0]) * s
            y_s = f_prev[1] + (f_current[1] - f_prev[1]) * s
            theta_s = wrap_angle(f_prev[3] + (f_current[3] - f_prev[3]) * s)
            z_s = (a * (s)**2) + (b * s) + c

            for vertex in get_2d_rectangle_coordinates([x_s,y_s], foot_size, theta_s):
                if (not (map.check_collision(vertex[0], vertex[1], z_s))):
                    collision = True
                    break
            if collision:
                break
        if not collision:
            return h
    
    return None




##########################################  Not used stuff section #################################################################

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



def motion_primitive_selector(node):
    """
    [DEPRECATED] Not used anymore
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


##########################################  the end  #################################################################


