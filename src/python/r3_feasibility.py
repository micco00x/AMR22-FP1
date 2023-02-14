from calendar import c
from tabnanny import check
import numpy as np
from numpy.random import randint
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy import transpose,  cos ,sin
import random
import math
import sympy
from src.python.utils import get_z_rotation_matrix_2d, get_2d_rectangle_coordinates


#retunr True exists a feasible trajectory, otherwise return False
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

    #BODY DOESN'T COLLIDE
    r = 0.40 #raggio del cilindro (iperparametro da definire) [40 cm = ?]
    #Ab = math.pi * r * r #area di base (pi greco * raggio^2)
    h_robot = 1.5 #altezza robot (iperparametro da definire) [2 m = ?] (deve essere maggiore dell'altezza originale)
    zb = 0.40 #altezza da terra in cui non si calcola il cylindro (dove permetto i movimenti) (iperparametro da definire) [40 cm = ?]

    x_mean = (f_actual[0] + f_prev[0])/2
    y_mean = (f_actual[1] + f_prev[1])/2
    z_mean = (f_actual[2] + f_prev[2])/2
    z_range = h_robot - zb

    #print("foot actual: ", f_actual)
    #print("foot prev: ", f_prev)

    #è piu un # che un cylindro
    range_x = np.linspace((x_mean-r), (x_mean+r), num=10, endpoint=True)
    range_y = np.linspace((y_mean-r), (y_mean+r), num=10, endpoint=True)
    for x in range_x:
        for y in range_y:
            if (not (map.check_collision(x, y, h_robot+z_mean, z_range))):
                print('forse')
                return False
    

    #FOOT TRAJECTORY CHECK
    trajectory = generate_trajectory(f_prev, f_actual, map)
    if trajectory == -1:
        return False

    return True






def generate_trajectory(f_prev, f_current, map):
    h_min = 0.05 #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = 0.30 #maximum height for the step (iperparametro da definire) [30 cm = ?]

    #calcolate ipotenusa
    distance = math.sqrt(((f_current[0]-f_prev[0])**2) + ((f_current[1]-f_prev[1])**2))
    if not distance:
        return [0, 0] #
    #generate trajectory with all h feasible until one is acceptable
    for h_max_set in np.linspace(h_min, h_max, 6, endpoint= True):
        #check if it can exists
        if h_max_set + f_prev[2] < f_current[2]:
            continue

        #GENERATE TRAJECTORY
        #equations to solve

        #OPTION 1
        #finding a, b and c we obtain the equations of the parabola, so we can derive more points (in the middle of the path)
        #define variables
        a, b = sympy.symbols("a b", real=True)

        #i think i have a range (0, i) for x
        #and starting height is 0, and arriving height is f[2]-f_prev[2]

        #set equations equals to 0
        eq1 = sympy.Eq((-4*a)*h_max_set - b * b, 0) # -4a*h + 4ac - b^2 = 0
        #eq2 = sympy.Eq(c, 0) #because seet start of parabola in the origin
        eq3 = sympy.Eq((a * distance * distance) + (b * distance) - (f_current[2]-f_prev[2]), 0) #point on x = ipotenusa, and y = f[2]-f_prev[2]
        # eq1 = sympy.Eq((-4*a)*h_max_set + (4*a*c) - b * b, 0) # -4a*h + 4ac - b^2 = 0
        # eq2 = sympy.Eq(c, 0) #because seet start of parabola in the origin
        # eq3 = sympy.Eq((a * ipotenusa * ipotenusa) + (b * ipotenusa) + c - (f_current[2]-f_prev[2]), 0) #point on x = ipotenusa, and y = f[2]-f_prev[2]

        #solve equations -> return an array of dictionaries with the values of each variables
        res = sympy.solve([eq1, eq3])

        #choose the positive values (parameters)
        a_par = 0
        b_par = 0

        for i in res:
            if i[a] < a_par:
                a_par = i[a]
                b_par = i[b]
        
        trajectory_params = [a_par, b_par]


        #GENERATE 5 RANDOM POINTS
        #generate 5 "random" points of the parabola and return them

        #idea:
        #y dell'equazione della parabola è z
        #x dell'equazione della parabola è la proiezione sin cos di x e y (da capire bene come farla, in base a cosa scelgo sin e cos)

        #da correggere anche sopra!!
        collision = False
        interval = np.linspace(0, distance, num=5, endpoint=True) #num = number of value to generate in the interval
        theta = math.atan2((f_current[1]-f_prev[1]), (f_current[0]-f_prev[0])) #angle to calcolate x,y on 3d world
        rotation_matrix = get_z_rotation_matrix_2d(theta)
        for delta in interval:
            #reconstruction  of the 3d coordinates
            z = trajectory_params[0] * delta**2 + trajectory_params[1]* delta + f_prev[2]
            delta = np.array([delta * math.cos(theta), delta * math.sin(theta)], dtype=np.float64)
            delta = rotation_matrix.dot(delta) + f_prev[:1]
            x = delta[0]
            y = delta[1]

            #check dimension foot
            #orientation of the foot (?)
            foot_size = [0.26, 0.15]
            for vertex in get_2d_rectangle_coordinates([x,y], foot_size, f_prev[3]):
                if (not (map.check_collision(vertex[0], vertex[1], z))):
                    collision = True
                    break
            if collision: break
        if collision: continue
        return trajectory_params #doesn't exist any feasible trajectory
    return -1










def rewiring(v_new, tree): #v_new è il nodo appena aggiunto8 CHE NOI CHIAMIAMO v_candidate
    neighbors = neighborhood(v_new, tree) #define neighbors
    for neighbor in neighbors:
        if neighbor == v_new.parent: #if parents skip
            continue

        if (r2_feasibility(v_new.f_sup, neighbor.f_sup) and (r3_feasibility(v_new.f_swg, neighbor.f_sup ))): #check r2, r3
            if (cost_of_a_new_vertex(neighbor, v_new)) < neighbor.cost: #check if cost is less
                neighbor.parent.children.remove(neighbor)
                neighbor.f_swg = v_new.f_sup    #change f_swg of the new child
                v_new.children.append(neighbor)   #set new child of v_new
                neighbor.parent = v_new
                if v_new.f_swg_id == 'Right':
                    neighbor.f_swg_id = 'Left'
                else:
                    neighbor.f_swg_id = 'Right'

                t_new = generate_trajectory(v_new.f_swg, neighbor.f_sup) #calcolate new trajectory

                return tree
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





