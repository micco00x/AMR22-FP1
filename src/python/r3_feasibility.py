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


#retunr True exists a feasible trajectory, otherwise return False
def r3_feasibility(v, f, f_prev, f_sup, map):
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
    h_robot = 2 #altezza robot (iperparametro da definire) [2 m = ?] (deve essere maggiore dell'altezza originale)
    zb = 0.40 #altezza da terra in cui non si calcola il cylindro (dove permetto i movimenti) (iperparametro da definire) [40 cm = ?]

    x_mean = (f_sup[0] + f[0])/2
    y_mean = (f_sup[1] + f[1])/2
    z_mean = (f_sup[2] + f[2])/2
    z_range = h_robot - zb

    #è piu un poligono che un cylindro
    for x in range((x_mean-r), (x_mean+r)):
        for y in range((y_mean-r), (y_mean+r)):
            if (not (map.check_collision(x, y, (h_robot+z_mean), z_range))):
                return False
    

    #FOOT TRAJECTORY CHECK
    trajectory = generate_trajectory(f_prev, f)
    if trajectory == -1:
        return False

    return True






def generate_trajectory(f_prev, f):
    h_min = 0.05 #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = 0.30 #maximum height for the step (iperparametro da definire) [30 cm = ?]

    #generate trajectory with all h feasible until one is acceptable
    for h_max_set in range(h_min, h_max):
        #check if it can exists
        if h_max_set + f_prev[2] < f[2]:
            continue

        #GENERATE TRAJECTORY
        #equations to solve

        #OPTION 1
        #finding a, b and c we obtain the equations of the parabola, so we can derive more points (in the middle of the path)
        #define variables
        a, b, c = sympy.symbols("a b c", real=True)

        #calcolate ipotenusa
        ipotenusa = math.sqrt(((f[0]-f_prev[0])*(f[0]-f_prev[0])) + ((f[1]-f_prev[1])*(f[1]-f_prev[1])))
        #i think i have a range (0, i) for x
        #and starting height is 0, and arriving height is f[2]-f_prev[2]

        #set equations equals to 0
        eq1 = sympy.Eq((-4*a)*h_max_set + (4*a*c) - b * b, 0) # -4a*h + 4ac - b^2 = 0
        eq2 = sympy.Eq(c, 0) #because seet start of parabola in the origin
        eq3 = sympy.Eq((a * ipotenusa * ipotenusa) + (b * ipotenusa) + c - (f[2]-f_prev[2]), 0) #point on x = ipotenusa, and y = f[2]-f_prev[2]

        #solve equations -> return an array of dictionaries with the values of each variables
        res = sympy.solve([eq1, eq2, eq3])

        
        #choose the positive values (parameters)
        a_par = 0
        b_par = 0
        c_par = 0
        for i in res:
            if (i[b] > b_par) or (b_par == a_par and c_par == a_par and a_par == 0):
                a_par = i[a]
                b_par = i[b]
                c_par = i[c]

        trajectory = [a_par, b_par, c_par]


        #GENERATE 5 RANDOM POINTS
        #generate 5 "random" points of the parabola and return them

        #idea:
        #y dell'equazione della parabola è z
        #x dell'equazione della parabola è la proiezione sin cos di x e y (da capire bene come farla, in base a cosa scelgo sin e cos)

        #da correggere anche sopra!!

        interval = np.linspace(0, ipotenusa, num=5, endpoint=True) #num = number of value to generate in the interval
        theta = math.atan2((f[1]-f_prev[1]), (f[0]-f_prev[0])) #angle to calcolate x,y on 3d world

        check = 0
        for x_generated in interval:
            #reconstruction  of the 3d coordinates
            z = (a * x_generated * x_generated) + (b* x_generated) + c
            x = (x_generated * math.cos(theta)) + f_prev[0]
            y = (x_generated * math.sin(theta)) + f_prev[1]

            #check dimension foot
            #orientation of the foot (?)
            for x_check in range(x-0.03, x+0.03):
                for y_check in range(y-0.06, y+0.06):
                    if (not (map.check_collision(x_check, y_check, z))):
                        return -1
        
        if check == 0:
            return trajectory #feasible trajectory returned

    return -1 #doesn't exist any feasible trajectory










def rewiring(v_new, tree):
    h_min = 0.05 #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = 0.30 #maximum height for the step (iperparametro da definire) [30 cm = ?]

    neighbors = neighborhood(v_new, tree) #define neighbors
    for i in neighbors:
        if i == v_new.parent: #if parents skip
            continue

        if (Feasibility_check(v_new, i)): #check r1, r2, r3
            if (v_new. cost + edge_cost(v_new, i)) < i.cost: #check if cost is less   ##############cost on tree (on each node)#########################
                i.f_swg = v_new.f_sup    #change f_swg of the new child
                i.is_Child(v_new)   #set new child of v_new
                t_new = generate_trajectory(v_new.f_swg, i.f_sup) #calcolate new trajectory
                #set new trajectory on tree

                neighbors2 = neighborhood(i, tree)
                for j in neighbors2:
                    t_new = generate_trajectory(i.f_swg, j.f_sup) #calcolate new trajectory
                    #set new trajectory on tree




def edge_cost(v_a, v_b):
    c1 = 1  #cost1

    c2 = abs(v_b.f_sup - v_a.f_swg) #cost2

    c3 = 1 / (v_b.f_sup) #miss clearance (?)





