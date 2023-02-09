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
def r3_feasibility(v, f, f_prev, map):
    """
    This verifies that the footstep from j-2 to j is collision free.
    To do so, we have to check that:
        - the body doesn't collide with anything (considering a cilinder a little bit larger than the body, to avoid little measurement errors)
        - the foot trajectory doesn't collide with obstacles
    
    f = jth footstep
    f_prev = j-2th footstep
    v = vertex (mid point)
    """

    #BODY DOESN'T COLLIDE
    r = 0.40 #raggio del cilindro (iperparametro da definire) [40 cm = ?]
    #Ab = math.pi * r * r #area di base (pi greco * raggio^2)
    h_robot = 2 #altezza robot (iperparametro da definire) [2 m = ?] (deve essere maggiore dell'altezza originale)
    zb = 0.40 #altezza da terra in cui non si calcola il cylindro (dove permetto i movimenti) (iperparametro da definire) [40 cm = ?]

    #h = h_robot - zb #height of the cylinder's mid point

    #è piu un poligono che un cylindro
    for z in range(v[2]+zb, v[2] + h_robot): #considering v[2] height of the foot of the robot (mid point on the floor)
        for y in range(v[1]-r, v[1]+r):
            for x in range(v[0]-r, v[0]+r):
                if (map[x,y,z] != 0): #check that there aren't obstacle on the patches of the cylinder
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

        #ELISSOIDE
        #finding a, b and c we obtain the equations of the elissi, so we can derive more points (in the middle of the path)
        #define variables
        a, b, c = sympy.symbols("a b c", real=True)

        #a(x-xC)2+b(y- yC)2+c(z-zC) 2=1

        f_middle = [(f_prev[0]+f[0])/2, (f_prev[1]+f[1])/2, (f_prev[2]+f[2])/2] #middle point

        #set equations equals to 0
        eq1 = sympy.Eq(a * 2 * (f_prev[0] - f_middle[0])**2 + 2 * b * (f_prev[1] - f_middle[1])**2 + 2 * c * (f_prev[2] - f_middle[2])**2, 1)
        eq2 = sympy.Eq(a * 2 * (f[0] - f_middle[0])**2 + 2 * b * (f[1] - f_middle[1])**2 + 2 * c * (f[2] - f_middle[2])**2, 1)
        eq3 = 0 #???

        #solve equations -> return an array of dictionaries with the values of each variables
        res = sympy.solve([eq1, eq2, eq3])


        #OPTION 1
        #finding a, b and c we obtain the equations of the parabola, so we can derive more points (in the middle of the path)
        #define variables
        a, b, c = sympy.symbols("a b c", real=True)

        #set equations equals to 0
        eq1 = sympy.Eq((-4*a*c) + b * b - h_max_set, 0)
        eq2 = sympy.Eq((a * f_prev[0]* f_prev[0]) + (b * f_prev[0]) + c -f_prev[1], 0)
        eq3 = sympy.Eq((a * f[0] * f[0]) + (b * f[0]) + c - f[1], 0)

        #solve equations -> return an array of dictionaries with the values of each variables
        res = sympy.solve([eq1, eq2, eq3])


        #OPTION2
        #i'm not sure about that
        #use the middle point with the maximum height (because the step is always the same)

        #define variables
        a, b, c = sympy.symbols("a b c", real=True)

        f_middle = [(f_prev[0]+f[0])/2, (f_prev[1]+f[1])/2, h_max_set] #middle point
        #set equations equals to 0
        eq1 = sympy.Eq((a * f_middle[0]* f_middle[0]) + (b * f_middle[0]) + c - f_middle[1], 0)
        eq2 = sympy.Eq((a * f_prev[0]* f_prev[0]) + (b * f_prev[0]) + c -f_prev[1], 0)
        eq3 = sympy.Eq((a * f[0] * f[0]) + (b * f[0]) + c - f[1], 0)

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

        check = 0
        for i in (0, 4):
            x, y, z = 0, 0, 0
            if i < 2: #generate number on the first half of the step
                x = random.randint(f_prev[0], f_middle[0])
                y = random.randint(f_prev[1], f_middle[1])
                #project sagittal axis (?)
                z = sympy.symbols("z", real=True)
                eq1 = sympy.Eq((a_par * f_middle[0]* f_middle[0]) + (b_par * f_middle[0]) + c_par - f_middle[1], 0)

            if i >= 2:
                x = random.randint(f_middle[0], f[0])
                y = random.randint(f_middle[1], f[1])
                #project sagittal axis (?)
                z = sympy.symbols("z", real=True)
                eq1 = sympy.Eq((a_par * f_middle[0]* f_middle[0]) + (b_par * f_middle[0]) + c_par - f_middle[1], 0)
            
            if map[x,y,z] != 0: #if is not free
                check = -1
                break
        
        if check == 0:
            return trajectory #feasible trajectory returned

    return -1 #doesn't exist any feasible trajectory










def rewiring(v_new):
    h_min = 0.05 #minimum height for the step (iperparametro da definire) [5 cm = ?]
    h_max = 0.30 #maximum height for the step (iperparametro da definire) [30 cm = ?]

    neighbors = neighborhood(v_new) #define neighbors
    for i in neighbors:
        if i == v_new.parent: #if parents skip
            continue

        if (Feasibility_check(v_new, i)): #check r1, r2, r3
            if (v_new. cost + edge_cost(v_new, i)) < i.cost: #check if cost is less   ##############cost on tree (on each node)#########################
                i.f_swg = v_new.f_sup    #change f_swg of the new child
                i.is_Child(v_new)   #set new child of v_new
                t_new = generate_trajectory(v_new.f_swg, i.f_sup) #calcolate new trajectory
                #set new trajectory on tree

                neighbors2 = neighborhood(i)
                for j in neighbors2:
                    t_new = generate_trajectory(i.f_swg, j.f_sup) #calcolate new trajectory
                    #set new trajectory on tree




def edge_cost(v_a, v_b):
    c1 = 1  #cost1

    c2 = abs(v_b.f_sup - v_a.f_swg) #cost2

    c3 = 1 / (v_b.f_sup) #miss clearance (?)





