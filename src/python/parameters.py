
ROBOT_FOOT_SIZE = (0.24, 0.14) # (m, m)
HEIGHT_ROBOT = 1.5
ZB = 0.4
R = 0.25

K_MU = 1
K_GAMMA = 0.4

# Original primitives: # TODO modified value. ask for what to use
# PRIMITIVES_X = [-0.08, 0, 0.08, 0.16, 0.24] # m
# PRIMITIVES_Y = [ 0.20, 0.30 ] # m
# PRIMITIVES_THETA = [ 0, 0.4] # rad
# Primitives for tests:
# PRIMITIVES_X = [-0.07, 0, 0.08, 0.16, 0.23] # m
PRIMITIVES_X = [-0.06, 0, 0.06, 0.10, 0.14, 0.18, 0.23] # m
# PRIMITIVES_Y = [ -0.24, -0.12, 0, 0.12, 0.24] # m
PRIMITIVES_Y = [ -0.07, 0, 0.07 ] # m
PRIMITIVES_THETA = [ -0.2, 0, 0.2 ] # rad
#PRIMITIVES_THETA = [ 0] # rad

H_MIN = 0.02 # m
H_MAX = 0.24 # m
DELTA_H = 0.02 # m

# For stance feasibility:
DELTA_X_NEG = 0.08 # m
DELTA_X_POS = 0.24 # m
DELTA_Y_NEG = 0.25 # m # TODO modified value. ask for what to use
DELTA_Y_POS = 0.25 # m # TODO modified value. ask for what to use
DELTA_Z_NEG = 0.16 # m
DELTA_Z_POS = 0.16 # m
DELTA_THETA_NEG = 0.3 # rad
DELTA_THETA_POS = 0.3 # rad
L = 0.25  # m

MLSM_RESOLUTION = 0.02 # m