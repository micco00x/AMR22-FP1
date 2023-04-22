
# Autonomous humanoid navigation in multi-floor environments

Starting from recent papers novelties, we have extended the previously proposed methodologies for navigation in complex uneven terrains by developing a footstep planner based on a randomized algorithm for an autonomous humanoid robot in a multi-floor environment. This was implemented by leveraging a multi-level surface map. 

Tests have been carried out on three different environments, each of them with peculiar characteristics, in order to assess the correct behaviour. 

Finally, the results were validated with simulations on Coppellia Sim using a gait generator based on IS-MCP for computing the CoM trajectory.


# Setup
```bash
pip3 install -r requirements.txt
```

# CoppeliaSim plugin
Translate a scene in Coppelia into a file containing all the useful information about the objects in the scene.

# Multi Level Surface Map
Exploit the information taken from the scene to build a 2.5d map.

# Motion Planning
RRT* to search for a path to the Goal region.

# Show results
To launch the planner with default settings:
```bash
./main.py --world data/world_of_stairs.json --time-max 10000 --resolution 0.02
```
- **world**: path to the json containing all the info about the objects in the scene
- **time-max**: number of iterations of the RRT
- **resolution**: size for the world discretization

# Convert footstep plan
To convert the `.tsv` footstep plan in order to be published as ROS topic:
```bash
./convert-tsv-footstep-plan.py --input TSV_FILE_PATH --output TXT_FILE_PATH
```
