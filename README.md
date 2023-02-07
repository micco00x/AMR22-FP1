
# Autonomous humanoid navigation in multi-floor environments

### Setup
```bash
pip3 install -r requirements.txt
```

## Coppelia plugin
Translate a scene in Coppelia into a file containing all the useful information about the objects in the scene.

## Multi Level Surface Map
Exploit the information taken from the scene to build a 2.5d map.

## Motion Planning
RRT* to search for a path to the Goal region.

# Distance function 
Think about exploit:
- (Weighted) Euclidian distance. More importance given to vertical distance.
- Graph based on the connected surfaces. Arcs weighted by the value returnd by the distance function.
- ...

## Visualization

```bash
python src/python/visualization.py --world data/world_of_stairs.json
```
