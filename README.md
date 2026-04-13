# 3D Motion Planning

3D motion planning for autonomous drone navigation in urban environments.

## Features

- **2.5D Grid-based Planning**: A* search with altitude-aware path planning
- **3D Voxel Mapping**: True 3D voxel grid representation of the environment
- **Collision Detection**: Safety margin ensures path clears all buildings
- **Path Simplification**: Bresenham algorithm for collinearity tests

## Project Structure

```
├── motion_planning.py     # Main drone path planning script
├── planning_utils.py     # A* search, grid creation, path simplification
├── voxel_3d.py          # 3D voxel mapping
├── collision_demo.py     # Collision detection demo
├── colliders.csv        # Obstacle data (San Francisco)
└── simulator/           # Udacity drone simulator
```

## Running

### 3D Voxel Mapping
```bash
python3 voxel_3d.py
```
Creates `voxel_3d_slices.png` showing altitude slices of the 3D map.

### Collision Detection Demo
```bash
python3 collision_demo.py
```
Shows path avoiding buildings with safety margin.

### With Simulator
```bash
# Terminal 1 - Start simulator
./simulator/Motion-Planning_Linux_64-bit

# Terminal 2 - Run planning
python3 motion_planning.py
```

## Key Algorithms

### A* Search
- 2.5D grid search with altitude awareness
- Cost function considers altitude changes

### Path Simplification
- Uses Bresenham line algorithm to check if direct path is clear
- Adds safety margin (5m default) to prevent building collisions

### 3D Voxel Grid
- Discretizes N/E/altitude into voxels
- Each voxel marked as obstacle or free space

## Results

- Path climbs to required altitude to clear tall buildings
- Safety margin prevents any collision
- Visualizations show 3D environment structure