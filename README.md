# 3D Motion Planning

3D motion planning for autonomous drone navigation in urban environments.

## Features

- **2.5D Grid-based Planning**: A* search with altitude-aware path planning
- **3D Voxel Mapping**: True 3D voxel grid representation of the environment and route planning
- **Collision Detection**: Safety margin ensures path clears all buildings
- **Path Simplification**: Bresenham ray-tracing algorithm and collinearity tests
- **Interactive Goal Selection**: Pick a goal dynamically from a generated matplotlib grid

## Project Structure

```text
├── collision_demo.py          # Collision detection and path validation demo
├── motion_planning.py         # Main drone path planning script with interactive goal selection
├── planning_utils.py          # A* search, grid creation, ray tracing, path simplification
├── voxel_3d.py                # True 3D voxel mapping and 3D A* search implementation
├── backyard_flyer_solution.py # Reference solution for basic drone flight (square pattern)
├── colliders.csv              # Obstacle data (San Francisco)
└── simulator/                 # Udacity drone simulator (external)
```

## Running

### 3D Voxel Mapping
```bash
python3 voxel_3d.py
```
Creates `voxel_3d_slices.png` showing altitude slices of the 3D map and `voxel_xy_max.png`.

### Collision Detection Demo
```bash
python3 collision_demo.py
```
Shows path avoiding buildings with safety margin. Generates `collision_demo.png`.

### With Simulator
```bash
# Terminal 1 - Start simulator
./simulator/Motion-Planning_Linux_64-bit

# Terminal 2 - Run planning
python3 motion_planning.py
```

## Key Algorithms

### A* Search (2.5D and 3D)
- 2.5D grid search (`planning_utils.py`) with altitude awareness and cost functions considering altitude changes.
- True 3D A* search (`voxel_3d.py`) traversing voxelized space.

### Path Pruning and Simplification
- Pruning collinear points using cross products.
- Bresenham line algorithm to check if a direct path clears obstacles in the 2.5D grid with a safety margin (default 5m).

### Voxel Grid and 2.5D Grid Generation
- 2.5D formulation (`planning_utils.py`): Extracting maximum heights from obstacle data.
- 3D Voxel representation (`voxel_3d.py`): Discretizes north, east, and altitude into a 3D boolean numpy array. 

## Results

- Path climbs to required altitude to clear tall buildings
- Safety margin prevents any collision
- Visualizations show 3D environment structure