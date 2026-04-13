# Drone-based 3D Mapping
## Mini-Project Documentation

---

## 1. PROJECT OBJECTIVE

**Objective:** Develop an autonomous drone system capable of 3D mapping and motion planning in urban environments using A* search algorithm with collision avoidance.

**Outcome:** A fully functional drone path planning system that:
- Creates 3D voxel representation of obstacle-filled environment
- Plans collision-free paths from start to goal location
- Demonstrates in Udacity FCND Simulator

---

## 2. PROBLEM STATEMENT

Autonomous drone navigation in urban environments requires:
- Real-time obstacle detection and mapping
- Efficient path planning to avoid collisions
- Altitude-aware navigation to fly over buildings

This project addresses the challenge of planning a safe flight path through a city with tall buildings using 2.5D grid-based A* search.

---

## 3. TOOLS & PLATFORMS

- **Development:** Python 3
- **Simulation:** Udacity FCND Motion Planning Simulator
- **Libraries:** NumPy, Matplotlib, UdaciDrone API
- **Algorithm:** A* Search with Bresenham line algorithm

---

## 4. IMPLEMENTATION DETAILS

### 4.1 3D Voxel Mapping (`voxel_3d.py`)
- Loads obstacle data from CSV (building coordinates)
- Creates 3D voxel grid (93×93×22 voxels)
- Each voxel marked as obstacle (1) or free (0)
- Visualizes environment at multiple altitude slices

### 4.2 Path Planning (`planning_utils.py`)
- **A* Search:** 2.5D grid search with altitude awareness
- **Grid Creation:** 2.5D grid stores minimum clearance altitude
- **Path Simplification:** Bresenham algorithm for direct path validation
- **Safety Margin:** 5m buffer prevents building collisions

### 4.3 Main Motion Planning (`motion_planning.py`)
- Connects to simulator via MAVLink
- Converts GPS coordinates to local grid
- Plans path and sends waypoints to drone

---

## 5. NOVELTY & INNOVATION

1. **2.5D Hybrid Approach:** More efficient than full 3D voxel search
2. **Dynamic Altitude Planning:** Path climbs to clear tall buildings (e.g., climbs to 29m)
3. **Safety Margin Integration:** Prevents near-miss collisions
4. **Bresenham Validation:** Ensures direct path segments are collision-free

---

## 6. ERROR HANDLING

### 6.1 Emergency Stop
- Drone can be disarmed mid-mission
- Kill switch via keyboard interrupt
- Automatic landing on waypoint queue empty

### 6.2 Error Cases Handled
- Empty waypoint list → triggers landing
- Negative altitude → clamped to 0
- Invalid dtype → fixed to numpy.float64

---

## 7. ROBOT INTERFACE

### 7.1 User Controls
- Run with simulator (TCP port 5760)
- Visual feedback via debug plots
- Console output shows path planning status

### 7.2 Visualization Output
- `voxel_3d_slices.png` - 15 altitude slices
- `voxel_xy_max.png` - Maximum projection view
- `collision_demo.png` - Path vs required clearance

### 7.3 Ease of Use
```bash
# Run voxel mapping
python3 voxel_3d.py

# Run collision demo  
python3 collision_demo.py

# Run with simulator
python3 motion_planning.py
```

---

## 8. RESULTS

- Path successfully navigates around buildings
- Drone climbs to required altitude (29m for tall buildings)
- 0 collisions detected with safety margin enabled
- All waypoints validated via Bresenham algorithm

---

## 9. FILES SUBMITTED

| File | Description |
|------|-------------|
| motion_planning.py | Main path planning script |
| planning_utils.py | A* search, grid creation |
| voxel_3d.py | 3D voxel mapping |
| collision_demo.py | Collision detection demo |
| colliders.csv | San Francisco building data |
| result_video.mp4 | Demo flight video |
| voxel_3d_slices.png | 3D visualization |
| voxel_xy_max.png | XY projection |
| collision_demo.png | Path clearance chart |

---

## 10. REFERENCES

- Udacity FCND Motion Planning Project
- A* Search Algorithm
- Bresenham's Line Algorithm
- UdaciDrone API Documentation