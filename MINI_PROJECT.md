# Drone-based 3D Mapping
## Mini-Project Documentation

---

## 1. PROJECT OVERVIEW

### 1.1 Objective
Develop an autonomous drone system capable of 3D mapping and motion planning in urban environments using A* search algorithm with collision avoidance.

### 1.2 Problem Statement
Autonomous drone navigation in urban environments requires:
- Real-time obstacle detection and mapping
- Efficient path planning to avoid collisions with buildings
- Altitude-aware navigation to fly over buildings

This project addresses the challenge of planning a safe flight path through a city with tall buildings using 2.5D grid-based A* search.

### 1.3 Expected Outcome
A fully functional drone path planning system that:
- Creates 3D voxel representation of obstacle-filled environment
- Plans collision-free paths from start to goal location
- Demonstrates in Udacity FCND Simulator

---

## 2. TOOLS & PLATFORMS USED

| Tool | Purpose |
|------|---------|
| Python 3 | Development language |
| NumPy | Numerical computing for grid data |
| Matplotlib | Visualization of maps and paths |
| UdaciDrone API | Drone simulator communication |
| Udacity FCND Simulator | Flight simulation environment |
| A* Algorithm | Path planning |
| Bresenham Algorithm | Line-of-sight collision check |

---

## 3. DETAILED IMPLEMENTATION

### 3.1 Data Source: colliders.csv
The file contains obstacle data for San Francisco downtown area:
- Format: `posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ`
- Example: `-310.2389, -439.2315, 85.5, 5, 5, 85.5`
- Total: 3,845 buildings/obstacles
- Covers: ~920m × 920m area

### 3.2 Grid Creation (create_grid function)
```
Input: obstacle data, safety_distance (5m)
Output: 2.5D grid, north_offset, east_offset

Process:
1. Calculate min/max north, east coordinates from all obstacles
2. Create 2D grid of size (north_size × east_size)
3. For each cell, store MINIMUM ALTITUDE needed to clear all obstacles
4. Grid[n,e] = required clearance height at that location
```

**Key Insight:** The 2.5D grid stores the MINIMUM altitude needed at each position to clear all obstacles above it.

### 3.3 A* Search Algorithm
```
Input: grid, start_position, goal_position, target_altitude
Output: path (list of waypoints)

Process:
1. Start with initial position at current drone location
2. Use priority queue to explore neighbors (8-connected grid)
3. Cost = movement cost + altitude change penalty
4. Heuristic = Euclidean distance to goal
5. Continue until goal reached or queue empty
```

**Why A*?**
- Optimal: Finds shortest path
- Complete: Will find path if one exists
- Grid-based: Easy to implement and visualize

### 3.4 Path Simplification (Bresenham Algorithm)
The simplified path might still have unnecessary waypoints. We use Bresenham's line algorithm to check if direct path between two waypoints is clear:

```
For each pair of waypoints (start, end):
1. Generate all grid cells between them (Bresenham line)
2. For each cell, check if path altitude >= grid clearance
3. If any cell requires higher altitude → cannot simplify
4. Otherwise → direct path is safe
```

**Example Output:**
```
Before: 19 waypoints
After simplification: 5 waypoints
Path: [(200, 200, 5), (232, 192, 5), (292, 192, 5), ..., (600, 400, 5)]
```

### 3.5 Safety Margin
Added 5m safety margin to prevent near-misses:
- Every path altitude must be (required + 5m)
- This ensures drone stays well clear of buildings

---

## 4. FILE DESCRIPTIONS

### 4.1 voxel_3d.py
Creates TRUE 3D voxel representation:
- 93 × 93 × 22 voxel grid
- Each voxel = 10m × 10m × 10m cube
- Visualizes as 15 altitude slices

### 4.2 collision_demo.py
Demonstrates collision detection:
- Runs path planning
- Compares path altitude vs required clearance
- Shows NO COLLISIONS when safety margin enabled

### 4.3 planning_utils.py
Core algorithms:
- `create_grid()` - 2.5D grid creation
- `a_star()` - A* search implementation
- `path_simplify()` - Bresenham-based simplification
- `bresenham()` - Line generation
- `heuristic()` - Distance calculation

### 4.4 motion_planning.py
Main drone control:
- Connects to simulator (TCP:127.0.0.1:5760)
- Converts GPS → local coordinates
- Plans path → sends waypoints
- Handles flight states (TAKEOFF, WAYPOINT, LANDING)

---

## 5. NOVELTY & INNOVATION

### 5.1 2.5D Hybrid Approach
Instead of full 3D voxel search (93×93×22 = 190,000+ cells), we use 2.5D (93×93 = 8,600 cells). More efficient!

### 5.2 Dynamic Altitude Planning
Path automatically climbs to clear obstacles:
- At (488, 308): climbs to 29m altitude
- This demonstrates intelligent altitude selection

### 5.3 Safety Margin Integration
Prevents "close calls" - drone always stays 5m above building tops.

### 5.4 Bresenham Validation
Ensures every path segment is collision-free, not just waypoints.

---

## 6. ERROR HANDLING

### 6.1 Emergency Stop
- Keyboard interrupt (Ctrl+C) stops mission
- Automatic disarm available
- Kill switch in simulator

### 6.2 Error Cases Handled
| Error | Solution |
|-------|----------|
| Empty waypoint list | Triggers automatic landing |
| Negative altitude | Clamped to 0 |
| Invalid data type | Fixed to numpy.float64 |
| Grid out of bounds | Clipping applied |

---

## 7. ROBOT INTERFACE & INTERACTION

### 7.1 Running the Project
```bash
# Terminal 1 - Start simulator
./simulator/Motion-Planning_Linux_64-bit
# (Select "Motion Planning" in simulator)

# Terminal 2 - Run planning
python3 motion_planning.py
```

### 7.2 Visualization Outputs
- `voxel_3d_slices.png` - Shows obstacles at different altitudes
- `voxel_xy_max.png` - Maximum obstacle height at each position
- `collision_demo.png` - Path altitude vs required clearance

### 7.3 Console Output
Shows real-time status:
```
Searching path...
Path found!
19 waypoints
OK - NO COLLISIONS - path is clear!
```

---

## 8. RESULTS

### 8.1 Test Case
- **Start:** (200, 200, 5)
- **Goal:** (600, 400, 5)
- **Obstacles:** 3,845 buildings

### 8.2 Path Details
- Original A* path: 19 waypoints
- Simplified path: 5 waypoints
- Maximum altitude reached: 29m (to clear tall building)
- Collisions detected: 0

### 8.3 Visualizations Created
1. 15 altitude slices showing 3D obstacles
2. XY maximum projection
3. Path clearance chart

---

## 9. VIVA QUESTIONS & ANSWERS

### Q1: How does the drone avoid buildings?
**A:** The grid stores minimum clearance altitude at each position. A* ensures path altitude is always above this value.

### Q2: Why use 2.5D instead of full 3D?
**A:** 2.5D is more efficient (8,600 cells vs 190,000+) while still planning altitude-aware paths.

### Q3: What is the safety margin?
**A:** 5m buffer added to all clearance calculations. Ensures drone stays well clear of buildings.

### Q4: How does Bresenham algorithm help?
**A:** Checks if direct path between waypoints is clear. If not, adds intermediate waypoints.

### Q5: What happens if path altitude is too low?
**A:** The algorithm automatically increases altitude to clear obstacles. Example: climbs to 29m at position (488, 308).

---

## 10. CONCLUSION

This project demonstrates:
- ✓ 3D environment mapping using voxel grid
- ✓ A* path planning with collision avoidance
- ✓ Altitude-aware navigation
- ✓ Safety margin implementation
- ✓ Error handling and emergency stop

The drone successfully plans and executes collision-free paths in an urban environment with 3,845 buildings.

---

## 11. REFERENCES

1. Udacity FCND Motion Planning Project
2. A* Search Algorithm - Russell & Norvig
3. Bresenham's Line Algorithm (1962)
4. UdaciDrone API Documentation