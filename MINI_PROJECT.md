# DRONE-BASED 3D MAPPING
## COMPREHENSIVE MINI-PROJECT DOCUMENTATION

---

## TABLE OF CONTENTS

1. [Introduction & Problem Statement](#1-introduction--problem-statement)
2. [Theoretical Background](#2-theoretical-background)
3. [Data Structure & Input](#3-data-structure--input)
4. [Algorithm Implementation - Step by Step](#4-algorithm-implementation---step-by-step)
5. [Code Explanation - Every Function](#5-code-explanation---every-function)
6. [Novelty & Innovation](#6-novelty--innovation)
7. [Error Handling](#7-error-handling)
8. [Robot Interface](#8-robot-interface)
9. [Running the Project](#9-running-the-project)
10. [Viva Questions & Answers](#10-viva-questions--answers)
11. [Results & Analysis](#11-results--analysis)
12. [Conclusion](#12-conclusion)

---

## 1. INTRODUCTION & PROBLEM STATEMENT

### 1.1 What is This Project About?

This project is about making an autonomous drone fly from one point to another in a city full of buildings WITHOUT CRASHING.

**Real-world scenario:** Imagine a drone delivery service. The drone needs to:
1. Know where buildings are
2. Plan a path that goes around them
3. Fly at the right altitude to clear building tops

### 1.2 The Challenge

In our project area (San Francisco downtown):
- **3,845 buildings** of various heights
- Buildings up to **200+ meters** tall
- Narrow streets between buildings
- Drone must fly safely above all obstacles

### 1.3 Our Solution

We use **A* (A-star) search algorithm** - the same algorithm used in video games for pathfinding - adapted for 3D drone navigation.

---

## 2. THEORETICAL BACKGROUND

### 2.1 Why Path Planning?

Without path planning, a drone would:
- Fly in a straight line
- Crash into buildings
- Not know alternative routes

With path planning:
- Calculate optimal route beforehand
- Avoid all obstacles
- Reach destination safely

### 2.2 What is A* Search?

A* (pronounced "A-star") is a searching algorithm that finds the SHORTEST path from start to goal.

**How it works (simple explanation):**
```
1. Start at beginning
2. Look at all neighboring points
3. Pick the one closest to goal + least distance traveled
4. Repeat until goal reached
```

**The Magic Formula:**
```
f(n) = g(n) + h(n)

where:
- g(n) = cost from start to current point
- h(n) = estimated cost from current point to goal
- f(n) = total estimated cost (lower is better)
```

### 2.3 Why 2.5D Instead of Full 3D?

**Full 3D:** Search through every (x, y, z) position
- Problem: Too many positions (93 × 93 × 22 = 190,278 cells)
- Slower, more memory

**2.5D Approach:** Search in 2D (x, y), decide altitude dynamically
- Each 2D cell stores: "How high must I fly to clear obstacles here?"
- Problem: Only 93 × 93 = 8,646 cells
- Faster, efficient, still works!

---

## 3. DATA STRUCTURE & INPUT

### 3.1 colliders.csv Format

The input file contains building data:

```
lat0 37.792480, lon0 -122.397450
posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ
-310.2389,-439.2315,85.5,5,5,85.5
-300.2389,-439.2315,85.5,5,5,85.5
...
```

**Header line 1:** Starting GPS coordinates (latitude, longitude)
**Header line 2:** Column names
**Data lines:** Each building

**Meaning of each column:**
| Column | Meaning | Example |
|--------|---------|----------|
| posX | North position (meters from reference) | -310.2389 |
| posY | East position (meters from reference) | -439.2315 |
| posZ | Building center altitude | 85.5 |
| halfSizeX | Half the building width (North-South) | 5 |
| halfSizeY | Half the building width (East-West) | 5 |
| halfSizeZ | Building height | 85.5 |

### 3.2 How We Read the Data

```python
data = np.loadtxt('colliders.csv', delimiter=',', dtype=np.float64, skiprows=2)
```

This loads:
- Comma-separated values
- As floating-point numbers
- Skipping first 2 header lines
- Result: 3,845 rows × 6 columns = NumPy array

---

## 4. ALGORITHM IMPLEMENTATION - STEP BY STEP

### STEP 1: Create the Grid

**Input:** Building data (3,845 buildings)
**Output:** 2D grid where each cell = minimum safe altitude

**Process:**
```python
def create_grid(data, safe_distance):
    # 1. Find boundaries
    north_min = min(all north positions - building size)
    north_max = max(all north positions + building size)
    east_min = min(all east positions - building size)
    east_max = max(all east positions + building size)
    
    # 2. Calculate grid size
    north_size = north_max - north_min + 1  # ~922 cells
    east_size = east_max - east_min + 1     # ~922 cells
    
    # 3. Create empty grid (all zeros = fly at any height)
    grid = zeros((north_size, east_size))
    
    # 4. Mark obstacles
    for each building:
        # Calculate which grid cells it covers
        # Set those cells to building height
        grid[north_cell, east_cell] = building_height
    
    return grid, north_offset, east_offset
```

**Result:** Grid where `grid[100][200] = 50` means "At this location, you need to be at least 50m high to clear the building"

### STEP 2: Run A* Search

**Input:** Start position, goal position, grid
**Output:** List of waypoints

**Process:**
```python
def a_star(grid, start, goal):
    # 1. Priority queue - manages which point to check next
    queue = PriorityQueue()
    queue.put((0, start))  # (cost, position)
    
    # 2. Track where we came from (for path reconstruction)
    came_from = {start: None}
    
    # 3. Keep exploring
    while queue not empty:
        # Get position with lowest cost
        current_cost, current = queue.get()
        
        # Did we reach goal?
        if current == goal:
            return reconstruct_path(came_from)
        
        # Check all 8 neighbors (N, NE, E, SE, S, SW, W, NW)
        for each neighbor:
            if neighbor is valid (on grid, not obstacle):
                new_cost = current_cost + movement_cost
                if new_cost < best_known_cost:
                    queue.put((new_cost, neighbor))
                    came_from[neighbor] = current
```

### STEP 3: Simplify the Path

A* gives us many waypoints. Some are unnecessary!

**Example:**
```
A* Output: [(0,0,5), (1,0,5), (2,0,5), (3,0,5), (4,0,5), (5,0,5)]
Simplified:  [(0,0,5), (5,0,5)]
```

**How Simplification Works (Bresenham Algorithm):**
```python
def path_simplify(path, safety_margin):
    simplified = [path[0]]
    
    for each waypoint pair (start, end):
        # Draw line between them
        line_cells = bresenham(start, end)
        
        # Check if line is safe
        if all cells are clear:
            # Skip intermediate points
            simplified.append(end)
        else:
            # Must go through intermediate points
            add them to path
    
    return simplified
```

### STEP 4: Convert to Waypoints

Grid coordinates → Real-world coordinates:

```python
def path_to_waypoints(path):
    waypoints = []
    for each grid point (n, e, altitude):
        real_north = n + north_offset
        real_east = e + east_offset
        waypoints.append([real_north, real_east, altitude, heading])
    return waypoints
```

---

## 5. CODE EXPLANATION - EVERY FUNCTION

### 5.1 planning_utils.py

#### `class Action:`
Defines possible movements from any grid cell:
```python
class Action(Enum):
    N  = (1, 0)   # North: +1 north,  0 east
    NE = (1, 1)   # Northeast
    E  = (0, 1)   # East
    SE = (-1, 1)  # Southeast
    S  = (-1, 0)  # South
    SW = (-1, -1)  # Southwest
    W  = (0, -1)   # West
    NW = (1, -1)   # Northwest
```
Each action also stores a COST (moving diagonally costs more).

#### `def create_grid(data, safe_distance):`
Creates the 2.5D grid as explained in Step 1.

#### `def heuristic(position, goal):`
Calculates straight-line distance:
```python
def heuristic(position, goal):
    return sqrt((x1-x2)² + (y1-y2)²)
```
This guides A* toward goal.

#### `def valid_actions(grid, current_node):`
Returns which directions are safe to move:
```python
def valid_actions(grid, current):
    for each of 8 directions:
        if (inside grid) AND (not inside building):
            add to valid_actions
    return valid_actions_with_costs
```

#### `def a_star(grid, h, start, goal, flight_altitude):`
The main A* implementation:
1. Initialize priority queue with start position
2. Keep exploring until goal reached or queue empty
3. Use heuristic to guide search
4. Track path using `came_from` dictionary

#### `def path_prune(path, collinear_fn):`
Removes unnecessary waypoints on straight lines.

#### `def path_simplify(grid, path, safety_margin):`
Uses Bresenham to check if direct path is safe:
- If safe: fly directly
- If blocked: add intermediate waypoints

#### `def bresenham(start, end):`
Line drawing algorithm - generates all grid cells between two points.

### 5.2 voxel_3d.py

#### `def create_voxmap(data, voxel_size):`
Creates TRUE 3D voxel grid:
- 93 × 93 × 22 = 190,278 voxels
- Each voxel is obstacle or free

#### `def a_star_3d(voxmap, start, goal):`
3D version of A* (not used in main code but demonstrates full 3D)

### 5.3 motion_planning.py

#### `class MotionPlanning(Drone):`
Main drone controller class inheriting from UdaciDrone.

#### `def local_position_callback():`
Called when drone position updates - checks if at waypoint.

#### `def waypoint_transition():`
Sends next waypoint to drone:
```python
def waypoint_transition(self):
    if waypoints exist:
        next = waypoints.pop(0)
        cmd_position(next.north, next.east, next.altitude)
    else:
        land()
```

#### `def plan_path():`
Main planning function:
1. Load obstacle data
2. Create grid
3. Get drone position (from simulator)
4. Get goal position
5. Run A* search
6. Simplify path
7. Send waypoints to drone

### 5.4 collision_demo.py

Demonstrates that our path never crashes:
1. Plan path from A to B
2. For each waypoint, check altitude vs building height
3. Report if any collision detected

---

## 6. NOVELTY & INNOVATION

### 6.1 2.5D Hybrid Approach
- Instead of full 3D (slow), use 2.5D (fast)
- Grid stores clearance height, not just obstacle/no
- Same accuracy, 20x faster

### 6.2 Dynamic Altitude
- Path altitude CHANGES based on buildings
- Example: At position (488, 308), path climbs to 29m
- This is "intelligent" altitude selection

### 6.3 Safety Margin
- Always add 5m buffer above minimum
- Prevents near-misses
- Accounts for GPS errors

### 6.4 Bresenham Validation
- Not just waypoints are safe
- Every point BETWEEN waypoints checked
- No collisions possible

---

## 7. ERROR HANDLING

### 7.1 Empty Waypoint List
**Problem:** What if A* finds no path?
```python
def waypoint_transition(self):
    if not self.waypoints:
        self.landing_transition()  # Land safely
        return
```

### 7.2 Negative Altitude
**Problem:** Altitude could be negative
```python
self.target_position[2] = max(0, self.target_position[2])
```

### 7.3 Data Type Error
**Problem:** Old numpy syntax ('Float64') doesn't work
```python
# Fixed: dtype=np.float64 (correct)
# Old: dtype='Float64' (wrong)
```

### 7.4 Emergency Stop
- Ctrl+C in terminal stops everything
- Simulator has built-in kill switch
- Drone disarms if connection lost

---

## 8. ROBOT INTERFACE

### 8.1 How to Control

1. **Start Simulator:** Select "Motion Planning" mode
2. **Run Code:** `python3 motion_planning.py`
3. **Drone Automatically:**
   - Takes off
   - Plans path
   - Flies through waypoints
   - Lands at goal

### 8.2 What You See

**In Simulator:**
- Drone flying in 3D city
- Buildings rendered
- Waypoints marked

**In Terminal:**
```
Searching path...
Path found!
OK - NO COLLISIONS - path is clear!
```

### 8.3 Visualizations

| File | What It Shows |
|------|--------------|
| voxel_3d_slices.png | 15 altitude "slices" of city |
| voxel_xy_max.png | Maximum building height at each point |
| collision_demo.png | Path altitude vs required clearance |

---

## 9. RUNNING THE PROJECT

### 9.1 Quick Test (No Simulator)
```bash
# Test collision detection
python3 collision_demo.py

# Output:
# Path found with 19 waypoints
# OK - NO COLLISIONS - path is clear!
```

### 9.2 Full Test (With Simulator)
```bash
# Terminal 1: Start simulator
./simulator/Motion-Planning_Linux_64-bit
# (Click "Motion Planning" in simulator)

# Terminal 2: Run planning
python3 motion_planning.py
```

### 9.3 Generate Visualizations
```bash
# 3D voxel mapping
python3 voxel_3d.py
# Creates: voxel_3d_slices.png, voxel_xy_max.png
```

---

## 10. VIVA QUESTIONS & ANSWERS

### Q1: How does the drone know where buildings are?
**Answer:** We load building data from `colliders.csv`. This file contains GPS coordinates and dimensions for 3,845 buildings in San Francisco. We convert this to a grid where each cell stores the minimum clearance altitude.

### Q2: Why use A* instead of other algorithms?
**Answer:** A* is optimal and complete - it GUARANTEES the shortest path if one exists. It's also well-suited for grid-based navigation and easy to implement.

### Q3: What is the safety margin for?
**Answer:** The 5m safety margin accounts for:
- GPS errors (could be off by a few meters)
- Wind pushing the drone
- Sensor inaccuracies
- Gives pilots time to react

### Q4: How does Bresenham help?
**Answer:** A* gives us waypoints, but we need to make sure the DIRECT PATH between waypoints is clear. Bresenham generates all cells along a line and checks each one.

### Q5: What happens if the drone encounters a new obstacle?
**Answer:** In this project, we assume a known static environment. For dynamic obstacles, you'd need real-time sensing (LiDAR, camera) and re-planning during flight.

### Q6: Why does altitude change during flight?
**Answer:** Different parts of the path have different buildings. The path automatically climbs to clear tall buildings and descends where buildings are short.

### Q7: What is the difference between 2.5D and full 3D?
**Answer:** 
- Full 3D: Every (x, y, z) is a separate state (190,000+ states)
- 2.5D: Only (x, y) matters, z is calculated (8,600 states)
- Result: Same accuracy, 20x faster

### Q8: How do you handle edge cases?
**Answer:**
- No path found → land safely
- Empty waypoint list → trigger landing
- Negative altitude → clamp to 0
- Grid boundary → clipping

---

## 11. RESULTS & ANALYSIS

### 11.1 Test Case
- **Start:** Grid position (200, 200)
- **Goal:** Grid position (600, 400)
- **Buildings:** 3,845

### 11.2 Path Statistics
| Metric | Value |
|--------|-------|
| A* waypoints | 19 |
| Simplified waypoints | 5 |
| Maximum altitude | 29m |
| Path length | ~500m |
| Collision check | PASSED (0 collisions) |

### 11.3 Key Observations
1. Path climbs to 29m at position (488, 308) to clear tall building
2. Path descends when area is clear
3. Safety margin keeps drone 5m above all obstacles

---

## 12. CONCLUSION

This project successfully demonstrates:

1. **3D Mapping:** Creating voxel representation of urban environment
2. **Path Planning:** Using A* for optimal route finding
3. **Collision Avoidance:** Safety margin + Bresenham validation
4. **Dynamic Altitude:** Intelligent height adjustments
5. **Error Handling:** Multiple fail-safes implemented

The drone completes its mission without any collisions, reaching the goal safely by dynamically adjusting altitude to clear buildings along the path.

---

## REFERENCES

1. Udacity Flying Car Nanodegree - Motion Planning Project
2. "Artificial Intelligence: A Modern Approach" - Russell & Norvig
3. Bresenham's Line Algorithm (1962)
4. UdaciDrone API Documentation