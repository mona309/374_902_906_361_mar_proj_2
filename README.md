# Drone-Based 3D Mapping using ROS2 Humble + Webots
# team 2

This project fulfills the requirements for a drone-based 3D mapping mini-project in robotics. It features a complete ROS2 workspace simulating a Mavic2Pro executing 2D SLAM in Webots to map obstacles.

## 1. FOLDER TREE

```
drone_mapping_ws/
└── src/
    └── drone_mapping/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── launch/
        │   ├── bringup.launch.py
        │   ├── slam.launch.py
        │   └── full_demo.launch.py
        ├── scripts/
        │   ├── drone_controller.py
        │   ├── safety_node.py
        │   ├── teleop_keyboard.py
        │   └── waypoint_navigator.py
        ├── worlds/
        │   └── map_world.wbt
        ├── rviz/
        │   └── mapping.rviz
        └── config/
```

*(Note: Every file contains full, runnable code with no missing dependencies.)*

---

## 2. SETUP INSTRUCTIONS (Ubuntu 22.04 / WSL)

1. Ensure ROS2 Humble and Webots are installed natively or in WSL.
2. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-webots-ros2 ros-humble-slam-toolbox python3-pip
   ```
3. Navigate to the created workspace inside your WSL distribution:
   ```bash
   cd ~/drone_mapping_ws
   ```
4. Build the packages:
   ```bash
   colcon build
   ```

---

## 3. EXACT RUN COMMANDS

To execute the entire system, follow these commands in terminal windows:

### Terminal 1 (Build and Master Launch)
This brings up Webots, the SLAM tool, safety nodes, and RViz visualizations all at once.
```bash
cd ~/drone_mapping_ws
colcon build
source install/setup.bash
ros2 launch drone_mapping full_demo.launch.py
```

### Terminal 2 (Teleoperation)
Drive the drone to build the map!
```bash
cd ~/drone_mapping_ws
source install/setup.bash
ros2 run drone_mapping teleop_keyboard
```
Use `WASD` to steer, `Q/E` for altitude, and `K` to instantly cut thrust (kill switch).

### (Optional) Terminal 3 (Autonomous Waypoint Navigation)
```bash
cd ~/drone_mapping_ws
source install/setup.bash
ros2 run drone_mapping waypoint_navigator
```

---

## 4. DEMO SCRIPT

To demonstrate this effectively in an evaluation setting:
1. **Initialize System**: Show the `ros2 launch drone_mapping full_demo.launch.py` starting up cleanly. Explain that it brings up Webots physics, TF static trees, fake odometry, Hector SLAM (or SLAM Toolbox), and RViz.
2. **Present World**: Focus on the Webots window showing the `Mavic2Pro` surrounded by wooden boxes and walls.
3. **Show RViz Panel**: Side-by-side with webots, point out the raw Lidar scan matching the physical layout exactly.
4. **Begin Mapping (Teleop)**: Start Terminal 2 and take off with `Q` and move forward with `S` (pitch forward). Map lines will start solidifying in RViz.
5. **Demonstrate Safety Node**: Deliberately fly directly at a Wall. Note how `safety_node` intercepts the velocity and forces forward momentum to 0 before crashing, triggering a WARNING in the console logs.
6. **Trigger Kill Switch**: Hit `K` while flying and demonstrate the immediate loss of motor velocity. 
7. **Generate Autonomous Run**: Trigger `waypoint_navigator` and watch the drone run a hardcoded route to fill out the map automatically.

---

## 5. VIVA ANSWERS

**Q1: How do you achieve mapping without wheel odometry on a drone?**
**A1:** We use a setup similar to Hector SLAM (implemented here via `slam_toolbox` configured with a stateless odometry frame `odom` -> `base_link` zero-transform). Because LIDAR scans are dense enough in this static indoor environment, the SLAM node resolves purely via scan-matching.

**Q2: What is the purpose of the safety node and how does it integrate into your processing pipeline?**
**A2:** The `safety_node` subscribes to the Lidar (`/drone/lidar/scan`) and raw teleop commands (`/cmd_vel_raw`). It acts as a middleware or mux. If the Lidar detects an obstacle under the 1.0m threshold, it overrides forward pitch/speed with zeroes to prevent a collision, ignoring user inputs unless they are backing away.

**Q3: How does your Python node communicate with the Webots simulation?**
**A3:** We use the `webots_ros2_driver` python plugin architecture. We define our `drone_controller.py` as a plugin in the Webots URDF. It directly accesses the Webots `Supervisor` API internally to set motor velocities based on classical flight mixer logic (`roll`, `pitch`, `yaw`, thrust addition) calculated from incoming ROS `/cmd_vel` Twists. 

**Q4: How does the drone stabilize its altitude organically?**
**A4:** The python controller calculates the base gravitational offset required to hover (approximately 68.5 rad/s for this Mavic model) and interpolates input vertical velocities relative to this equilibrium payload to rise or sink reliably without a fully tuned internal PID fallback.

**Q5: What was the primary motivation for structuring the Launch files into components?**
**A5:** Following ROS2 best practices, separating `bringup`, `slam`, and `full_demo` allows for robust, granular debugging. We can debug mapping on rosbag data by disabling `bringup` and calling `slam` independently. 
