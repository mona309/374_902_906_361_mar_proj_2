# Gazebo + Drone + Teleop + Mapping - Complete Setup

This setup provides a complete Gazebo-based drone simulation with teleoperation and SLAM mapping capabilities.

## Setup Instructions

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-bridge ros-humble-slam-toolbox python3-pip
```

### 2. Build the Workspace

```bash
cd ~/drone_mapping_ws
colcon build
source install/setup.bash
```

### 3. Set Up Gazebo Model Path

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/drone_mapping_ws/src/drone_mapping/models
```

## Final Commands to Run

### Terminal 1: Complete Gazebo + SLAM System
```bash
cd ~/drone_mapping_ws
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/drone_mapping_ws/src/drone_mapping/models
ros2 launch drone_mapping gazebo_full_demo.launch.py
```

### Terminal 2: Teleoperation Control
```bash
cd ~/drone_mapping_ws
source install/setup.bash
ros2 run drone_mapping teleop_keyboard
```

### Optional Terminal 3: Autonomous Waypoint Navigation
```bash
cd ~/drone_mapping_ws
source install/setup.bash
ros2 run drone_mapping waypoint_navigator
```

## Controls

### Teleoperation Keys:
- **W**: Forward
- **S**: Backward  
- **A**: Strafe left
- **D**: Strafe right
- **Q**: Up (increase altitude)
- **E**: Down (decrease altitude)
- **K**: Kill switch (emergency stop)
- **R**: Reset velocity
- **Ctrl+C**: Quit

## Features

### 1. Gazebo Simulation
- Complete quadrotor model with physics
- LiDAR sensor for obstacle detection
- IMU for orientation
- Depth camera for visual data
- Multiple obstacles in the world for mapping

### 2. Safety System
- Automatic obstacle detection and avoidance
- Emergency stop functionality
- Velocity filtering for safe operation

### 3. SLAM Mapping
- Real-time 2D mapping using SLAM Toolbox
- Lidar-based scan matching
- Map visualization in RViz

### 4. Teleoperation
- Keyboard-based drone control
- Intuitive movement controls
- Safety integration

## System Architecture

1. **Gazebo Server**: Physics simulation and world rendering
2. **ROS-GZ Bridge**: Communication between Gazebo and ROS2
3. **Drone Controller**: Force-based flight control
4. **Safety Node**: Obstacle detection and command filtering
5. **SLAM Toolbox**: Real-time mapping
6. **RViz**: Visualization of map and robot state

## Troubleshooting

### If Gazebo doesn't find the model:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/drone_mapping/models
```

### If topics don't appear:
```bash
ros2 topic list
ros2 topic echo /drone/lidar/scan
```

### If SLAM doesn't work:
- Check that LiDAR data is being published
- Verify TF tree is correct: `ros2 run tf2_tools view_frames`
- Ensure SLAM Toolbox configuration is loaded

## Expected Output

When running successfully, you should see:
1. Gazebo window with drone and obstacles
2. RViz window showing the map being built
3. Console output from SLAM and safety nodes
4. Real-time map updates as you fly the drone

The drone will start at position (0, 0, 2) and you can begin mapping by using the teleoperation controls.
