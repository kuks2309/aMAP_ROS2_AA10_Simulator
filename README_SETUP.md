# AA10 ROS2 Simulation Setup Guide

## Prerequisites
- Ubuntu 22.04 
- ROS2 Humble
- Gazebo 11

## Installation Steps

### 1. Install ROS2 Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

### 2. Build the Workspace
```bash
cd ~/aa10_sim_ros2_catkin_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Options

#### Option A: New Stable Launch File (Recommended)
```bash
ros2 launch uni_car_ros2 robot_spawn_gazebo.launch.py
```

#### Option B: Original Launch File
```bash
ros2 launch uni_car_ros2 robot_spawn_simple.launch.py
```

## Troubleshooting

### If Gazebo GUI crashes
1. Try software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

2. Or run headless (server only):
```bash
ros2 launch uni_car_ros2 robot_spawn_gazebo.launch.py gui:=false
```

### Check if robot spawned successfully
```bash
ros2 topic list
ros2 topic echo /joint_states
```

## File Structure
- `src/uni_car_ros2/launch/` - Launch files
- `src/uni_car_ros2/urdf/` - Robot URDF files  
- `src/uni_car_ros2/worlds/` - Gazebo world files

## Known Issues
- gzclient may crash on some systems due to OpenGL/camera issues
- Server continues to run normally even if GUI crashes
- Robot spawns successfully regardless of GUI status