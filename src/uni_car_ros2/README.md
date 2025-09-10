# UNI CAR ROS2 Simulator

ROS2 version of the UNI CAR autonomous vehicle simulator for educational purposes.

## Features
- Ackermann steering vehicle model
- Gazebo simulation with multiple world environments
- Sensor suite: LiDAR, IMU, Camera, Sonar
- ROS2 Control integration
- Navigation stack ready

## Prerequisites
- ROS2 Humble/Iron/Rolling
- Gazebo 11 or Gazebo Fortress
- ros2_control and ros2_controllers packages
- ackermann_msgs

## Installation

```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-ackermann-msgs ros-humble-teleop-twist-keyboard

# Build the package
cd ~/md_sim_ros2_catkin_ws
colcon build --packages-select uni_car_ros2
source install/setup.bash
```

## Usage

### Launch simulation with empty world
```bash
ros2 launch uni_car_ros2 uni_car_world.launch.py
```

### Launch with different worlds
```bash
# House environment
ros2 launch uni_car_ros2 uni_car_world.launch.py world_name:=house.world

# Track environment  
ros2 launch uni_car_ros2 uni_car_world.launch.py world_name:=track.world

# Mando track environment
ros2 launch uni_car_ros2 uni_car_world.launch.py world_name:=mando_track.world
```

### Launch with RViz visualization
```bash
ros2 launch uni_car_ros2 uni_car_world.launch.py rviz:=true
```

### Manual control with keyboard
```bash
# In a new terminal
ros2 launch uni_car_ros2 teleop_keyboard.launch.py
```

## Topics

### Subscribed Topics
- `/ackermann_steering_controller/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Published Topics
- `/scan` (sensor_msgs/LaserScan): LiDAR data
- `/imu` (sensor_msgs/Imu): IMU data
- `/camera/image_raw` (sensor_msgs/Image): Camera image
- `/range_front` (sensor_msgs/Range): Front sonar
- `/odom` (nav_msgs/Odometry): Odometry from controller
- `/joint_states` (sensor_msgs/JointState): Joint states

## Parameters

### Launch Parameters
- `world_name`: Gazebo world file (default: empty.world)
- `x_pose`, `y_pose`, `z_pose`: Initial robot position
- `roll`, `pitch`, `yaw`: Initial robot orientation
- `use_sim_time`: Use simulation time (default: true)
- `rviz`: Launch RViz2 (default: false)

## Robot Specifications
- Base dimensions: 30x18.5x7 cm
- Wheel radius: 3.5 cm
- Wheelbase: 20 cm
- Track width: 19 cm
- Max steering angle: ±30°

## Controllers
The robot uses `ackermann_steering_controller` for realistic car-like motion control with:
- Front wheel steering
- Rear wheel drive
- Velocity and acceleration limits
- Odometry publishing

## Authors
Original ROS1 version designed for autonomous education by:
- Jong-Youb Sah (Yeungnam University)
- Kuk-Won Koh (Hanla University)

Converted to ROS2.