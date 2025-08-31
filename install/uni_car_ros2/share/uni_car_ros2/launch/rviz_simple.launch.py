#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot description
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_laser_visual = DeclareLaunchArgument(
        'laser_visual',
        default_value='true',
        description='Show laser visual'
    )
    
    declare_camera_visual = DeclareLaunchArgument(
        'camera_visual',
        default_value='true',
        description='Show camera visual'
    )
    
    declare_imu_visual = DeclareLaunchArgument(
        'imu_visual',
        default_value='true',
        description='Show IMU visual'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # RViz2 with config file
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        rviz2
    ])