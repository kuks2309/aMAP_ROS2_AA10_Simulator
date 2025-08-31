#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'track.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Start Gazebo with ROS2 plugins explicitly loaded
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', '/opt/ros/humble/lib/libgazebo_ros_init.so',
            '-s', '/opt/ros/humble/lib/libgazebo_ros_factory.so',
            '-s', '/opt/ros/humble/lib/libgazebo_ros_force_system.so',
            world_file
        ],
        output='screen'
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
    
    # Delayed spawn entity (wait for Gazebo to fully load)
    spawn_entity = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 10 && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity uni_car -x 0.0 -y 0.0 -z 0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])