#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package names
    pkg_name = 'uni_car_ros2'
    
    # Paths
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
    # Robot description from URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0') 
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.1')
    
    # Start Gazebo server
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             world_file],
        output='screen'
    )
    
    # Start Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
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
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'uni_car',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        
        # Nodes
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        spawn_entity
    ])