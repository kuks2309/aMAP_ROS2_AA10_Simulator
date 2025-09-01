#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')
    
    # Robot description
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot spawn x position')
        
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0', 
        description='Robot spawn y position')
        
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.5',
        description='Robot spawn z position')
    
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
            'gui': 'true',
            'debug': 'false'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Spawn robot (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-entity', 'simple_car',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose
                ]
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose, 
        declare_z_pose,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot
    ])