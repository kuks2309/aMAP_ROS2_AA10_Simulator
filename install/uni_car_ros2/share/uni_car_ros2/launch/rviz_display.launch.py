#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car_fix.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Robot description
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint state publisher GUI'
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
    
    # Joint state publisher (with GUI for testing)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_gui,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])