ub#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    world_file = os.path.join(pkg_share, 'worlds', 'aa10_complete_stadium.world')
    models_path = os.path.join(pkg_share, 'models')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get default Gazebo resource path
    result = subprocess.run(['bash', '-c', 'source /usr/share/gazebo/setup.bash && echo $GAZEBO_RESOURCE_PATH'], 
                          capture_output=True, text=True)
    gazebo_resource_path = result.stdout.strip()
    
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
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': models_path,
            'GAZEBO_RESOURCE_PATH': f'{gazebo_resource_path}:{models_path}'
        }
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo
    ])