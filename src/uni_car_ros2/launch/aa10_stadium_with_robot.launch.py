#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    # Package
    pkg_share = get_package_share_directory('uni_car_ros2')
    world_file = os.path.join(pkg_share, 'worlds', 'aa10_complete_stadium.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car.urdf')
    models_path = os.path.join(pkg_share, 'models')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
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
        'z_pose', default_value='0.1',
        description='Robot spawn z position')
    
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
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-entity', 'uni_car',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', z_pose]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose, 
        declare_z_pose,
        gazebo,
        robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo,
                on_exit=[spawn_robot]
            )
        ),
        joint_state_publisher
    ])