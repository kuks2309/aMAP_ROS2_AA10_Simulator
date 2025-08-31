#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Teleop twist keyboard node for manual control
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/ackermann_steering_controller/cmd_vel')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'speed': 0.5,
            'turn': 1.0
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        teleop_twist_keyboard
    ])