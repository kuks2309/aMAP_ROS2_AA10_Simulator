#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package names
    pkg_name = 'uni_car_ros2'
    gazebo_ros_pkg = 'gazebo_ros'
    
    # Paths
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'uni_car.urdf')
    world_path = os.path.join(pkg_share, 'worlds')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='empty.world')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    
    # Robot description from xacro
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty.world',
        description='World file name (empty.world, house.world, track.world, etc.)'
    )
    
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='-0.5')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.0')
    declare_roll = DeclareLaunchArgument('roll', default_value='0.0')
    declare_pitch = DeclareLaunchArgument('pitch', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(gazebo_ros_pkg),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_share, 'worlds', world_name]),
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
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }]
    )
    
    # Joint state publisher (commented out - will be handled by controller)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'uni_car',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )
    
    # Controller manager for ros2_control (commented out for basic launch)
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    #     ],
    #     output='screen'
    # )
    
    # Load joint state broadcaster
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )
    
    # Load ackermann steering controller  
    # ackermann_steering_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )
    
    # RViz2 (optional)
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'uni_car_world.rviz'])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz', default='false'))
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_world_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_roll,
        declare_pitch,
        declare_yaw,
        declare_rviz,
        
        # Nodes
        gazebo,
        robot_state_publisher,
        # joint_state_publisher,
        spawn_entity,
        # controller_manager,
        # joint_state_broadcaster_spawner,
        # ackermann_steering_controller_spawner,
        rviz2
    ])