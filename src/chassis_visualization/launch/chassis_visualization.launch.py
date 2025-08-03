#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # Chassis visualization node
    chassis_visualization_node = Node(
        package='chassis_visualization',
        executable='chassis_visualization',
        name='chassis_visualization',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )
    
    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('chassis_visualization'),
        'config',
        'rviz_config.rviz'
    ])
    
    # RViz2 node with custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Robot state publisher (for URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ''}  # Will be updated by chassis_visualization node
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        chassis_visualization_node,
        robot_state_publisher,
        rviz_node,
    ])
