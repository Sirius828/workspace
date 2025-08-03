#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Chassis simulation node
    chassis_simulation_node = Node(
        package='chassis_hardware',
        executable='python3',
        arguments=[PathJoinSubstitution([
            FindPackageShare('chassis_hardware'),
            '..',
            '..',
            '..',
            'src',
            'chassis_hardware',
            'scripts',
            'chassis_simulation.py'
        ])],
        name='chassis_simulation',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )
    
    # Include chassis visualization launch
    chassis_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        chassis_simulation_node,
        chassis_visualization_launch,
    ])
