#!/usr/bin/env python3
"""
Gimbal Trajectory Planner Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for gimbal trajectory planner"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('gimbal_trajectory_planner')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'trajectory_planner_config.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )
    
    # Create the trajectory planner node
    trajectory_planner_node = Node(
        package='gimbal_trajectory_planner',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        trajectory_planner_node
    ])
