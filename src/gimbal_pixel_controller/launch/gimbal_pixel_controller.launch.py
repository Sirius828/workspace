#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('gimbal_pixel_controller')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'gimbal_params.yaml'),
        description='Path to the gimbal controller parameters file'
    )
    
    # Gimbal pixel controller node
    gimbal_controller_node = Node(
        package='gimbal_pixel_controller',
        executable='pixel_controller_node',
        name='gimbal_pixel_controller',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        gimbal_controller_node,
    ])
