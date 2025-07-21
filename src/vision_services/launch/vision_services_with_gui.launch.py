#!/usr/bin/env python3
"""
Launch file for Vision Services with GUI
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('vision_services')
    config_file = os.path.join(pkg_share, 'config', 'vision_config.yaml')
    
    return LaunchDescription([
        # Launch the vision services node
        Node(
            package='vision_services',
            executable='vision_services_node',
            name='vision_services_node',
            output='screen',
            parameters=[config_file],
            arguments=[]
        ),
        
        # Launch the vision GUI node
        Node(
            package='vision_services',
            executable='vision_gui_node',
            name='vision_gui_node',
            output='screen',
            parameters=[],
            arguments=[]
        ),
    ])
