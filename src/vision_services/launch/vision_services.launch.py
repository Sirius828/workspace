#!/usr/bin/env python3
"""
Launch file for Vision Services Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vision_services'),
            'config',
            'vision_config.yaml'
        ]),
        description='Path to the vision configuration file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode with image windows'
    )
    
    preview_arg = DeclareLaunchArgument(
        'preview_enabled',
        default_value='true',
        description='Enable preview interface'
    )
    
    # Vision Services Node
    vision_services_node = Node(
        package='vision_services',
        executable='vision_services_node',
        name='vision_services_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'debug': LaunchConfiguration('debug')},
            {'preview_enabled': LaunchConfiguration('preview_enabled')}
        ],
        remappings=[
            # 可以在这里重映射话题名称
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        debug_arg,
        preview_arg,
        vision_services_node
    ])
