#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    package_name = 'diffbot_service'
    package_share_directory = get_package_share_directory(package_name)
    
    # 配置文件路径
    config_file_path = os.path.join(
        package_share_directory, 'config', 'diffbot_service_config.yaml'
    )
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file', default=config_file_path)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Path to the config file'
        ),
        
        # DiffBot Service节点
        Node(
            package='diffbot_service',
            executable='diffbot_service_node',
            name='diffbot_service',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # 可以在这里重映射话题名称
            ]
        ),
    ])
