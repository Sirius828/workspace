#!/usr/bin/env python3
"""
简化版云台轨迹规划器启动文件
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('gimbal_trajectory_planner'),
        'config',
        'simple_gimbal_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='gimbal_trajectory_planner',
            executable='trajectory_planner_node',
            name='simple_gimbal_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # 如果需要重新映射话题，在这里添加
                # ('/target_position', '/vision/target_position'),
            ]
        )
    ])
