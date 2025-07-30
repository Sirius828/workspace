#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import conditions
import os

def generate_launch_description():
    # 声明launch参数
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='是否启动GUI界面'
    )
    
    # 获取launch参数
    use_gui = LaunchConfiguration('use_gui')
    
    return LaunchDescription([
        use_gui_arg,
        
        # 启动DiffBot服务节点
        Node(
            package='diffbot_service',
            executable='diffbot_service_node',
            name='diffbot_service_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/goal_pose', '/goal_pose'),
            ]
        ),
        
        # 启动GUI节点（可选）
        Node(
            package='diffbot_service',
            executable='diffbot_gui_node.py',
            name='diffbot_gui_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/numbers', '/numbers'),
                ('/start', '/start'),
            ],
            condition=conditions.IfCondition(use_gui)
        ),
        
        # 启动测试数字发布节点（用于测试GUI）
        Node(
            package='diffbot_service',
            executable='test_number_publisher.py',
            name='test_number_publisher',
            output='screen',
            parameters=[],
            remappings=[
                ('/numbers', '/numbers'),
            ]
        ),
    ])
