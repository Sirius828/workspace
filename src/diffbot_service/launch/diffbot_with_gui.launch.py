#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        
        # 启动GUI节点
        Node(
            package='diffbot_service',
            executable='diffbot_gui_node.py',
            name='diffbot_gui_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/numbers', '/numbers'),
                ('/start', '/start'),
            ]
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
