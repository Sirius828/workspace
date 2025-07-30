#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # 启动GUI节点
        Node(
            package='diffbot_service',
            executable='diffbot_gui_node.py',
            name='diffbot_gui_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/number', '/number'),
                ('/start', '/start'),
            ]
        ),
    ])
