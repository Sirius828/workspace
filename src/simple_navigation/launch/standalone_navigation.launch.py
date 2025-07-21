#!/usr/bin/env python3
"""
独立的简单导航器启动文件
不依赖于其他硬件包，可以单独测试导航功能
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'linear_kp',
            default_value='1.0',
            description='线速度比例增益'
        ),
        
        DeclareLaunchArgument(
            'angular_kp',
            default_value='2.0', 
            description='角速度比例增益'
        ),
        
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.5',
            description='最大线速度 (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='1.0',
            description='最大角速度 (rad/s)'
        ),
        
        # 简单导航器节点
        Node(
            package='simple_navigation',
            executable='simple_navigator',
            name='simple_navigator',
            output='screen',
            parameters=[{
                'linear_kp': LaunchConfiguration('linear_kp'),
                'angular_kp': LaunchConfiguration('angular_kp'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'position_tolerance': 0.1,
                'angle_tolerance': 0.1,
                'approach_distance': 0.5,
            }]
        ),
        
        # 延迟3秒后显示使用说明
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='simple_navigation',
                    executable='send_goal.py',
                    name='usage_info',
                    output='screen',
                    arguments=['--help'],
                    parameters=[{'use_sim_time': False}]
                )
            ]
        ),
    ])
