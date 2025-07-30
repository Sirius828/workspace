#!/usr/bin/env python3
"""
简单导航器启动文件
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'linear_kp',
            default_value='2.0',
            description='线速度比例增益'
        ),
        
        DeclareLaunchArgument(
            'angular_kp', 
            default_value='2.0',
            description='角速度比例增益'
        ),
        
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='1.0',
            description='最大线速度 (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='2.0', 
            description='最大角速度 (rad/s)'
        ),
        
        DeclareLaunchArgument(
            'position_tolerance',
            default_value='0.02',
            description='位置到达容差 (m)'
        ),
        
        DeclareLaunchArgument(
            'angle_tolerance',
            default_value='0.5',
            description='角度到达容差 (rad)'
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
                'position_tolerance': LaunchConfiguration('position_tolerance'),
                'angle_tolerance': LaunchConfiguration('angle_tolerance'),
                'approach_distance': 0.5,
            }]
        ),
    ])
