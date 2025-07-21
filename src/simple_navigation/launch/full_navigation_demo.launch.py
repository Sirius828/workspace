#!/usr/bin/env python3
"""
完整的差速轮机器人导航演示
包含硬件接口 + 简单导航器
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 包路径
    ti_diffbot_share = FindPackageShare('ti_diffbot_hardware')
    simple_nav_share = FindPackageShare('simple_navigation')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='启用仿真模式 (true) 或使用真实硬件 (false)'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='启动RViz可视化'
        ),
        
        # 启动底盘硬件接口
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ti_diffbot_share, 'launch', 'diffbot_hardware.launch.py'])
            ]),
            launch_arguments={
                'simulation_mode': LaunchConfiguration('simulation_mode'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()
        ),
        
        # 启动简单导航器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([simple_nav_share, 'launch', 'simple_navigation.launch.py'])
            ]),
            launch_arguments={
                'linear_kp': '1.2',
                'angular_kp': '2.5',
                'max_linear_vel': '0.4',
                'max_angular_vel': '0.8',
            }.items()
        ),
    ])
