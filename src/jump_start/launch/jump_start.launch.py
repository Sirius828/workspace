#!/usr/bin/env python3
"""
Jump Start Launch File
启动FSM控制器和任务执行器
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = FindPackageShare('jump_start')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'fsm_config.yaml'
    ])
    
    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # FSM控制器节点
    fsm_controller_node = Node(
        package='jump_start',
        executable='fsm_controller',
        name='fsm_controller',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True
    )
    
    # 任务执行器节点
    mission_executor_node = Node(
        package='jump_start',
        executable='mission_executor',
        name='mission_executor',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        
        LogInfo(msg="Starting Jump Start FSM Controller and Mission Executor..."),
        
        fsm_controller_node,
        mission_executor_node,
        
        LogInfo(msg="Jump Start system launched successfully!")
    ])
