#!/usr/bin/env python3
"""
集成启动文件 - Jump Start + Start Engine
将 start_engine GUI 与 jump_start FSM 控制器集成
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取包的共享目录
    jump_start_pkg = FindPackageShare('jump_start')
    start_engine_pkg = FindPackageShare('start_engine')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        jump_start_pkg,
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
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Enable start_engine GUI'
    )
    
    enable_position_controller_arg = DeclareLaunchArgument(
        'enable_position_controller',
        default_value='true',
        description='Enable chassis position controller'
    )
    
    # FSM控制器节点 - 使用配置文件参数
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
    
    # Start Engine GUI节点 - 使用安装在bin目录的可执行文件
    start_engine_gui_node = Node(
        package='start_engine',
        executable='start_gui',
        name='start_engine_gui',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_gui'))
    )
    
    # 位置控制器节点 - 添加缺失的导航控制器
    position_controller_node = Node(
        package='chassis_position_controller',
        executable='position_controller',
        name='position_controller',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_position_controller'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        enable_gui_arg,
        enable_position_controller_arg,
        
        LogInfo(msg="Starting integrated Jump Start + Start Engine system..."),
        
        # 首先启动位置控制器（机器人运动必需）
        position_controller_node,
        
        # 然后启动FSM控制器
        fsm_controller_node,
        
        # 最后启动GUI
        start_engine_gui_node,
        
        LogInfo(msg="🚀 Integration system launched!"),
        LogInfo(msg="   - Position Controller: Converts /target_pose_with_speed to /cmd_vel"),
        LogInfo(msg="   - FSM Controller: Provides /start service"),
        LogInfo(msg="   - Start Engine GUI: Calls /start service"),
        LogInfo(msg="   - Click '启动系统' in GUI to trigger FSM mission")
    ])
