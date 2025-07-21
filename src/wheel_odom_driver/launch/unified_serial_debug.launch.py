#!/usr/bin/env python3
"""
统一串口管理节点单独启动文件 (Python版本)
专用于调试底盘里程计和云台控制功能
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('wheel_odom_driver')
    
    # 声明launch参数
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='串口设备路径 (/dev/ttyTHS0, /dev/ttyUSB0, /dev/ttyACM0等)'
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'unified_serial_debug_config.yaml']),
        description='配置文件路径'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    declare_enable_tf = DeclareLaunchArgument(
        'enable_tf',
        default_value='true',
        description='是否发布TF变换'
    )
    
    declare_enable_gimbal_feedback = DeclareLaunchArgument(
        'enable_gimbal_feedback',
        default_value='true',
        description='是否发布云台角度反馈'
    )
    
    # 获取参数
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_tf = LaunchConfiguration('enable_tf')
    enable_gimbal_feedback = LaunchConfiguration('enable_gimbal_feedback')
    
    # 统一串口管理节点
    unified_serial_manager = Node(
        package='wheel_odom_driver',
        executable='unified_serial_manager_node',
        name='unified_serial_manager',
        output='screen',
        parameters=[
            config_file,  # 加载YAML配置文件
            {
                # 覆盖参数（命令行参数优先级更高）
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'use_sim_time': use_sim_time,
                'publish_tf': enable_tf,
                'publish_gimbal_feedback': enable_gimbal_feedback,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    return LaunchDescription([
        # 声明所有参数
        declare_serial_port,
        declare_baud_rate,
        declare_config_file,
        declare_log_level,
        declare_use_sim_time,
        declare_enable_tf,
        declare_enable_gimbal_feedback,
        
        # 启动信息
        LogInfo(msg="========================================"),
        LogInfo(msg="🔌 统一串口管理节点调试模式"),
        LogInfo(msg="========================================"),
        LogInfo(msg=["串口设备: ", serial_port]),
        LogInfo(msg=["波特率: ", baud_rate]),
        LogInfo(msg=["配置文件: ", config_file]),
        LogInfo(msg=["日志级别: ", log_level]),
        LogInfo(msg=""),
        LogInfo(msg="📊 监控话题:"),
        LogInfo(msg="  里程计数据: ros2 topic echo /wheel/odom"),
        LogInfo(msg="  云台角度: ros2 topic echo /gimbal/current_angle"),
        LogInfo(msg=""),
        LogInfo(msg="🎮 控制话题:"),
        LogInfo(msg="  底盘速度: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"),
        LogInfo(msg="  云台角度: ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: 10.0, y: 5.0, z: 0.0}'"),
        LogInfo(msg=""),
        LogInfo(msg="🔧 调试命令:"),
        LogInfo(msg="  查看话题: ros2 topic list"),
        LogInfo(msg="  查看节点: ros2 node info /unified_serial_manager"),
        LogInfo(msg="  查看参数: ros2 param list /unified_serial_manager"),
        LogInfo(msg="  话题频率: ros2 topic hz /wheel/odom"),
        LogInfo(msg="========================================"),
        
        # 启动节点
        unified_serial_manager,
    ])


if __name__ == '__main__':
    generate_launch_description()
