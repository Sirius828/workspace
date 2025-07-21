#!/usr/bin/env python3
"""
完整机器人系统启动文件 (Python版本) - 兼容旧架构
支持条件启动和参数传递
注意：此文件为兼容性保留，推荐使用unified_robot_system.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明启动参数
    declare_enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera system'
    )
    
    declare_enable_yolo = DeclareLaunchArgument(
        'enable_yolo',
        default_value='false',
        description='Enable YOLO detection'
    )
    
    declare_enable_gimbal = DeclareLaunchArgument(
        'enable_gimbal',
        default_value='true',
        description='Enable gimbal controller'
    )
    
    declare_enable_keyboard = DeclareLaunchArgument(
        'enable_keyboard',
        default_value='true',
        description='Enable keyboard teleop'
    )
    
    # 统一串口管理节点 (新架构核心)
    unified_serial_manager = Node(
        package='wheel_odom_driver',
        executable='unified_serial_manager_node',
        name='unified_serial_manager',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyTHS1',
            'baud_rate': 115200,
            'timeout': 1.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
            'publish_gimbal_feedback': True,
            'cmd_vel_timeout': 2.0,
            'gimbal_cmd_timeout': 5.0,
        }]
    )
    
    # 简化云台控制器
    simplified_gimbal_controller = Node(
        package='gimbal_controller',
        executable='simplified_gimbal_controller_node',
        name='simplified_gimbal_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gimbal')),
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'camera_horizontal_fov': 53.35,
            'camera_vertical_fov': 41.30,
            'use_camera_info': True,
            'kp_yaw': 2.0,
            'ki_yaw': 0.1,
            'kd_yaw': 0.5,
            'kp_pitch': 2.0,
            'ki_pitch': 0.1,
            'kd_pitch': 0.5,
            'dead_zone_pixels': 10,
            'max_angle_step': 2.0,
            'yaw_limit_min': -90.0,
            'yaw_limit_max': 90.0,
            'pitch_limit_min': -30.0,
            'pitch_limit_max': 30.0,
            'tracking_enabled': True,
            'control_frequency': 20.0,
        }]
    )
    
    # 键盘遥控
    keyboard_teleop = Node(
        package='keyboard_teleop',
        executable='reliable_keyboard_teleop_node',
        name='reliable_keyboard_teleop',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_keyboard')),
        parameters=[{
            'linear_scale': 1.0,
            'angular_scale': 1.0,
            'step_mode': True,
            'step_linear': 0.1,
            'step_angular': 0.2,
            'accumulate_mode': True,
            'max_linear': 2.0,
            'max_angular': 2.0,
            'publish_rate': 10.0,
            'timeout': 0.5,
        }]
    )
    
    # 构建启动描述
    launch_description = [
        # 声明参数
        declare_enable_camera,
        declare_enable_yolo,
        declare_enable_gimbal,
        declare_enable_keyboard,
        
        # 启动信息
        LogInfo(msg="启动完整机器人系统 (兼容模式)..."),
        LogInfo(msg="推荐使用: ros2 launch wheel_odom_driver unified_robot_system.launch.py"),
        
        # 核心节点
        unified_serial_manager,
        simplified_gimbal_controller,
        keyboard_teleop,
    ]
    
    return LaunchDescription(launch_description)


if __name__ == '__main__':
    generate_launch_description()
