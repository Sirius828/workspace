#!/usr/bin/env python3
"""
统一串口管理机器人系统启动文件 (Python版本)
使用unified_serial_manager统一管理串口通信，避免多节点串口冲突
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare as FindPkg


def generate_launch_description():
    # 声明launch参数
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS0',
        description='串口设备路径'
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动RViz'
    )
    
    declare_enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='是否启动相机'
    )
    
    declare_enable_keyboard = DeclareLaunchArgument(
        'enable_keyboard',
        default_value='true',
        description='是否启动键盘遥控'
    )
    
    declare_enable_yolo = DeclareLaunchArgument(
        'enable_yolo',
        default_value='false',
        description='是否启动YOLO检测'
    )
    
    declare_enable_lidar = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='是否启动激光雷达'
    )
    
    # 获取参数
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_keyboard = LaunchConfiguration('enable_keyboard')
    enable_yolo = LaunchConfiguration('enable_yolo')
    enable_lidar = LaunchConfiguration('enable_lidar')
    
    # 统一串口管理节点 - 核心节点
    unified_serial_manager = Node(
        package='wheel_odom_driver',
        executable='unified_serial_manager_node',
        name='unified_serial_manager',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'timeout': 1.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
            'publish_gimbal_feedback': True,
            'cmd_vel_timeout': 2.0,
            'gimbal_cmd_timeout': 5.0,
        }]
    )
    
    # 简化云台控制节点 - 只发布角度命令
    simplified_gimbal_controller = Node(
        package='gimbal_controller',
        executable='simplified_gimbal_controller_node',
        name='simplified_gimbal_controller',
        output='screen',
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'camera_horizontal_fov': 53.35,
            'camera_vertical_fov': 41.30,
            'use_camera_info': True,
            # PID参数
            'kp_yaw': 2.0,
            'ki_yaw': 0.1,
            'kd_yaw': 0.5,
            'kp_pitch': 2.0,
            'ki_pitch': 0.1,
            'kd_pitch': 0.5,
            # 控制参数
            'dead_zone_pixels': 10,
            'max_angle_step': 2.0,
            'yaw_limit_min': -90.0,
            'yaw_limit_max': 90.0,
            'pitch_limit_min': -30.0,
            'pitch_limit_max': 30.0,
            # 跟踪参数
            'tracking_enabled': True,
            'control_frequency': 20.0,
        }]
    )
    
    # 可靠键盘遥控节点
    keyboard_teleop_group = GroupAction(
        condition=IfCondition(enable_keyboard),
        actions=[
            Node(
                package='keyboard_teleop',
                executable='reliable_keyboard_teleop_node',
                name='reliable_keyboard_teleop',
                output='screen',
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
        ]
    )
    
    # 相机驱动组
    camera_group = GroupAction(
        condition=IfCondition(enable_camera),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('camera_driver'),
                        'launch',
                        'camera.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': 'camera',
                    'device_id': '0',
                    'image_width': '640',
                    'image_height': '480',
                    'fps': '30',
                }.items()
            )
        ]
    )
    
    # YOLO目标检测组
    yolo_group = GroupAction(
        condition=IfCondition(enable_yolo),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('yolo_detector'),
                        'launch',
                        'yolo_detector.launch.py'
                    ])
                ]),
                launch_arguments={
                    'input_topic': '/camera/image_raw',
                    'output_topic': '/target_pixel',
                    'confidence_threshold': '0.5',
                }.items()
            )
        ]
    )
    
    # 激光雷达组
    lidar_group = GroupAction(
        condition=IfCondition(enable_lidar),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('oradar_lidar'),
                        'launch',
                        'oradar_scan.launch.py'
                    ])
                ]),
                launch_arguments={
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'laser_frame',
                }.items()
            )
        ]
    )
    
    # RViz可视化组
    rviz_group = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('wheel_odom_driver'),
                    'rviz',
                    'robot_visualization.rviz'
                ])]
            )
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        declare_serial_port,
        declare_baud_rate,
        declare_use_rviz,
        declare_enable_camera,
        declare_enable_keyboard,
        declare_enable_yolo,
        declare_enable_lidar,
        
        # 核心节点
        unified_serial_manager,
        simplified_gimbal_controller,
        
        # 可选功能组
        keyboard_teleop_group,
        camera_group,
        yolo_group,
        lidar_group,
        rviz_group,
    ])
