#!/usr/bin/env python3

"""
机器人行为树系统启动文件

这个启动文件会启动完整的机器人行为树系统，包括：
1. 底盘硬件控制
2. 云台像素控制器
3. YOLO检测器
4. 位置控制器
5. 行为树执行器
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    robot_bt_pkg = get_package_share_directory('robot_behavior_tree')
    
    # Declare launch arguments
    behavior_tree_file_arg = DeclareLaunchArgument(
        'behavior_tree_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_behavior_tree'),
            'behavior_trees',
            'robot_main_behavior.xml'
        ]),
        description='Path to the behavior tree XML file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start behavior tree execution'
    )
    
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable behavior tree logging'
    )
    
    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable behavior tree monitor'
    )
    
    enable_hardware_arg = DeclareLaunchArgument(
        'enable_hardware',
        default_value='true',
        description='Enable hardware components (chassis, gimbal, camera)'
    )
    
    enable_simulation_arg = DeclareLaunchArgument(
        'enable_simulation',
        default_value='false',
        description='Enable simulation mode'
    )
    
    # Get launch configurations
    behavior_tree_file = LaunchConfiguration('behavior_tree_file')
    auto_start = LaunchConfiguration('auto_start')
    enable_logging = LaunchConfiguration('enable_logging')
    enable_monitor = LaunchConfiguration('enable_monitor')
    enable_hardware = LaunchConfiguration('enable_hardware')
    enable_simulation = LaunchConfiguration('enable_simulation')
    
    # Behavior Tree Executor Node
    behavior_tree_executor = Node(
        package='robot_behavior_tree',
        executable='behavior_tree_executor',
        name='behavior_tree_executor',
        output='screen',
        parameters=[{
            'behavior_tree_file': behavior_tree_file,
            'auto_start': auto_start,
            'enable_logging': enable_logging,
            'tick_frequency': 10.0,
            'log_file_path': '/tmp/robot_bt_trace.fbl'
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/chassis/odom', '/chassis/odom'),
            ('/cmd_gimbal', '/cmd_gimbal'),
            ('/target_position_pixel', '/target_position_pixel')
        ]
    )
    
    # Behavior Tree Monitor (Python script)
    behavior_tree_monitor = Node(
        package='robot_behavior_tree',
        executable='behavior_tree_monitor.py',
        name='behavior_tree_monitor',
        output='screen',
        condition=IfCondition(enable_monitor)
    )
    
    # Hardware components
    chassis_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_hardware'),
                'launch',
                'chassis_with_led.launch.py'
            ])
        ]),
        condition=IfCondition(enable_hardware)
    )
    
    # Position Controller
    position_controller = Node(
        package='chassis_position_controller',
        executable='simple_robot_controller.py',
        name='position_controller',
        output='screen',
        condition=IfCondition(enable_hardware)
    )
    
    # Gimbal Pixel Controller
    gimbal_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gimbal_pixel_controller'),
                'launch',
                'gimbal_pixel_controller.launch.py'
            ])
        ]),
        condition=IfCondition(enable_hardware)
    )
    
    # YOLO Detector
    yolo_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_detector'),
                'launch',
                'yolo_detector.launch.py'
            ])
        ]),
        condition=IfCondition(enable_hardware)
    )
    
    # Camera driver (if needed)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'orbbec_camera.launch.py'
            ])
        ]),
        condition=IfCondition(enable_hardware)
    )
    
    return LaunchDescription([
        # Launch arguments
        behavior_tree_file_arg,
        auto_start_arg,
        enable_logging_arg,
        enable_monitor_arg,
        enable_hardware_arg,
        enable_simulation_arg,
        
        # Core behavior tree system
        behavior_tree_executor,
        behavior_tree_monitor,
        
        # Hardware components
        chassis_hardware_launch,
        position_controller,
        gimbal_controller_launch,
        yolo_detector_launch,
        camera_launch,
    ])
