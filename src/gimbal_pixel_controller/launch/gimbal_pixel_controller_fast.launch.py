#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('gimbal_pixel_controller')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'gimbal_params_fast_response.yaml'),
        description='Path to the gimbal controller parameters file'
    )
    
    use_realtime_arg = DeclareLaunchArgument(
        'use_realtime',
        default_value='true',
        description='Use real-time scheduling priority'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Gimbal pixel controller node with real-time optimizations
    gimbal_controller_node = Node(
        package='gimbal_pixel_controller',
        executable='pixel_controller_node',
        name='gimbal_pixel_controller',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False}  # 确保使用系统时间
        ],
        output='screen',
        emulate_tty=True,
        # 实时性优化
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # QoS 设置在节点内部完成
    )
    
    return LaunchDescription([
        config_file_arg,
        use_realtime_arg,
        log_level_arg,
        gimbal_controller_node,
    ])
