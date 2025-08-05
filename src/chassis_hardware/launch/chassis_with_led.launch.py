#!/usr/bin/env python3

"""
Integrated Chassis Hardware Launch File

This launch file starts all chassis-related components:
- Chassis hardware node (for real hardware communication)
- Victory LED controller node
- All necessary parameters and configurations

Usage:
  ros2 launch chassis_hardware chassis_with_led.launch.py

Arguments:
  serial_port: Serial port for chassis communication (default: /dev/ttyCH341USB0)
  baud_rate: Baud rate for serial communication (default: 115200)
  use_led: Enable LED controller (default: true)
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('chassis_hardware')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyCH341USB0',
        description='Serial port for chassis communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    use_led_arg = DeclareLaunchArgument(
        'use_led',
        default_value='true',
        description='Enable Victory LED controller'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'chassis_hardware_params.yaml'),
        description='Path to the ROS2 parameters file to use'
    )
    
    # Chassis hardware node
    chassis_hardware_node = Node(
        package='chassis_hardware',
        executable='chassis_hardware_node',
        name='chassis_hardware_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom'
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Victory LED controller node
    victory_led_node = Node(
        package='chassis_hardware',
        executable='victory_led_controller.py',
        name='victory_led_controller',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_led'))
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        use_led_arg,
        config_file_arg,
        chassis_hardware_node,
        victory_led_node,
    ])
