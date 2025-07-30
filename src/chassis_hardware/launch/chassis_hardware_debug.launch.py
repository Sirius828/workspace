#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'chassis_hardware_params.yaml'),
        description='Path to the ROS2 parameters file to use'
    )
    
    # Chassis hardware node with debug logging
    chassis_hardware_node = Node(
        package='chassis_hardware',
        executable='chassis_hardware_node',
        name='chassis_hardware_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }
        ],
        output='screen',
        emulate_tty=True,
        # Set log level to DEBUG for detailed information
        arguments=['--ros-args', '--log-level', 'chassis_hardware_node:=DEBUG'],
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        config_file_arg,
        chassis_hardware_node,
    ])
