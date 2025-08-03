#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 底盘仿真节点
    chassis_sim_node = Node(
        package='chassis_simulation',
        executable='chassis_sim_node',
        name='chassis_sim_node',
        output='screen'
    )
    
    # 包含chassis_display的显示launch
    chassis_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_display'),
                'launch',
                'display.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        chassis_sim_node,
        chassis_display_launch,
    ])
