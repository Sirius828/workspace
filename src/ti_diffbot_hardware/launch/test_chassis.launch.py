#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode', 
        default_value='true',
        description='是否使用真实硬件（false为仿真模式）'
    )
    
    # 获取启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_mode = LaunchConfiguration('hardware_mode')
    
    return LaunchDescription([
        use_sim_time_arg,
        hardware_mode_arg,
        
        # 延时2秒后启动测试节点，确保控制器已经准备好
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ti_diffbot_hardware',
                    executable='test_chassis.py',
                    name='chassis_test',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
    ])
