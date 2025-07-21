#!/usr/bin/env python3
"""
综合启动脚本 - 启动完整的云台视觉系统
===========================================

此脚本将同时启动以下组件：
1. Orbbec相机 (gemini_330_series.launch.py)
2. 像素到云台映射节点 (pixel_to_gimbal_launch.py)
3. 视觉服务与GUI (vision_services_with_gui.launch.py)
4. 云台串口通信 (serial_comm_launch.py)

作者: 自动生成
日期: 2025-07-15
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成完整的启动描述"""
    
    # 声明启动参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes (debug, info, warn, error)'
    )
    
    # 获取各个包的路径
    try:
        orbbec_camera_pkg = get_package_share_directory('orbbec_camera')
        gimbal_pixel_mapper_pkg = get_package_share_directory('gimbal_pixel_mapper')
        vision_services_pkg = get_package_share_directory('vision_services')
        gimbal_serial_comm_pkg = get_package_share_directory('gimbal_serial_comm')
    except Exception as e:
        print(f"错误：无法找到包路径 - {e}")
        print("请确保所有包都已正确构建和安装")
        return LaunchDescription([])
    
    # 1. 启动 Orbbec 相机
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(orbbec_camera_pkg, 'launch', 'gemini_330_series.launch.py')
        ]),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )
    
    # 2. 延迟启动云台串口通信 (等待相机初始化)
    gimbal_serial_comm_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(gimbal_serial_comm_pkg, 'launch', 'serial_comm_launch.py')
                ])
            )
        ]
    )
    
    # 3. 延迟启动像素到云台映射节点
    pixel_to_gimbal_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(gimbal_pixel_mapper_pkg, 'launch', 'pixel_to_gimbal_launch.py')
                ]),
                launch_arguments={
                    'log_level': LaunchConfiguration('log_level'),
                }.items()
            )
        ]
    )
    
    # 4. 延迟启动视觉服务与GUI
    vision_services_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(vision_services_pkg, 'launch', 'vision_services_with_gui.launch.py')
                ])
            )
        ]
    )
    
    return LaunchDescription([
        # 启动参数
        log_level_arg,
        
        # 按顺序启动各个组件
        orbbec_camera_launch,        # 立即启动相机
        gimbal_serial_comm_launch,   # 2秒后启动串口通信
        pixel_to_gimbal_launch,      # 3秒后启动像素映射
        vision_services_launch,      # 4秒后启动视觉服务
    ])


if __name__ == '__main__':
    generate_launch_description()
