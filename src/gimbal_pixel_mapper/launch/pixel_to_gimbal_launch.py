#!/usr/bin/env python3
"""
Launch file for gimbal_pixel_mapper package
启动 pixel_to_gimbal_node 节点并加载配置文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 launch 描述"""
    
    # 获取包路径
    package_share_directory = get_package_share_directory('gimbal_pixel_mapper')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_directory, 'config', 'pixel2gimbal.yaml'),
        description='Path to config file for pixel to gimbal mapping'
    )
    
    # 节点名称参数
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='pixel2gimbal',
        description='Name of the pixel to gimbal node'
    )
    
    # 命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    # 日志级别参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node (debug, info, warn, error)'
    )
    
    def launch_setup(context, *args, **kwargs):
        """启动设置函数"""
        
        # 创建节点
        pixel_to_gimbal_node = Node(
            package='gimbal_pixel_mapper',
            executable='pixel_to_gimbal_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config_file')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0
        )
        
        return [pixel_to_gimbal_node]
    
    return LaunchDescription([
        config_file_arg,
        node_name_arg,
        namespace_arg,
        log_level_arg,
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()
