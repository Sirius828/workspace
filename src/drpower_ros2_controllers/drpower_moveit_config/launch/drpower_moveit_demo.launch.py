#!/usr/bin/env python3
"""
DrPower机械臂MoveIt2演示launch文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 包路径
    drpower_config_pkg = FindPackageShare('drpower_moveit_config')
    drpower_hardware_pkg = FindPackageShare('drpower_hardware_interface')
    moveit_config_pkg = FindPackageShare('moveit_configs_utils')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 automatically with this launch file.'
    )
    
    # DrPower硬件接口
    drpower_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([drpower_hardware_pkg, 'launch', 'drpower_arm.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # MoveIt配置
    robot_description_config = PathJoinSubstitution([
        drpower_hardware_pkg, 'urdf', 'drpower_ros2_control.xacro'
    ])
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config,
        }]
    )
    
    # 关节状态发布器
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # 位置控制器
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drpower_arm_controller'],
        output='screen',
    )
    
    # MoveIt配置
    moveit_config = {
        'robot_description': robot_description_config,
        'robot_description_semantic': PathJoinSubstitution([
            drpower_config_pkg, 'config', 'drpower_arm.srdf'
        ]),
        'robot_description_kinematics': PathJoinSubstitution([
            drpower_config_pkg, 'config', 'kinematics.yaml'
        ]),
        'robot_description_planning': PathJoinSubstitution([
            drpower_config_pkg, 'config', 'joint_limits.yaml'
        ]),
        'use_sim_time': use_sim_time,
    }
    
    # move_group节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # RViz2
    rviz_config_file = PathJoinSubstitution([
        drpower_config_pkg, 'config', 'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[moveit_config],
        condition=IfCondition(use_rviz),
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        drpower_hardware_launch,
        robot_state_publisher,
        joint_state_broadcaster,
        position_controller,
        move_group_node,
        rviz_node,
    ])
