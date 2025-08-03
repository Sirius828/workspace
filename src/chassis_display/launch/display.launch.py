#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('chassis_display'),
        'urdf',
        'chassis.urdf'
    ])
    
    # Get RViz config file path
    rviz_config = PathJoinSubstitution([
        FindPackageShare('chassis_display'),
        'rviz',
        'chassis_display.rviz'
    ])
    
    # Robot State Publisher - 发布机器人描述和静态变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['cat ', urdf_file])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )
    
    # Joint State Publisher - 发布关节状态（对于固定机器人可选）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # RViz2 - 可视化
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
