#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rb21_localization_dir = get_package_share_directory('rb21_localization')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    lidar_frequency_arg = DeclareLaunchArgument(
        'lidar_frequency',
        default_value='15',
        description='LiDAR motor speed in Hz'
    )
    
    # EKF configuration file path
    ekf_config_file = os.path.join(rb21_localization_dir, 'config', 'ekf.yaml')
    
    # Laser filter configuration file path  
    laser_filter_config_file = os.path.join(rb21_localization_dir, 'config', 'laser_filter.yaml')
    
    # SLAM Toolbox configuration file path
    slam_config_file = os.path.join(rb21_localization_dir, 'config', 'slam_toolbox.yaml')
    
    # LiDAR Node with configurable frequency
    lidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/oradar'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': LaunchConfiguration('lidar_frequency')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # EKF Localization Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odometry/filtered', '/odom')
        ]
    )
    
    # Laser Scan Filter Node
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[
            laser_filter_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('scan', '/scan_filtered')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        lidar_frequency_arg,
        lidar_node,
        ekf_node,
        laser_filter_node,
        slam_toolbox_node,
    ])
