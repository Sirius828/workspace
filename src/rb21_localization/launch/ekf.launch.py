import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rb21_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config,
                        {'use_sim_time': False}],
            remappings=[
                ('/wheel/odom', '/wheel/odom'),
                ('/imu/data_raw', '/imu/data_raw')
            ],
        )
        
    ])
    