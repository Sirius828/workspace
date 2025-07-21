#!/usr/bin/env python3
"""
Launch file for Number Detection Node
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('number_detection')
    config_file = os.path.join(pkg_share, 'config', 'number_detection.yaml')
    
    return LaunchDescription([
        # Launch the number detector node with virtual environment
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                (
                    'source ~/ssd/ferrari/bin/activate && '
                    'export PYTHONPATH="$VIRTUAL_ENV/lib/python3.10/site-packages:$PYTHONPATH" && '
                    'exec ros2 run number_detection number_detector_node '
                    '--ros-args --params-file {}'.format(config_file)
                )
            ],
            name='number_detector',
            output='screen'
        ),
    ])
