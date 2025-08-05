from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('chassis_position_controller')
    
    # Config file path (optional, uses defaults if not found)
    config_file = os.path.join(pkg_dir, 'config', 'position_controller.yaml')
    
    # Position Controller Node
    position_controller_node = Node(
        package='chassis_position_controller',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[{
            'position_tolerance': 0.1,
            'angle_tolerance': 0.1,
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'default_linear_speed': 0.3,
            'default_angular_speed': 1.0,
        }]
    )

    return LaunchDescription([
        position_controller_node,
    ])
    
    return LaunchDescription([
        controller_type_arg,
        correct_speed_controller_node,
        enhanced_controller_node,
        basic_controller_node,
    ])
