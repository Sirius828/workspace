from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件的绝对路径
    config_file = os.path.join(
        get_package_share_directory('gimbal_serial_comm'),
        'config',
        'config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='gimbal_serial_comm',
            executable='serial_comm_node',
            name='serial_comm_node',
            output='screen',
            parameters=[config_file],
        )
    ])