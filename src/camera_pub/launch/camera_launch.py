from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pub',
            executable='camera_pub_node',
            name='camera_pub',
            parameters=[
                {'video_device': '/dev/video0'},
                {'topic': '/camera/color/image_raw'}
            ]
        )
    ])
