#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml_config(package_name, config_file):
    """从配置文件中加载YAML参数"""
    try:
        package_share_directory = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_directory, config_file)
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except Exception as e:
        print(f"无法加载配置文件 {config_file}: {e}")
        return None


def generate_launch_description():
    # 获取包路径
    package_name = 'ti_diffbot_hardware'
    package_share_directory = get_package_share_directory(package_name)
    
    # 加载硬件配置文件以获取默认的simulation_mode值
    config = load_yaml_config(package_name, 'hardware_interface.yaml')
    default_simulation_mode = 'false'  # 后备默认值
    if config and 'hardware_interface' in config and 'ros__parameters' in config['hardware_interface']:
        params = config['hardware_interface']['ros__parameters']
        if 'simulation_mode' in params:
            default_simulation_mode = 'true' if params['simulation_mode'] else 'false'
    
    # 机器人描述文件路径
    robot_description_path = os.path.join(
        package_share_directory, 'urdf', 'ti_diffbot.urdf.xacro'
    )
    
    # 控制器配置文件路径
    controllers_config_path = os.path.join(
        package_share_directory, 'diffbot_controllers.yaml'
    )
    
    # 硬件接口配置文件路径
    hardware_config_path = os.path.join(
        package_share_directory, 'hardware_interface.yaml'
    )
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    simulation_mode = LaunchConfiguration('simulation_mode', default=default_simulation_mode)
    
    # 机器人描述
    robot_description_content = Command([
        'xacro ', robot_description_path,
        ' use_sim_time:=', use_sim_time,
        ' simulation_mode:=', simulation_mode
    ])
    
    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_config_path,
        ],
        output='screen',
    )
    
    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '10'],
        output='screen',
    )
    
    # 差速轮控制器
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '10'],
        output='screen',
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )
    
    # RViz2可视化 (可选)
    rviz_config_path = os.path.join(
        package_share_directory, 'rviz', 'diffbot_visualization.rviz'
    )
    
    def launch_rviz(context, *args, **kwargs):
        use_rviz_value = LaunchConfiguration('use_rviz').perform(context)
        if use_rviz_value.lower() == 'true':
            return [Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )]
        return []
    
    rviz_launch = OpaqueFunction(function=launch_rviz)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        
        DeclareLaunchArgument(
            'simulation_mode',
            default_value=default_simulation_mode,
            description='Enable simulation mode (true) or use real hardware (false). Default from config file.'
        ),
        
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_launch,
    ])
