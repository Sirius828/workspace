#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_path",
            default_value="/dev/ttyUSB0",
            description="CAN转USB设备路径",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="串口波特率",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_frequency",
            default_value="100.0",
            description="控制频率 (Hz)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="是否使用仿真时间",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="drpower_controllers.yaml",
            description="控制器配置文件名",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="drpower_arm.urdf.xacro",
            description="机器人描述文件名",
        )
    )

    # 初始化参数
    device_path = LaunchConfiguration("device_path")
    baudrate = LaunchConfiguration("baudrate") 
    control_frequency = LaunchConfiguration("control_frequency")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")

    # 获取URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("drpower_hardware_interface"), "urdf", description_file]),
            " device_path:=", device_path,
            " baudrate:=", baudrate,
            " control_frequency:=", control_frequency,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 获取控制器配置
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("drpower_hardware_interface"),
            "config",
            controllers_file,
        ]
    )

    # 控制器管理器
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # 机器人状态发布器
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 机械臂轨迹控制器
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # 云台控制器（如果配置了）
    gimbal_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gimbal_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition("false"),  # 默认不启动，需要时手动修改
    )

    # 延迟启动控制器，确保硬件接口先启动
    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        gimbal_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
