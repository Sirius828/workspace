from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
import os

def generate_launch_description():
    
    # ========== 可配置参数声明 ==========
    
    # YOLO模型文件参数
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='final_int8.engine',
        description='YOLO模型文件名 (支持格式: .engine, .pt, .onnx)'
    )
    
    # 检测置信度阈值参数
    yolo_confidence_arg = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.77',
        description='YOLO检测置信度阈值 (范围: 0.0-1.0)'
    )
    
    # 目标检测类别参数
    yolo_target_class_arg = DeclareLaunchArgument(
        'yolo_target_class',
        default_value='sticker',
        description='目标检测类别 (模型中的实际类别名称: ferrrari)'
    )
    
    # 推理设备参数
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='0',
        description='YOLO推理设备 (0=第一块GPU, 1=第二块GPU, cpu=CPU推理)'
    )
    
    # 包装脚本路径
    wrapper_script = '/home/sirius/ssd/ros2workspace/src/yolo_detector/scripts/yolo_wrapper.py'
    
    return LaunchDescription([
        # 声明所有参数
        yolo_model_arg,
        yolo_confidence_arg,
        yolo_target_class_arg,
        yolo_device_arg,
        
        # 启动信息
        LogInfo(msg="========== YOLO Detector Starting =========="),
        LogInfo(msg=[
            "配置参数: ",
            "模型=", LaunchConfiguration('yolo_model'), " ",
            "类别=", LaunchConfiguration('yolo_target_class'), " ",
            "置信度=", LaunchConfiguration('yolo_confidence'), " ",
            "设备=", LaunchConfiguration('yolo_device')
        ]),
        
        # 启动YOLO检测器（使用包装脚本）
        ExecuteProcess(
            cmd=[
                '/home/sirius/ssd/ferrari/bin/python3',  # 直接使用虚拟环境的Python
                wrapper_script,
                '--ros-args',
                '-p', ['model_path:=modules/', LaunchConfiguration('yolo_model')],
                '-p', ['confidence_threshold:=', LaunchConfiguration('yolo_confidence')],
                '-p', ['target_class:=', LaunchConfiguration('yolo_target_class')],
                '-p', ['device:=', LaunchConfiguration('yolo_device')],
            ],
            name='yolo_detector',
            output='screen'
        ),
        
        LogInfo(msg="YOLO Detector 启动完成！"),
    ])
