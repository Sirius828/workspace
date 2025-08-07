#!/home/sirius/ssd/ferrari/bin/python3
"""
YOLO检测器包装脚本 - 直接使用虚拟环境的Python解释器
"""
import os
import sys
import subprocess

def setup_environment():
    """设置环境变量，确保虚拟环境和ROS2共存"""
    
    # 虚拟环境路径
    venv_path = "/home/sirius/ssd/ferrari"
    venv_site_packages = f"{venv_path}/lib/python3.10/site-packages"
    
    # 获取当前PYTHONPATH
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    
    # 将虚拟环境的site-packages添加到PYTHONPATH前面
    if venv_site_packages not in current_pythonpath:
        if current_pythonpath:
            new_pythonpath = f"{venv_site_packages}:{current_pythonpath}"
        else:
            new_pythonpath = venv_site_packages
        os.environ['PYTHONPATH'] = new_pythonpath
    
    # 添加ROS2包路径，确保能导入ROS2模块
    ros2_paths = [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/home/sirius/ssd/ros2workspace/install/yolo_detector/lib/python3.10/site-packages",
        "/home/sirius/ssd/ros2workspace/install/gimbal_controller/lib/python3.10/site-packages",
        "/home/sirius/ssd/ros2workspace/install/camera_driver/lib/python3.10/site-packages"
    ]
    
    # 将ROS2路径添加到sys.path
    for path in ros2_paths:
        if path not in sys.path and os.path.exists(path):
            sys.path.append(path)
    
    # 确保ROS2环境变量存在
    if 'ROS_DISTRO' not in os.environ:
        os.environ['ROS_DISTRO'] = 'humble'
        print("设置ROS_DISTRO=humble")
    
    # 设置其他必要的ROS2环境变量
    if 'ROS_VERSION' not in os.environ:
        os.environ['ROS_VERSION'] = '2'
    
    if 'RMW_IMPLEMENTATION' not in os.environ:
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    
    print(f"Python解释器: {sys.executable}")
    print(f"PYTHONPATH: {os.environ.get('PYTHONPATH', 'NOT SET')}")
    print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'NOT SET')}")
    print(f"虚拟环境路径: {venv_site_packages}")

def main():
    """主函数"""
    # 设置环境
    setup_environment()
    
    # 导入ROS2模块
    try:
        import rclpy
        print("✓ ROS2 rclpy 导入成功")
    except ImportError as e:
        print(f"✗ ROS2 rclpy 导入失败: {e}")
        print("尝试手动添加ROS2路径...")
        
        # 手动添加ROS2路径
        ros_paths = [
            "/opt/ros/humble/lib/python3.10/site-packages",
            "/opt/ros/humble/local/lib/python3.10/dist-packages"
        ]
        for path in ros_paths:
            if os.path.exists(path) and path not in sys.path:
                sys.path.insert(0, path)
        
        try:
            import rclpy
            print("✓ ROS2 rclpy 手动导入成功")
        except ImportError as e2:
            print(f"✗ ROS2 rclpy 手动导入仍然失败: {e2}")
            return 1
    
    # 导入ultralytics
    try:
        import ultralytics
        print(f"✓ ultralytics 导入成功，版本: {ultralytics.__version__}")
    except ImportError as e:
        print(f"✗ ultralytics 导入失败: {e}")
        return 1
    
    # 导入检测器节点 - 直接从源目录导入
    try:
        src_path = "/home/sirius/ssd/ros2workspace/src/yolo_detector"
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        
        from yolo_detector.detector_node import main as detector_main
        print("✓ YOLO检测器节点导入成功")
    except ImportError as e:
        print(f"✗ YOLO检测器节点导入失败: {e}")
        print("可用的包路径:")
        for path in sys.path:
            print(f"  {path}")
        return 1
    
    # 运行检测器节点
    print("启动YOLO检测器节点...")
    try:
        detector_main(sys.argv[1:])  # 传递命令行参数
        return 0
    except Exception as e:
        print(f"运行检测器节点时出错: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
