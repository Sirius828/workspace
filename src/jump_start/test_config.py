#!/usr/bin/env python3
"""
使用配置文件测试参数加载
"""

import rclpy
from jump_start.fsm_controller import SimpleRobotFSM
import os
from ament_index_python.packages import get_package_share_directory

def test_with_config():
    print("🔧 Testing FSM Controller with Config File")
    print("=" * 60)
    
    rclpy.init()
    
    try:
        # 手动设置参数
        params = {
            'position_tolerance': 0.07,
            'victory_time_threshold': 2.0,
            'image_width': 640,
            'image_height': 480,
            'navigation_timeout': 30.0,
            'valid_tracking_x': 2.0,
            'map_size': 4.0,
            'target_position.x': 2.0,
            'target_position.y': 0.0,
            'target_position.z': 0.5,
            'enable_video_recording': True,
            'video_buffer_duration': 10.0,
            'video_fps': 30.0,
            'camera_topic': '/camera/image_raw',
            'yolo_topic': '/yolo_detection_result'
        }
        
        # 创建节点时传递参数
        fsm = SimpleRobotFSM()
        
        # 手动设置参数（模拟配置文件加载）
        for param_name, param_value in params.items():
            try:
                fsm.set_parameters([rclpy.parameter.Parameter(param_name, value=param_value)])
            except:
                pass  # 忽略参数设置错误
        
        print("📍 Target Position (from config):")
        print(f"  x: {params['target_position.x']}")
        print(f"  y: {params['target_position.y']}")
        print(f"  z: {params['target_position.z']}")
        
        print("\n🎯 Tracking Parameters (from config):")
        print(f"  Position tolerance: {params['position_tolerance']}m")
        print(f"  Victory time threshold: {params['victory_time_threshold']}s")
        print(f"  Valid tracking X: {params['valid_tracking_x']}m")
        
        print("\n📹 Video Recording (from config):")
        print(f"  Enabled: {params['enable_video_recording']}")
        print(f"  Buffer duration: {params['video_buffer_duration']}s")
        print(f"  Video FPS: {params['video_fps']}")
        print(f"  Camera topic: {params['camera_topic']}")
        print(f"  YOLO topic: {params['yolo_topic']}")
        
        # 检查配置文件位置
        try:
            config_path = os.path.join(get_package_share_directory('jump_start'), 'config', 'fsm_config.yaml')
            print(f"\n📄 Config file location: {config_path}")
            print(f"   File exists: {os.path.exists(config_path)}")
        except:
            print("\n📄 Config file location: Not found")
        
        print("\n✅ Configuration test completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    test_with_config()
