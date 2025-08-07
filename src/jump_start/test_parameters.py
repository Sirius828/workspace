#!/usr/bin/env python3
"""
配置参数测试脚本
"""

import rclpy
from jump_start.fsm_controller import SimpleRobotFSM
import yaml

def test_parameters():
    print("🔧 Testing FSM Controller Parameters")
    print("=" * 50)
    
    rclpy.init()
    
    try:
        fsm = SimpleRobotFSM()
        
        print("📍 Target Position:")
        print(f"  x: {fsm.target_x}")
        print(f"  y: {fsm.target_y}") 
        print(f"  z: {fsm.target_z}")
        
        print("\n🎯 Tracking Parameters:")
        print(f"  Position tolerance: {fsm.position_tolerance}m")
        print(f"  Victory time threshold: {fsm.victory_time_threshold}s")
        print(f"  Valid tracking X: {fsm.valid_tracking_x}m")
        
        print("\n📹 Video Recording:")
        print(f"  Enabled: {fsm.enable_video_recording}")
        print(f"  Buffer duration: {fsm.video_buffer_duration}s")
        print(f"  Video FPS: {fsm.video_fps}")
        print(f"  Camera topic: {fsm.camera_topic}")
        print(f"  YOLO topic: {fsm.yolo_topic}")
        
        print("\n📊 Image Parameters:")
        print(f"  Image width: {fsm.image_width}px")
        print(f"  Image height: {fsm.image_height}px")
        print(f"  Image center: ({fsm.image_center_x}, {fsm.image_center_y})")
        
        print("\n✅ All parameters loaded successfully!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    test_parameters()
