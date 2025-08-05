#!/usr/bin/env python3
"""
测试FSM控制器的位置逻辑功能
"""

import rclpy
from jump_start.fsm_controller import SimpleRobotFSM
import time

def test_position_logic():
    """测试位置约束逻辑"""
    print("🔍 Testing FSM Controller with position logic...")
    
    # 初始化rclpy
    rclpy.init()
    
    try:
        # 创建FSM控制器
        node = SimpleRobotFSM()
        
        print('✅ FSM Controller with position logic loaded successfully!')
        print(f'📍 Valid tracking X threshold: {node.valid_tracking_x}m')
        print(f'🗺️  Map size: {node.map_size}x{node.map_size}m')
        print(f'⏱️  Victory time threshold: {node.victory_time_threshold}s')
        print(f'📏 Position tolerance: {node.position_tolerance}m')
        print(f'🎯 Pixel tolerance: {node.pixel_tolerance}px')
        print(f'🖼️  Image size: {node.image_width}x{node.image_height}')
        
        # 测试位置逻辑
        print("\n🧪 Testing position validation logic:")
        
        # 测试无效位置（x <= 2.0）
        test_positions = [
            (0.0, 2.0, "起始位置"),
            (1.0, 2.0, "半场前"),
            (2.0, 2.0, "半场线上"),
            (2.1, 2.0, "刚过半场"),
            (3.0, 2.0, "有效区域"),
            (3.9, 2.0, "接近边界")
        ]
        
        for x, y, description in test_positions:
            is_valid = x > node.valid_tracking_x
            status = "✅ 有效" if is_valid else "❌ 无效"
            print(f"  位置 ({x}, {y}) - {description}: {status}")
        
        print("\n🎮 Position logic test completed!")
        
        # 清理
        node.destroy_node()
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False
    finally:
        rclpy.shutdown()
    
    return True

if __name__ == '__main__':
    test_position_logic()
