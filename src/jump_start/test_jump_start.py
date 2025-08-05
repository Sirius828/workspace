#!/usr/bin/env python3
"""
Jump Start 包测试脚本
验证FSM控制器基本功能
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import signal
import sys

def test_package_installation():
    """测试包安装"""
    print("🔍 Testing package installation...")
    
    try:
        # 测试导入模块
        import jump_start.fsm_controller
        import jump_start.mission_executor
        print("✅ Python modules imported successfully")
        
        # 测试ROS2命令
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        if 'jump_start' in result.stdout:
            print("✅ jump_start package found in ROS2")
        else:
            print("❌ jump_start package not found in ROS2")
            
    except Exception as e:
        print(f"❌ Import test failed: {e}")
        return False
    
    return True

def run_fsm_controller_test():
    """运行FSM控制器测试"""
    print("\n🚀 Starting FSM Controller test...")
    
    # 启动FSM控制器
    try:
        fsm_process = subprocess.Popen([
            'python3', '-c', 
            '''
import rclpy
from jump_start.fsm_controller import SimpleRobotFSM
import time

rclpy.init()
node = SimpleRobotFSM()
print("FSM Controller started successfully")

# 运行5秒钟
start_time = time.time()
while time.time() - start_time < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)

print("FSM Controller test completed")
node.destroy_node()
rclpy.shutdown()
            '''
        ], cwd='/home/sirius/ssd/ros2workspace')
        
        # 等待进程完成
        fsm_process.wait(timeout=10)
        
        if fsm_process.returncode == 0:
            print("✅ FSM Controller test passed")
            return True
        else:
            print("❌ FSM Controller test failed")
            return False
            
    except subprocess.TimeoutExpired:
        fsm_process.kill()
        print("⏰ FSM Controller test timeout")
        return False
    except Exception as e:
        print(f"❌ FSM Controller test error: {e}")
        return False

def main():
    """主测试函数"""
    print("="*50)
    print("🤖 JUMP START PACKAGE TEST")
    print("="*50)
    
    tests_passed = 0
    total_tests = 2
    
    # 测试1: 包安装
    if test_package_installation():
        tests_passed += 1
    
    # 测试2: FSM控制器
    if run_fsm_controller_test():
        tests_passed += 1
    
    # 结果
    print(f"\n{'='*50}")
    print(f"📊 TEST RESULTS: {tests_passed}/{total_tests} tests passed")
    print("="*50)
    
    if tests_passed == total_tests:
        print("🎉 All tests passed! jump_start package is ready to use.")
        print("\nTo start using the package:")
        print("1. Launch FSM Controller: ros2 run jump_start fsm_controller")
        print("2. Launch Mission Executor: ros2 run jump_start mission_executor")
        print("3. Or use launch file: ros2 launch jump_start jump_start.launch.py")
    else:
        print("❌ Some tests failed. Please check the installation.")
    
    return tests_passed == total_tests

if __name__ == '__main__':
    try:
        # 设置环境
        import os
        os.chdir('/home/sirius/ssd/ros2workspace')
        os.system('source install/setup.bash')
        
        success = main()
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        sys.exit(1)
