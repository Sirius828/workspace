#!/usr/bin/env python3
"""
位置约束测试脚本
测试基于x坐标的有效跟踪区域逻辑
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
import time
import math

class PositionTrackingTest(Node):
    def __init__(self):
        super().__init__('position_tracking_test')
        
        # 发布者
        self.mission_command_pub = self.create_publisher(String, '/mission_command', 10)
        self.target_pixel_pub = self.create_publisher(Point, '/target_position_pixel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/chassis/odom', 10)
        
        # 订阅者
        self.fsm_state_sub = self.create_subscription(String, '/fsm_state', self.fsm_state_callback, 10)
        self.victory_sub = self.create_subscription(Bool, '/victory', self.victory_callback, 10)
        self.status_sub = self.create_subscription(String, '/fsm_status', self.status_callback, 10)
        
        self.current_state = "IDLE"
        self.victory_achieved = False
        self.last_status = ""
        
        self.get_logger().info('🧪 Position Tracking Test Node started')
    
    def fsm_state_callback(self, msg):
        if self.current_state != msg.data:
            self.current_state = msg.data
            self.get_logger().info(f'FSM State: {self.current_state}')
    
    def victory_callback(self, msg):
        if msg.data and not self.victory_achieved:
            self.victory_achieved = True
            self.get_logger().info('🎉 VICTORY ACHIEVED!')
    
    def status_callback(self, msg):
        # 只在状态发生变化时记录
        if "in_valid_tracking_zone" in msg.data and msg.data != self.last_status:
            self.last_status = msg.data
            if "True" in msg.data:
                self.get_logger().info('✅ Robot entered valid tracking zone (x > 2.0)')
            else:
                self.get_logger().info('❌ Robot in invalid zone (x ≤ 2.0)')
    
    def send_command(self, command):
        """发送任务命令"""
        msg = String()
        msg.data = command
        self.mission_command_pub.publish(msg)
        self.get_logger().info(f'📤 Sent command: {command}')
    
    def publish_robot_position(self, x, y, z=0.0):
        """发布机器人位置"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'📍 Robot position: ({x:.1f}, {y:.1f}, {z:.1f})')
    
    def publish_target_pixel(self, x=320, y=240, confidence=0.9):
        """发布目标像素位置（默认在图像中心）"""
        pixel_msg = Point()
        pixel_msg.x = float(x)
        pixel_msg.y = float(y)
        pixel_msg.z = float(confidence)
        
        self.target_pixel_pub.publish(pixel_msg)
        # self.get_logger().info(f'🎯 Target pixel: ({x}, {y})')
    
    def test_invalid_zone_tracking(self):
        """测试在无效区域的跟踪"""
        self.get_logger().info('\n🔬 Testing tracking in INVALID zone (x ≤ 2.0)')
        
        # 1. 设置机器人在无效区域
        self.publish_robot_position(1.5, 2.0)
        time.sleep(0.5)
        
        # 2. 开始跟踪
        self.send_command('track')
        time.sleep(1.0)
        
        # 3. 持续发布目标在中心5秒
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self.publish_target_pixel(320, 240)  # 图像中心
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 4. 检查是否胜利（应该不会胜利）
        if self.victory_achieved:
            self.get_logger().error('❌ Test FAILED: Victory achieved in invalid zone!')
            return False
        else:
            self.get_logger().info('✅ Test PASSED: No victory in invalid zone')
            return True
    
    def test_valid_zone_tracking(self):
        """测试在有效区域的跟踪"""
        self.get_logger().info('\n🔬 Testing tracking in VALID zone (x > 2.0)')
        
        # 重置
        self.victory_achieved = False
        self.send_command('reset')
        time.sleep(1.0)
        
        # 1. 设置机器人在有效区域
        self.publish_robot_position(2.5, 2.0)
        time.sleep(0.5)
        
        # 2. 开始跟踪
        self.send_command('track')
        time.sleep(1.0)
        
        # 3. 持续发布目标在中心3秒（超过阈值2秒）
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.publish_target_pixel(320, 240)  # 图像中心
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 4. 检查是否胜利（应该胜利）
        if self.victory_achieved:
            self.get_logger().info('✅ Test PASSED: Victory achieved in valid zone')
            return True
        else:
            self.get_logger().error('❌ Test FAILED: No victory in valid zone!')
            return False
    
    def test_zone_transition(self):
        """测试区域切换时的计时重置"""
        self.get_logger().info('\n🔬 Testing zone transition and timer reset')
        
        # 重置
        self.victory_achieved = False
        self.send_command('reset')
        time.sleep(1.0)
        
        # 1. 在有效区域开始跟踪
        self.publish_robot_position(2.5, 2.0)
        time.sleep(0.5)
        self.send_command('track')
        time.sleep(0.5)
        
        # 2. 在有效区域跟踪1秒（不足2秒阈值）
        start_time = time.time()
        while time.time() - start_time < 1.0:
            self.publish_target_pixel(320, 240)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 3. 移动到无效区域
        self.get_logger().info('🔄 Moving to invalid zone...')
        self.publish_robot_position(1.5, 2.0)
        time.sleep(0.5)
        
        # 4. 继续跟踪1秒（计时器应该重置）
        start_time = time.time()
        while time.time() - start_time < 1.0:
            self.publish_target_pixel(320, 240)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 5. 回到有效区域
        self.get_logger().info('🔄 Moving back to valid zone...')
        self.publish_robot_position(2.5, 2.0)
        time.sleep(0.5)
        
        # 6. 继续跟踪3秒（需要重新计时满2秒）
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.publish_target_pixel(320, 240)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 7. 检查胜利（应该胜利，说明计时器正确重置了）
        if self.victory_achieved:
            self.get_logger().info('✅ Test PASSED: Timer correctly reset during zone transition')
            return True
        else:
            self.get_logger().error('❌ Test FAILED: Timer not reset properly')
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('🚀 Starting Position Tracking Tests')
        self.get_logger().info('='*60)
        
        # 等待系统准备
        time.sleep(2.0)
        
        tests = [
            ("Invalid Zone Tracking", self.test_invalid_zone_tracking),
            ("Valid Zone Tracking", self.test_valid_zone_tracking),
            ("Zone Transition", self.test_zone_transition),
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, test_func in tests:
            self.get_logger().info(f'\n📋 Running test: {test_name}')
            self.get_logger().info('-' * 40)
            
            try:
                if test_func():
                    passed += 1
                    self.get_logger().info(f'✅ {test_name} PASSED')
                else:
                    self.get_logger().error(f'❌ {test_name} FAILED')
            except Exception as e:
                self.get_logger().error(f'❌ {test_name} ERROR: {e}')
            
            # 测试间隔
            time.sleep(2.0)
        
        # 总结
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'📊 Test Results: {passed}/{total} tests passed')
        
        if passed == total:
            self.get_logger().info('🎉 All tests PASSED! Position-based tracking works correctly.')
        else:
            self.get_logger().error(f'❌ {total - passed} test(s) FAILED!')
        
        return passed == total

def main(args=None):
    rclpy.init(args=args)
    test_node = PositionTrackingTest()
    
    try:
        # 运行测试
        success = test_node.run_all_tests()
        
        if success:
            test_node.get_logger().info('\n✅ Position tracking system is working correctly!')
        else:
            test_node.get_logger().error('\n❌ Position tracking system has issues!')
            
    except KeyboardInterrupt:
        test_node.get_logger().info('\n🛑 Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
