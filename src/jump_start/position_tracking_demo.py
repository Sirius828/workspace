#!/usr/bin/env python3
"""
测试位置跟踪逻辑的演示脚本
验证机器人必须在x>2的位置才能进行有效云台跟踪
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import time
import threading


class PositionTrackingDemo(Node):
    def __init__(self):
        super().__init__('position_tracking_demo')
        
        # 发布者
        self.command_pub = self.create_publisher(String, '/mission_command', 10)
        self.target_pixel_pub = self.create_publisher(Point, '/target_position_pixel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/chassis/odom', 10)
        
        # 订阅者
        self.fsm_state_sub = self.create_subscription(String, '/fsm_state', self.state_callback, 10)
        self.fsm_status_sub = self.create_subscription(String, '/fsm_status', self.status_callback, 10)
        self.victory_sub = self.create_subscription(Bool, '/victory', self.victory_callback, 10)
        
        self.current_state = "IDLE"
        self.get_logger().info('Position Tracking Demo started')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f'FSM State: {self.current_state}')
    
    def status_callback(self, msg):
        # 解析状态信息
        try:
            status = eval(msg.data)
            if status.get('in_valid_tracking_zone', False):
                self.get_logger().info(f"✅ In valid zone: x={status.get('current_x', 0):.2f}m, tracking_time={status.get('victory_time', 0):.1f}s")
            else:
                self.get_logger().warn(f"🚫 Not in valid zone: x={status.get('current_x', 0):.2f}m <= {status.get('valid_tracking_threshold', 2.0)}m")
        except:
            pass
    
    def victory_callback(self, msg):
        if msg.data:
            self.get_logger().info('🏆 VICTORY ACHIEVED!')
    
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
    
    def publish_fake_odom(self, x, y, z=0.0):
        """发布虚拟里程计数据"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        self.odom_pub.publish(msg)
        self.get_logger().info(f'Published position: ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def publish_fake_target(self, center=True):
        """发布虚拟目标像素位置"""
        msg = Point()
        if center:
            msg.x = 320.0  # 图像中心
            msg.y = 240.0
        else:
            msg.x = 100.0  # 偏离中心
            msg.y = 100.0
        msg.z = 0.9  # 置信度
        self.target_pixel_pub.publish(msg)
        self.get_logger().info(f'Published target at: ({msg.x:.1f}, {msg.y:.1f})')
    
    def run_position_test(self):
        """运行位置测试"""
        self.get_logger().info('\n🚀 Starting Position-Based Tracking Test')
        self.get_logger().info('='*60)
        
        # 测试1：在无效区域（x=1.0）尝试跟踪
        self.get_logger().info('\n📍 Test 1: Tracking in invalid zone (x=1.0 < 2.0)')
        self.publish_fake_odom(1.0, 2.0)
        time.sleep(1.0)
        
        self.send_command('track')
        time.sleep(2.0)
        
        # 发布目标在中心
        for i in range(5):
            self.publish_fake_target(center=True)
            time.sleep(0.5)
            self.get_logger().info(f'Target in center #{i+1}/5 - Should NOT accumulate time!')
        
        # 测试2：移动到有效区域（x=2.5）
        self.get_logger().info('\n📍 Test 2: Moving to valid zone (x=2.5 > 2.0)')
        self.publish_fake_odom(2.5, 2.0)
        time.sleep(1.0)
        
        # 发布目标在中心
        for i in range(6):
            self.publish_fake_target(center=True)
            time.sleep(0.5)
            self.get_logger().info(f'Target in center #{i+1}/6 - Should accumulate time!')
        
        # 测试3：离开有效区域（x=1.5）
        self.get_logger().info('\n📍 Test 3: Leaving valid zone (x=1.5 < 2.0)')
        self.publish_fake_odom(1.5, 2.0)
        time.sleep(1.0)
        
        # 继续发布目标
        for i in range(3):
            self.publish_fake_target(center=True)
            time.sleep(0.5)
            self.get_logger().info(f'Target in center #{i+1}/3 - Should reset timer!')
        
        # 测试4：回到有效区域并完成跟踪
        self.get_logger().info('\n📍 Test 4: Return to valid zone and complete tracking')
        self.publish_fake_odom(3.0, 2.0)
        time.sleep(1.0)
        
        # 连续发布目标直到胜利
        for i in range(8):
            self.publish_fake_target(center=True)
            time.sleep(0.3)
            self.get_logger().info(f'Target tracking #{i+1}/8 - Working towards victory!')
        
        self.get_logger().info('\n✅ Position test completed!')
    
    def interactive_demo(self):
        """交互式演示"""
        while rclpy.ok():
            print("\n" + "="*50)
            print("🎯 POSITION-BASED TRACKING DEMO")
            print("="*50)
            print("1. Start tracking")
            print("2. Set position (x=1.0, invalid zone)")
            print("3. Set position (x=2.5, valid zone)")
            print("4. Set position (x=3.5, valid zone)")
            print("5. Publish target in center")
            print("6. Publish target off-center")
            print("7. Run automated position test")
            print("8. Reset FSM")
            print("q. Quit")
            print("-"*30)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    self.send_command('track')
                elif choice == '2':
                    self.publish_fake_odom(1.0, 2.0)
                elif choice == '3':
                    self.publish_fake_odom(2.5, 2.0)
                elif choice == '4':
                    self.publish_fake_odom(3.5, 2.0)
                elif choice == '5':
                    self.publish_fake_target(center=True)
                elif choice == '6':
                    self.publish_fake_target(center=False)
                elif choice == '7':
                    self.run_position_test()
                elif choice == '8':
                    self.send_command('reset')
                else:
                    print("Invalid choice")
                    
                # 让ROS处理回调
                rclpy.spin_once(self, timeout_sec=0.1)
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 Demo exiting...")


def main(args=None):
    rclpy.init(args=args)
    demo = PositionTrackingDemo()
    
    try:
        demo.interactive_demo()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
