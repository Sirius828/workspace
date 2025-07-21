#!/usr/bin/env python3
"""
底盘功能测试脚本 - 验证硬件接口和控制器功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math

class ChassisTest(Node):
    def __init__(self):
        super().__init__('chassis_test')
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 状态变量
        self.latest_odom = None
        self.latest_joints = None
        
        self.get_logger().info("🤖 底盘测试节点启动")
        
    def odom_callback(self, msg):
        self.latest_odom = msg
        
    def joint_callback(self, msg):
        self.latest_joints = msg
        
    def print_status(self):
        """打印当前状态"""
        print("\n" + "="*60)
        print("📊 底盘状态报告")
        print("="*60)
        
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            vel = self.latest_odom.twist.twist
            print(f"🗺️  里程计位置: x={pos.x:.3f}m, y={pos.y:.3f}m")
            print(f"🏃 里程计速度: 线速度={vel.linear.x:.3f}m/s, 角速度={vel.angular.z:.3f}rad/s")
        else:
            print("❌ 未收到里程计数据")
            
        if self.latest_joints:
            if len(self.latest_joints.position) >= 2:
                left_pos = self.latest_joints.position[0]
                right_pos = self.latest_joints.position[1]
                print(f"⚙️  关节位置: 左轮={left_pos:.3f}rad, 右轮={right_pos:.3f}rad")
            
            if len(self.latest_joints.velocity) >= 2:
                left_vel = self.latest_joints.velocity[0]
                right_vel = self.latest_joints.velocity[1]
                print(f"🔄 关节速度: 左轮={left_vel:.3f}rad/s, 右轮={right_vel:.3f}rad/s")
        else:
            print("❌ 未收到关节状态数据")
    
    def send_command(self, linear, angular, duration=2.0):
        """发送速度命令"""
        print(f"\n🚀 发送命令: 线速度={linear:.2f}m/s, 角速度={angular:.2f}rad/s")
        
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10Hz
        
        while time.time() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
            
        # 停止
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        print(f"✅ 命令执行完成 ({duration}秒)")
        
    def run_tests(self):
        """运行测试序列"""
        print("\n🧪 开始底盘测试序列...")
        
        # 等待数据
        print("\n⏳ 等待传感器数据...")
        timeout = 10.0
        start_time = time.time()
        
        while (not self.latest_odom or not self.latest_joints) and \
              (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.latest_odom or not self.latest_joints:
            print("❌ 超时：未收到传感器数据")
            return False
            
        print("✅ 传感器数据正常")
        
        # 测试1: 初始状态
        print("\n🔍 测试1: 初始状态检查")
        self.print_status()
        time.sleep(2)
        
        # 测试2: 前进
        print("\n🔍 测试2: 前进运动")
        self.send_command(0.1, 0.0, 3.0)
        time.sleep(1)
        self.print_status()
        
        # 测试3: 后退  
        print("\n🔍 测试3: 后退运动")
        self.send_command(-0.1, 0.0, 3.0)
        time.sleep(1)
        self.print_status()
        
        # 测试4: 左转
        print("\n🔍 测试4: 左转运动")
        self.send_command(0.0, 0.5, 3.0)
        time.sleep(1)
        self.print_status()
        
        # 测试5: 右转
        print("\n🔍 测试5: 右转运动")
        self.send_command(0.0, -0.5, 3.0)
        time.sleep(1)
        self.print_status()
        
        # 测试6: 组合运动
        print("\n🔍 测试6: 弧线运动")
        self.send_command(0.1, 0.3, 4.0)
        time.sleep(1)
        self.print_status()
        
        print("\n🎉 所有测试完成！")
        return True

def main():
    rclpy.init()
    
    test_node = ChassisTest()
    
    try:
        test_node.run_tests()
    except KeyboardInterrupt:
        print("\n🛑 用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试发生错误: {e}")
    finally:
        # 确保停止机器人
        cmd = Twist()
        test_node.cmd_pub.publish(cmd)
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
