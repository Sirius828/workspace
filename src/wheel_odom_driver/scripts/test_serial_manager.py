#!/usr/bin/env python3
"""
统一串口管理节点快速测试脚本
用于发送测试命令验证底盘和云台控制功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time
import math


class SerialManagerTester(Node):
    def __init__(self):
        super().__init__('serial_manager_tester')
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_cmd_pub = self.create_publisher(Vector3, '/gimbal/angle_cmd', 10)
        
        # 创建订阅器监控反馈
        self.gimbal_feedback_sub = self.create_subscription(
            Vector3,
            '/gimbal/current_angle',
            self.gimbal_feedback_callback,
            10
        )
        
        # 测试参数
        self.test_running = False
        self.start_time = None
        
        self.get_logger().info('串口管理节点测试器已启动')
        self.print_menu()
    
    def gimbal_feedback_callback(self, msg):
        """云台反馈回调"""
        self.get_logger().info(f'云台反馈: 偏航={msg.x:.1f}°, 俯仰={msg.y:.1f}°')
    
    def print_menu(self):
        """打印测试菜单"""
        print("\n" + "="*50)
        print("🧪 统一串口管理节点测试菜单")
        print("="*50)
        print("1. 测试底盘前进")
        print("2. 测试底盘后退")
        print("3. 测试底盘左右移动")
        print("4. 测试底盘旋转")
        print("5. 测试云台偏航")
        print("6. 测试云台俯仰")
        print("7. 综合测试（自动循环）")
        print("8. 停止所有运动")
        print("0. 退出")
        print("="*50)
    
    def test_chassis_forward(self):
        """测试底盘前进"""
        self.get_logger().info('测试底盘前进...')
        cmd = Twist()
        cmd.linear.x = 0.2  # 0.2 m/s 前进
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        self.stop_chassis()
    
    def test_chassis_backward(self):
        """测试底盘后退"""
        self.get_logger().info('测试底盘后退...')
        cmd = Twist()
        cmd.linear.x = -0.2  # 0.2 m/s 后退
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        self.stop_chassis()
    
    def test_chassis_strafe(self):
        """测试底盘左右移动"""
        self.get_logger().info('测试底盘左移...')
        cmd = Twist()
        cmd.linear.y = 0.2  # 0.2 m/s 左移
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        
        self.get_logger().info('测试底盘右移...')
        cmd.linear.y = -0.2  # 0.2 m/s 右移
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        self.stop_chassis()
    
    def test_chassis_rotation(self):
        """测试底盘旋转"""
        self.get_logger().info('测试底盘左转...')
        cmd = Twist()
        cmd.angular.z = 0.5  # 0.5 rad/s 左转
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        
        self.get_logger().info('测试底盘右转...')
        cmd.angular.z = -0.5  # 0.5 rad/s 右转
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        self.stop_chassis()
    
    def test_gimbal_yaw(self):
        """测试云台偏航"""
        self.get_logger().info('测试云台偏航...')
        
        # 左转
        cmd = Vector3()
        cmd.x = 30.0  # 30度偏航
        cmd.y = 0.0
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(2.0)
        
        # 右转
        cmd.x = -30.0  # -30度偏航
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(2.0)
        
        # 回中
        cmd.x = 0.0
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(1.0)
    
    def test_gimbal_pitch(self):
        """测试云台俯仰"""
        self.get_logger().info('测试云台俯仰...')
        
        # 上仰
        cmd = Vector3()
        cmd.x = 0.0
        cmd.y = 20.0  # 20度俯仰
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(2.0)
        
        # 下俯
        cmd.y = -20.0  # -20度俯仰
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(2.0)
        
        # 回中
        cmd.y = 0.0
        self.gimbal_cmd_pub.publish(cmd)
        time.sleep(1.0)
    
    def test_comprehensive(self):
        """综合测试"""
        self.get_logger().info('开始综合测试...')
        self.test_running = True
        self.start_time = time.time()
        
        # 创建定时器进行周期性测试
        self.test_timer = self.create_timer(0.1, self.comprehensive_test_loop)
    
    def comprehensive_test_loop(self):
        """综合测试循环"""
        if not self.test_running:
            return
        
        elapsed = time.time() - self.start_time
        
        # 底盘运动（圆形轨迹）
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1 * math.cos(elapsed * 0.5)
        cmd_vel.linear.y = 0.1 * math.sin(elapsed * 0.5)
        cmd_vel.angular.z = 0.2 * math.sin(elapsed * 0.3)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 云台运动（8字形）
        gimbal_cmd = Vector3()
        gimbal_cmd.x = 20.0 * math.sin(elapsed * 0.8)  # 偏航
        gimbal_cmd.y = 10.0 * math.sin(elapsed * 1.2)  # 俯仰
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        # 10秒后停止
        if elapsed > 10.0:
            self.stop_comprehensive_test()
    
    def stop_comprehensive_test(self):
        """停止综合测试"""
        self.test_running = False
        if hasattr(self, 'test_timer'):
            self.test_timer.cancel()
        self.stop_all()
        self.get_logger().info('综合测试完成')
    
    def stop_chassis(self):
        """停止底盘"""
        cmd = Twist()  # 所有速度为0
        self.cmd_vel_pub.publish(cmd)
    
    def stop_gimbal(self):
        """云台回中"""
        cmd = Vector3()  # 所有角度为0
        self.gimbal_cmd_pub.publish(cmd)
    
    def stop_all(self):
        """停止所有运动"""
        self.get_logger().info('停止所有运动')
        self.stop_chassis()
        self.stop_gimbal()
    
    def run_interactive(self):
        """交互式运行"""
        while rclpy.ok():
            try:
                choice = input("\n请选择测试项目 (0-8): ").strip()
                
                if choice == '1':
                    self.test_chassis_forward()
                elif choice == '2':
                    self.test_chassis_backward()
                elif choice == '3':
                    self.test_chassis_strafe()
                elif choice == '4':
                    self.test_chassis_rotation()
                elif choice == '5':
                    self.test_gimbal_yaw()
                elif choice == '6':
                    self.test_gimbal_pitch()
                elif choice == '7':
                    self.test_comprehensive()
                elif choice == '8':
                    self.stop_all()
                elif choice == '0':
                    self.get_logger().info('退出测试')
                    break
                else:
                    print("无效选择，请重新输入")
                    self.print_menu()
                    
            except KeyboardInterrupt:
                self.get_logger().info('用户中断测试')
                break
            except EOFError:
                break
        
        self.stop_all()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = SerialManagerTester()
        
        # 等待节点初始化
        time.sleep(1.0)
        
        # 开始交互式测试
        tester.run_interactive()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
