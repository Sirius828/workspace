#!/usr/bin/env python3
"""
简化版键盘遥控节点
更简单的键盘控制界面，适合快速测试
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time


class SimpleKeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_keyboard_teleop_node')
        
        # 参数
        self.step_size = 0.05
        self.max_vel = 1.0
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 当前速度
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        # 线程控制
        self.running = True
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # 启动键盘监听
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.print_instructions()
    
    def print_instructions(self):
        """打印操作说明"""
        print("\n简化键盘控制:")
        print("  w/s: 前进/后退")
        print("  a/d: 左转/右转") 
        print("  q/e: 左移/右移")
        print("  空格: 停止")
        print("  Ctrl+C: 退出")
        print(f"  步进: {self.step_size}")
        print("-" * 30)
    
    def get_key(self):
        """获取单个按键"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def keyboard_listener(self):
        """键盘监听线程"""
        while self.running and rclpy.ok():
            try:
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    self.running = False
                    break
                
                # 处理按键
                if key == 'w':
                    self.vx = min(self.vx + self.step_size, self.max_vel)
                elif key == 's':
                    self.vx = max(self.vx - self.step_size, -self.max_vel)
                elif key == 'a':
                    self.omega = min(self.omega + self.step_size, self.max_vel)
                elif key == 'd':
                    self.omega = max(self.omega - self.step_size, -self.max_vel)
                elif key == 'q':
                    self.vy = min(self.vy + self.step_size, self.max_vel)
                elif key == 'e':
                    self.vy = max(self.vy - self.step_size, -self.max_vel)
                elif key == ' ':
                    self.vx = self.vy = self.omega = 0.0
                    print("停止!")
                
                # 显示当前状态
                print(f"\rvx:{self.vx:+.2f} vy:{self.vy:+.2f} ω:{self.omega:+.2f}", end="", flush=True)
                
            except Exception as e:
                print(f"\n键盘监听出错: {e}")
                break
    
    def publish_velocity(self):
        """发布速度"""
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.omega
        self.cmd_vel_pub.publish(twist)
    
    def destroy_node(self):
        """清理"""
        self.running = False
        # 发送停止命令
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        print("\n程序退出")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleKeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
