#!/usr/bin/env python3
"""
简单可靠的键盘遥控节点
使用简化的输入方法，确保在ROS2环境中稳定工作
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import signal


class ReliableKeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('reliable_keyboard_teleop_node')
        
        # 参数
        self.step_size = 0.05
        self.max_vel = 1.0
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 当前速度
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 保存终端设置
        self.old_settings = None
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        
        # 创建定时器发布速度
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('可靠键盘遥控节点已启动')
        self.print_instructions()
        
        # 开始主循环
        self.main_loop()
    
    def print_instructions(self):
        """打印说明"""
        print("\n简单键盘控制:")
        print("w/s: 前进/后退")
        print("a/d: 左转/右转")  
        print("q/e: 左移/右移")
        print("空格: 停止")
        print("Ctrl+C: 退出")
        print(f"步进: {self.step_size}")
        print("-" * 30)
        self.print_status()
    
    def print_status(self):
        """打印状态"""
        print(f"\rvx:{self.vx:+.2f} vy:{self.vy:+.2f} ω:{self.omega:+.2f} >> ", end="", flush=True)
    
    def get_key(self):
        """获取按键"""
        if sys.stdin.isatty():
            return sys.stdin.read(1)
        return None
    
    def main_loop(self):
        """主循环"""
        try:
            while rclpy.ok():
                # 处理ROS回调
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 处理键盘输入
                key = self.get_key()
                if key:
                    if ord(key) == 3:  # Ctrl+C
                        break
                    self.process_key(key)
                    
        except Exception as e:
            self.get_logger().error(f'主循环出错: {e}')
    
    def process_key(self, key):
        """处理按键"""
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
            print("\n停止!")
        
        self.print_status()
    
    def publish_velocity(self):
        """发布速度"""
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.omega
        self.cmd_vel_pub.publish(twist)
    
    def signal_handler(self, signum, frame):
        """信号处理"""
        self.get_logger().info('\n收到退出信号')
        self.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    
    def cleanup(self):
        """清理"""
        # 恢复终端设置
        if self.old_settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        # 发送停止命令
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        print("\n发送停止命令")
    
    def destroy_node(self):
        """节点销毁"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ReliableKeyboardTeleopNode()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"启动失败: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
