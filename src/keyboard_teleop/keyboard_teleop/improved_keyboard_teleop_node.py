#!/usr/bin/env python3
"""
改进的键盘遥控节点
使用更可靠的键盘输入方法，解决输入阻塞问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import os


class ImprovedKeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('improved_keyboard_teleop_node')
        
        # 声明参数
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('step_size', 0.05)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        
        # 获取参数
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # 当前速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 保存终端设置
        if os.isatty(sys.stdin.fileno()):
            self.old_attr = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('改进的键盘遥控节点已启动')
        self.get_logger().info(f'发布话题: {self.cmd_vel_topic}')
        self.get_logger().info(f'步进大小: {self.step_size}')
        self.print_instructions()
    
    def print_instructions(self):
        """打印操作说明"""
        print("\n" + "="*50)
        print("键盘遥控控制:")
        print("="*50)
        print("移动控制:")
        print("  w/s: 前进/后退")
        print("  a/d: 左转/右转")
        print("  q/e: 左移/右移")
        print("功能控制:")
        print("  空格: 立即停止")
        print("  r: 重置速度")
        print("  Ctrl+C: 退出")
        print(f"步进: {self.step_size} | 最大线速度: {self.max_linear_vel}")
        print("="*50)
        self.print_status()
    
    def print_status(self):
        """打印当前状态"""
        print(f"\r状态: vx={self.current_linear_x:+.3f} vy={self.current_linear_y:+.3f} "
              f"ω={self.current_angular_z:+.3f} | 按键:", end="", flush=True)
    
    def get_key(self):
        """非阻塞方式获取键盘输入"""
        if os.isatty(sys.stdin.fileno()):
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        return None
    
    def timer_callback(self):
        """定时器回调，处理键盘输入和发布速度"""
        # 处理键盘输入
        key = self.get_key()
        if key:
            self.process_key(key)
        
        # 发布速度
        self.publish_velocity()
    
    def process_key(self, key):
        """处理按键"""
        if key == '\x03':  # Ctrl+C
            self.get_logger().info('\n收到退出信号')
            rclpy.shutdown()
            return
        
        if key == 'w':
            self.current_linear_x = min(self.current_linear_x + self.step_size, self.max_linear_vel)
            print(f"w", end="", flush=True)
        elif key == 's':
            self.current_linear_x = max(self.current_linear_x - self.step_size, -self.max_linear_vel)
            print(f"s", end="", flush=True)
        elif key == 'a':
            self.current_angular_z = min(self.current_angular_z + self.step_size, self.max_angular_vel)
            print(f"a", end="", flush=True)
        elif key == 'd':
            self.current_angular_z = max(self.current_angular_z - self.step_size, -self.max_angular_vel)
            print(f"d", end="", flush=True)
        elif key == 'q':
            self.current_linear_y = min(self.current_linear_y + self.step_size, self.max_linear_vel)
            print(f"q", end="", flush=True)
        elif key == 'e':
            self.current_linear_y = max(self.current_linear_y - self.step_size, -self.max_linear_vel)
            print(f"e", end="", flush=True)
        elif key == ' ':
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
            print("\n[停止]", end="", flush=True)
        elif key == 'r':
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
            print("\n[重置]", end="", flush=True)
        
        self.print_status()
    
    def publish_velocity(self):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z
        
        self.cmd_vel_pub.publish(twist)
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        # 恢复终端设置
        if os.isatty(sys.stdin.fileno()) and hasattr(self, 'old_attr'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        
        # 发送停止命令
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        print("\n程序退出，发送停止命令")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ImprovedKeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"\n程序出错: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
