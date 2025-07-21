#!/usr/bin/env python3
"""
键盘遥控节点
通过键盘控制机器人运动，支持步进控制和速度叠加
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # 声明参数
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('step_size', 0.05)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('auto_stop_timeout', 1.0)  # 秒，无按键时自动停止
        
        # 获取参数
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.auto_stop_timeout = self.get_parameter('auto_stop_timeout').get_parameter_value().double_value
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # 当前速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 线程控制
        self.running = True
        self.last_key_time = time.time()
        
        # 创建定时器定期发布速度
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('键盘遥控节点已启动')
        self.get_logger().info(f'发布话题: {self.cmd_vel_topic}')
        self.get_logger().info(f'步进大小: {self.step_size}')
        self.get_logger().info(f'最大线速度: {self.max_linear_vel} m/s')
        self.get_logger().info(f'最大角速度: {self.max_angular_vel} rad/s')
        self.print_instructions()
    
    def print_instructions(self):
        """打印操作说明"""
        print("\n" + "="*60)
        print("键盘遥控说明:")
        print("="*60)
        print("移动控制:")
        print("  w/s: 前进/后退  (x方向)")
        print("  a/d: 左转/右转  (角速度)")
        print("  q/e: 左移/右移  (y方向)")
        print("")
        print("控制功能:")
        print("  空格: 立即停止")
        print("  r: 重置所有速度为0")
        print("  +/-: 增加/减少步进大小")
        print("  Ctrl+C: 退出程序")
        print("")
        print("特性:")
        print(f"  - 每次按键增加/减少 {self.step_size} 的速度")
        print("  - 反复按同一方向键可以叠加速度")
        print(f"  - 最大线速度: ±{self.max_linear_vel} m/s")
        print(f"  - 最大角速度: ±{self.max_angular_vel} rad/s")
        print(f"  - {self.auto_stop_timeout}秒无按键自动停止")
        print("="*60)
        print()
        self.print_current_status()
    
    def print_current_status(self):
        """打印当前状态"""
        print(f"\r当前速度: vx={self.current_linear_x:+.3f} vy={self.current_linear_y:+.3f} "
              f"omega={self.current_angular_z:+.3f} | 步进:{self.step_size:.3f}", end="", flush=True)
    
    def get_key(self):
        """获取键盘输入"""
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
        try:
            while self.running and rclpy.ok():
                key = self.get_key()
                self.last_key_time = time.time()
                
                if key == '\x03':  # Ctrl+C
                    self.get_logger().info('\n收到退出信号')
                    self.running = False
                    break
                
                self.process_key(key)
                self.print_current_status()
                
        except Exception as e:
            self.get_logger().error(f'键盘监听出错: {e}')
            self.running = False
    
    def process_key(self, key):
        """处理按键"""
        if key == 'w':
            # 前进
            self.current_linear_x = min(self.current_linear_x + self.step_size, self.max_linear_vel)
        elif key == 's':
            # 后退
            self.current_linear_x = max(self.current_linear_x - self.step_size, -self.max_linear_vel)
        elif key == 'a':
            # 左转
            self.current_angular_z = min(self.current_angular_z + self.step_size, self.max_angular_vel)
        elif key == 'd':
            # 右转
            self.current_angular_z = max(self.current_angular_z - self.step_size, -self.max_angular_vel)
        elif key == 'q':
            # 左移
            self.current_linear_y = min(self.current_linear_y + self.step_size, self.max_linear_vel)
        elif key == 'e':
            # 右移
            self.current_linear_y = max(self.current_linear_y - self.step_size, -self.max_linear_vel)
        elif key == ' ':
            # 立即停止
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
            print("\n立即停止!")
        elif key == 'r':
            # 重置
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
            print("\n速度已重置!")
        elif key == '+' or key == '=':
            # 增加步进
            self.step_size = min(self.step_size + 0.01, 0.5)
            print(f"\n步进增加到: {self.step_size:.3f}")
        elif key == '-':
            # 减少步进
            self.step_size = max(self.step_size - 0.01, 0.01)
            print(f"\n步进减少到: {self.step_size:.3f}")
        elif key == 'h':
            # 帮助
            print("\n")
            self.print_instructions()
    
    def publish_velocity(self):
        """定期发布速度"""
        # 检查是否超时
        if time.time() - self.last_key_time > self.auto_stop_timeout:
            if (self.current_linear_x != 0.0 or 
                self.current_linear_y != 0.0 or 
                self.current_angular_z != 0.0):
                
                self.current_linear_x = 0.0
                self.current_linear_y = 0.0
                self.current_angular_z = 0.0
                print("\n超时自动停止!")
                self.print_current_status()
        
        # 创建并发布Twist消息
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
        self.running = False
        
        # 发送停止命令
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        print("\n发送停止命令")
        self.get_logger().info('键盘遥控节点已停止')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = KeyboardTeleopNode()
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if 'teleop_node' in locals():
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
