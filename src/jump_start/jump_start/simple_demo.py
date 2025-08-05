#!/usr/bin/env python3
"""
简单的FSM使用示例
演示如何使用jump_start包进行机器人控制
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimpleDemo(Node):
    def __init__(self):
        super().__init__('simple_demo')
        
        # 发布任务命令
        self.command_pub = self.create_publisher(
            String,
            '/mission_command',
            10
        )
        
        # 订阅FSM状态
        self.state_sub = self.create_subscription(
            String,
            '/fsm_state',
            self.state_callback,
            10
        )
        
        self.current_state = "IDLE"
        self.get_logger().info('Simple Demo started')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f'FSM State: {self.current_state}')
    
    def send_command(self, command):
        """发送命令"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent: {command}')
    
    def run_demo(self):
        """运行演示"""
        self.get_logger().info('🚀 Starting FSM Demo')
        
        # 等待系统准备
        time.sleep(2.0)
        
        # 演示1: 导航到位置
        self.get_logger().info('📍 Demo 1: Navigation')
        self.send_command('navigate 1.0 1.0 0.3')
        time.sleep(10.0)
        
        # 演示2: 目标跟踪
        self.get_logger().info('🎯 Demo 2: Target tracking')
        self.send_command('track')
        time.sleep(15.0)
        
        # 停止
        self.get_logger().info('🛑 Stopping mission')
        self.send_command('stop')
        
        self.get_logger().info('✅ Demo completed')


def main(args=None):
    rclpy.init(args=args)
    demo = SimpleDemo()
    
    try:
        # 运行演示
        demo.run_demo()
        
        # 继续监听
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
