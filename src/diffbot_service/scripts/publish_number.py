#!/usr/bin/env python3
"""
测试脚本：发布数字到/number话题
"""

import rclcpp
from rclcpp.node import Node
from std_msgs.msg import Float64
import argparse
import time

class NumberPublisher(Node):
    def __init__(self, number):
        super().__init__('number_publisher')
        self.publisher = self.create_publisher(Float64, '/number', 10)
        self.number = number
        
    def publish_number(self):
        msg = Float64()
        msg.data = self.number
        self.publisher.publish(msg)
        self.get_logger().info(f'发布数字: {self.number}')

def main():
    parser = argparse.ArgumentParser(description='发布数字到/number话题')
    parser.add_argument('--number', type=float, default=10.0, 
                       help='要发布的数字 (单位: 厘米)')
    parser.add_argument('--delay', type=float, default=2.0,
                       help='发布前等待时间 (秒)')
    
    args = parser.parse_args()
    
    rclcpp.init()
    
    publisher = NumberPublisher(args.number)
    
    print(f"等待 {args.delay} 秒后发布数字: {args.number} cm")
    time.sleep(args.delay)
    
    publisher.publish_number()
    
    # 保持节点运行一会儿确保消息发送
    rclcpp.spin_once(publisher, timeout_sec=1.0)
    
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
