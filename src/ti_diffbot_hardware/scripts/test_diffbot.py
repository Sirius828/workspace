#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestDiffBot(Node):
    def __init__(self):
        super().__init__('test_diffbot')
        
        # 创建速度命令发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        
        # 创建定时器，每0.1秒发布一次命令
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        
        # 测试序列
        self.test_sequence = [
            (1.0, 0.0, 3.0),    # 直行 1m/s，3秒
            (0.0, 1.0, 3.0),    # 左转 1rad/s，3秒
            (-1.0, 0.0, 3.0),   # 后退 1m/s，3秒
            (0.0, -1.0, 3.0),   # 右转 1rad/s，3秒
            (0.0, 0.0, 1.0),    # 停止 1秒
        ]
        
        self.current_test = 0
        self.test_start_time = time.time()
        
        self.get_logger().info('TiDiffBot 测试节点启动')
        self.get_logger().info('开始测试序列...')
    
    def publish_cmd_vel(self):
        if self.current_test >= len(self.test_sequence):
            self.get_logger().info('测试完成，停止机器人')
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # 获取当前测试参数
        linear_x, angular_z, duration = self.test_sequence[self.current_test]
        
        # 检查是否到达测试时间
        if time.time() - self.test_start_time > duration:
            self.current_test += 1
            self.test_start_time = time.time()
            
            if self.current_test < len(self.test_sequence):
                linear_x, angular_z, duration = self.test_sequence[self.current_test]
                self.get_logger().info(f'测试 {self.current_test + 1}: 线速度={linear_x:.1f}, 角速度={angular_z:.1f}, 持续={duration:.1f}秒')
            return
        
        # 发布速度命令
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestDiffBot()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        twist = Twist()
        test_node.cmd_vel_pub.publish(twist)
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
