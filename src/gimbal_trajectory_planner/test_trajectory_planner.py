#!/usr/bin/env python3
"""
测试trajectory_planner节点的TF和消息兼容性
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_planner_test')
        
        # 创建发布者
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        self.corner_pub = self.create_publisher(Point32, '/corner_points', 10)
        self.midline_pub = self.create_publisher(Point32, '/midline_points', 10)
        
        # 创建定时器
        self.timer = self.create_timer(2.0, self.test_callback)
        self.test_count = 0
        
        self.get_logger().info('Test publisher node started')
        
    def test_callback(self):
        self.test_count += 1
        
        if self.test_count == 1:
            # 测试单点目标位置
            self.get_logger().info('Testing single target position (RESET)')
            target = Point()
            target.x = 320.0  # 图像中心
            target.y = 240.0
            target.z = 0.0
            self.target_pub.publish(target)
            
        elif self.test_count == 2:
            # 测试SQUARE_LOOP - 发布5个角点
            self.get_logger().info('Testing SQUARE_LOOP (5 corner points)')
            corners = [
                (100.0, 100.0),  # 左上
                (500.0, 100.0),  # 右上  
                (500.0, 400.0),  # 右下
                (100.0, 400.0),  # 左下
                (100.0, 100.0)   # 回到起点
            ]
            
            for i, (x, y) in enumerate(corners):
                corner = Point32()
                corner.x = x
                corner.y = y
                corner.z = 0.0
                self.corner_pub.publish(corner)
                self.get_logger().info(f'Published corner {i+1}: ({x}, {y})')
                time.sleep(0.1)  # 短暂延迟
                
        elif self.test_count == 3:
            # 测试A4_LOOP - 发布5个中线点
            self.get_logger().info('Testing A4_LOOP (5 midline points)')
            midlines = [
                (200.0, 150.0),  # 中线点1
                (400.0, 150.0),  # 中线点2
                (400.0, 350.0),  # 中线点3
                (200.0, 350.0),  # 中线点4
                (200.0, 150.0)   # 回到起点
            ]
            
            for i, (x, y) in enumerate(midlines):
                midline = Point32()
                midline.x = x
                midline.y = y
                midline.z = 0.0
                self.midline_pub.publish(midline)
                self.get_logger().info(f'Published midline {i+1}: ({x}, {y})')
                time.sleep(0.1)  # 短暂延迟
                
        elif self.test_count >= 4:
            self.get_logger().info('All tests completed')
            self.timer.cancel()

def main():
    rclpy.init()
    test_node = TestPublisher()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
