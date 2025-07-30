#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math
import time

class GimbalTestNode(Node):
    def __init__(self):
        super().__init__('gimbal_test_node')
        
        # 订阅云台控制指令，用于监控
        self.gimbal_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_gimbal',
            self.gimbal_cmd_callback,
            10
        )
        
        # 发布模拟目标位置，用于测试
        self.target_pub = self.create_publisher(
            Point,
            '/target_position_pixel',
            10
        )
        
        # 创建定时器来发布测试目标
        self.test_timer = self.create_timer(0.1, self.publish_test_target)
        
        # 测试参数
        self.test_mode = "circle"  # "center", "circle", "square", "manual"
        self.test_time = 0.0
        self.center_x = 320.0
        self.center_y = 240.0
        
        self.get_logger().info(f'Gimbal test node started in {self.test_mode} mode')
    
    def gimbal_cmd_callback(self, msg):
        """监控云台控制指令"""
        yaw_deg = math.degrees(msg.angular.x)
        pitch_deg = math.degrees(msg.angular.y)
        
        self.get_logger().info(
            f'Gimbal command: yaw={yaw_deg:.2f}°, pitch={pitch_deg:.2f}°'
        )
    
    def publish_test_target(self):
        """发布测试目标位置"""
        self.test_time += 0.1
        
        target_msg = Point()
        target_msg.z = 0.9  # 高置信度
        
        if self.test_mode == "center":
            # 测试中心位置
            target_msg.x = self.center_x
            target_msg.y = self.center_y
            
        elif self.test_mode == "circle":
            # 圆形运动
            radius = 100.0
            angle = self.test_time * 0.5  # 慢速圆形运动
            target_msg.x = self.center_x + radius * math.cos(angle)
            target_msg.y = self.center_y + radius * math.sin(angle)
            
        elif self.test_mode == "square":
            # 方形运动
            cycle_time = 8.0  # 8秒一个周期
            t = (self.test_time % cycle_time) / cycle_time  # 0-1
            
            if t < 0.25:
                # 右边
                target_msg.x = self.center_x + 100
                target_msg.y = self.center_y + (t * 4 - 0.5) * 200
            elif t < 0.5:
                # 上边
                target_msg.x = self.center_x + (0.75 - t * 4) * 200
                target_msg.y = self.center_y + 100
            elif t < 0.75:
                # 左边
                target_msg.x = self.center_x - 100
                target_msg.y = self.center_y + (1.25 - t * 4) * 200
            else:
                # 下边
                target_msg.x = self.center_x + (t * 4 - 1.75) * 200
                target_msg.y = self.center_y - 100
                
        # 确保目标在图像范围内
        target_msg.x = max(0, min(640, target_msg.x))
        target_msg.y = max(0, min(480, target_msg.y))
        
        self.target_pub.publish(target_msg)
    
    def set_test_mode(self, mode):
        """设置测试模式"""
        self.test_mode = mode
        self.test_time = 0.0
        self.get_logger().info(f'Test mode changed to: {mode}')

def main(args=None):
    rclpy.init(args=args)
    node = GimbalTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
