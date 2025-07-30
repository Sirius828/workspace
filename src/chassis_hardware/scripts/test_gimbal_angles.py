#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class GimbalAngleTestNode(Node):
    def __init__(self):
        super().__init__('gimbal_angle_test_node')
        
        # Publisher for gimbal commands
        self.cmd_gimbal_pub = self.create_publisher(Twist, '/cmd_gimbal', 10)
        
        self.get_logger().info('Gimbal angle test node started - testing radian to degree conversion')
        
        # Test timer
        self.timer = self.create_timer(3.0, self.test_gimbal_angles)
        self.test_counter = 0
        
    def test_gimbal_angles(self):
        cmd_gimbal = Twist()
        
        if self.test_counter == 0:
            # Test 45 degrees in radians
            cmd_gimbal.angular.x = math.pi / 4    # 45 degrees yaw
            cmd_gimbal.angular.y = math.pi / 6    # 30 degrees pitch
            self.get_logger().info(f'Test 1: Sending yaw={math.pi/4:.4f} rad (45°), pitch={math.pi/6:.4f} rad (30°)')
            
        elif self.test_counter == 1:
            # Test 90 degrees in radians
            cmd_gimbal.angular.x = math.pi / 2    # 90 degrees yaw
            cmd_gimbal.angular.y = -math.pi / 4   # -45 degrees pitch
            self.get_logger().info(f'Test 2: Sending yaw={math.pi/2:.4f} rad (90°), pitch={-math.pi/4:.4f} rad (-45°)')
            
        elif self.test_counter == 2:
            # Test 180 degrees in radians
            cmd_gimbal.angular.x = math.pi        # 180 degrees yaw
            cmd_gimbal.angular.y = math.pi / 3    # 60 degrees pitch
            self.get_logger().info(f'Test 3: Sending yaw={math.pi:.4f} rad (180°), pitch={math.pi/3:.4f} rad (60°)')
            
        else:
            # Reset to zero
            cmd_gimbal.angular.x = 0.0
            cmd_gimbal.angular.y = 0.0
            self.get_logger().info('Test 4: Sending yaw=0.0 rad (0°), pitch=0.0 rad (0°)')
            self.test_counter = -1
        
        self.cmd_gimbal_pub.publish(cmd_gimbal)
        self.test_counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = GimbalAngleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
