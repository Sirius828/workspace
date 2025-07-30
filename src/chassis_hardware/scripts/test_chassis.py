#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class ChassisTestNode(Node):
    def __init__(self):
        super().__init__('chassis_test_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_gimbal_pub = self.create_publisher(Twist, '/cmd_gimbal', 10)
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/chassis/odom', self.odom_callback, 10)
        
        self.get_logger().info('Chassis test node started')
        
        # Test timer
        self.timer = self.create_timer(2.0, self.test_commands)
        self.test_counter = 0
        
    def odom_callback(self, msg):
        self.get_logger().info(
            f'Received odometry - Position: ({msg.pose.pose.position.x:.3f}, '
            f'{msg.pose.pose.position.y:.3f}), Velocity: '
            f'({msg.twist.twist.linear.x:.3f}, {msg.twist.twist.linear.y:.3f})'
        )
    
    def test_commands(self):
        # Test different movement patterns
        cmd_vel = Twist()
        cmd_gimbal = Twist()
        
        if self.test_counter == 0:
            # Move forward
            cmd_vel.linear.x = 0.5
            cmd_gimbal.angular.x = 0.0  # yaw
            cmd_gimbal.angular.y = 0.0  # pitch
            self.get_logger().info('Test: Moving forward')
            
        elif self.test_counter == 1:
            # Move sideways
            cmd_vel.linear.y = 0.3
            cmd_gimbal.angular.x = 0.5  # yaw
            self.get_logger().info('Test: Moving sideways with gimbal yaw')
            
        elif self.test_counter == 2:
            # Rotate
            cmd_vel.angular.z = 0.5
            cmd_gimbal.angular.y = 0.3  # pitch
            self.get_logger().info('Test: Rotating with gimbal pitch')
            
        else:
            # Stop
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            cmd_gimbal.angular.x = 0.0
            cmd_gimbal.angular.y = 0.0
            self.get_logger().info('Test: Stopping')
            self.test_counter = -1
        
        self.cmd_vel_pub.publish(cmd_vel)
        self.cmd_gimbal_pub.publish(cmd_gimbal)
        self.test_counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = ChassisTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
