#!/usr/bin/env python3
"""
测试云台和底盘命令发送
向ROS话题发送测试命令，测试统一串口管理器的命令转发功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time
import math

class CommandTestNode(Node):
    def __init__(self):
        super().__init__('command_test_node')
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_cmd_pub = self.create_publisher(Vector3, '/gimbal/angle_cmd', 10)
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_test_commands)  # 10Hz
        
        self.start_time = self.get_clock().now()
        self.get_logger().info('命令测试节点已启动')
    
    def publish_test_commands(self):
        """发布测试命令"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # 发布底盘速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2 * math.sin(elapsed * 0.5)  # 前后运动
        cmd_vel.linear.y = 0.1 * math.cos(elapsed * 0.3)  # 左右运动
        cmd_vel.angular.z = 0.1 * math.sin(elapsed * 0.2)  # 旋转
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 发布云台角度命令
        gimbal_cmd = Vector3()
        gimbal_cmd.x = 15.0 * math.sin(elapsed * 0.4)  # 偏航角 -15°到+15°
        gimbal_cmd.y = 10.0 * math.cos(elapsed * 0.6)  # 俯仰角 -10°到+10°
        gimbal_cmd.z = 0.0
        
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        # 记录发送的命令
        self.get_logger().info(f'底盘命令: vx={cmd_vel.linear.x:.3f}, vy={cmd_vel.linear.y:.3f}, omega={cmd_vel.angular.z:.3f}')
        self.get_logger().info(f'云台命令: yaw={gimbal_cmd.x:.1f}°, pitch={gimbal_cmd.y:.1f}°')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = CommandTestNode()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
