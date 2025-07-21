#!/usr/bin/env python3
"""
统一串口管理数据监控脚本
监控各个话题的数据流，用于验证系统功能
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import time

class DataMonitorNode(Node):
    def __init__(self):
        super().__init__('data_monitor_node')
        
        # 创建订阅器
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel/odom',
            self.odom_callback,
            10
        )
        
        self.gimbal_angle_sub = self.create_subscription(
            Vector3,
            '/gimbal/current_angle',
            self.gimbal_angle_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.gimbal_cmd_sub = self.create_subscription(
            Vector3,
            '/gimbal/angle_cmd',
            self.gimbal_cmd_callback,
            10
        )
        
        # 数据统计
        self.odom_count = 0
        self.gimbal_angle_count = 0
        self.cmd_vel_count = 0
        self.gimbal_cmd_count = 0
        
        self.last_odom_time = None
        self.last_gimbal_time = None
        
        # 创建状态报告定时器
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('数据监控节点已启动')
        self.get_logger().info('监控话题: /wheel/odom, /gimbal/current_angle, /cmd_vel, /gimbal/angle_cmd')
    
    def odom_callback(self, msg):
        """里程计数据回调"""
        self.odom_count += 1
        self.last_odom_time = self.get_clock().now()
        
        if self.odom_count % 50 == 0:  # 每50个消息打印一次
            self.get_logger().info(f'里程计数据: 位置({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}), 速度({msg.twist.twist.linear.x:.3f}, {msg.twist.twist.linear.y:.3f})')
    
    def gimbal_angle_callback(self, msg):
        """云台角度回调"""
        self.gimbal_angle_count += 1
        self.last_gimbal_time = self.get_clock().now()
        
        if self.gimbal_angle_count % 20 == 0:  # 每20个消息打印一次
            self.get_logger().info(f'云台当前角度: 偏航{msg.x:.1f}°, 俯仰{msg.y:.1f}°')
    
    def cmd_vel_callback(self, msg):
        """底盘速度命令回调"""
        self.cmd_vel_count += 1
        
        if self.cmd_vel_count % 10 == 0:  # 每10个消息打印一次
            self.get_logger().info(f'收到底盘命令: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, omega={msg.angular.z:.3f}')
    
    def gimbal_cmd_callback(self, msg):
        """云台角度命令回调"""
        self.gimbal_cmd_count += 1
        
        if self.gimbal_cmd_count % 10 == 0:  # 每10个消息打印一次
            self.get_logger().info(f'收到云台命令: yaw={msg.x:.1f}°, pitch={msg.y:.1f}°')
    
    def print_status(self):
        """打印状态报告"""
        current_time = self.get_clock().now()
        
        # 计算数据频率
        odom_status = "活跃" if (self.last_odom_time and 
                                (current_time - self.last_odom_time).nanoseconds / 1e9 < 1.0) else "无数据"
        
        gimbal_status = "活跃" if (self.last_gimbal_time and 
                                  (current_time - self.last_gimbal_time).nanoseconds / 1e9 < 1.0) else "无数据"
        
        self.get_logger().info('=== 数据监控状态报告 ===')
        self.get_logger().info(f'里程计数据: {self.odom_count} 条消息, 状态: {odom_status}')
        self.get_logger().info(f'云台角度: {self.gimbal_angle_count} 条消息, 状态: {gimbal_status}')
        self.get_logger().info(f'底盘命令: {self.cmd_vel_count} 条消息')
        self.get_logger().info(f'云台命令: {self.gimbal_cmd_count} 条消息')
        self.get_logger().info('========================')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor_node = DataMonitorNode()
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'monitor_node' in locals():
            monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
