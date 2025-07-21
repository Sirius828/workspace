#!/usr/bin/env python3
"""
发送一系列路径点给简单导航器
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations
import math
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # 发布目标位置
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 订阅里程计以监控位置
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', 
            self.odom_callback, 10)
        
        self.current_pose = None
        self.current_waypoint = 0
        self.waypoints = []
        self.position_tolerance = 0.15  # 到达容差
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def add_waypoint(self, x, y, yaw=0.0):
        """添加路径点"""
        self.waypoints.append((x, y, yaw))
        
    def send_goal(self, x, y, yaw):
        """发送单个目标位置"""
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'发送路径点 {self.current_waypoint + 1}: ({x:.2f}, {y:.2f}) @ {math.degrees(yaw):.1f}°')
        
    def distance_to_goal(self, goal_x, goal_y):
        """计算到目标的距离"""
        if self.current_pose is None:
            return float('inf')
            
        dx = goal_x - self.current_pose.position.x
        dy = goal_y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
        
    def navigate_waypoints(self):
        """按顺序导航所有路径点"""
        if not self.waypoints:
            self.get_logger().warn("没有设置路径点！")
            return
            
        self.get_logger().info(f"开始导航 {len(self.waypoints)} 个路径点...")
        
        # 等待里程计数据
        while self.current_pose is None:
            self.get_logger().info("等待里程计数据...")
            rclpy.spin_once(self, timeout_sec=1.0)
            
        # 发送第一个路径点
        self.current_waypoint = 0
        x, y, yaw = self.waypoints[self.current_waypoint]
        self.send_goal(x, y, yaw)
        
        # 监控导航过程
        while self.current_waypoint < len(self.waypoints):
            rclpy.spin_once(self, timeout_sec=0.1)
            
            x, y, yaw = self.waypoints[self.current_waypoint]
            distance = self.distance_to_goal(x, y)
            
            # 检查是否到达当前路径点
            if distance < self.position_tolerance:
                self.get_logger().info(f"✅ 到达路径点 {self.current_waypoint + 1}")
                self.current_waypoint += 1
                
                # 发送下一个路径点
                if self.current_waypoint < len(self.waypoints):
                    time.sleep(1.0)  # 短暂停留
                    x, y, yaw = self.waypoints[self.current_waypoint]
                    self.send_goal(x, y, yaw)
                    
            time.sleep(0.1)
            
        self.get_logger().info("🎉 所有路径点导航完成！")

def main():
    rclpy.init()
    navigator = WaypointNavigator()
    
    try:
        # 定义一些示例路径点 (正方形路径)
        navigator.add_waypoint(1.0, 0.0, 0.0)      # 前进
        navigator.add_waypoint(1.0, 1.0, math.pi/2)  # 右转
        navigator.add_waypoint(0.0, 1.0, math.pi)    # 后退
        navigator.add_waypoint(0.0, 0.0, -math.pi/2) # 左转回到起点
        
        # 开始导航
        navigator.navigate_waypoints()
        
    except KeyboardInterrupt:
        navigator.get_logger().info("导航被用户中断")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
