#!/usr/bin/env python3

"""
Chassis Simulation Node

订阅 /cmd_vel 命令，积分计算机器人位置，发布 /chassis/odom 和 TF变换。
用于仿真模式下提供odom数据，让RViz可视化机器人运动。

功能：
- 订阅 /cmd_vel (geometry_msgs/Twist)
- 发布 /chassis/odom (nav_msgs/Odometry) 
- 发布 TF变换: odom -> base_link
- 支持三轮全向移动底盘运动学
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import tf_transformations

class ChassisSimNode(Node):
    def __init__(self):
        super().__init__('chassis_sim_node')
        
        # 机器人状态变量
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vw = 0.0
        
        self.last_time = self.get_clock().now()
        
        # 发布器
        self.odom_publisher = self.create_publisher(Odometry, '/chassis/odom', 10)
        
        # 订阅器
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时器：50Hz发布频率
        self.timer = self.create_timer(0.02, self.update_and_publish)
        
        self.get_logger().info('底盘仿真节点已启动 - 发布 /chassis/odom 和 TF变换')
        self.get_logger().info('订阅 /cmd_vel 进行运动仿真')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_vw = msg.angular.z
        
        self.get_logger().debug(f'收到速度命令: vx={self.current_vx:.3f}, vy={self.current_vy:.3f}, vw={self.current_vw:.3f}')
    
    def update_and_publish(self):
        """更新机器人位姿并发布里程计和TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # 三轮全向底盘运动学：机器人可以任意方向移动
        # 将机体坐标系速度转换到世界坐标系
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        world_vx = cos_yaw * self.current_vx - sin_yaw * self.current_vy
        world_vy = sin_yaw * self.current_vx + cos_yaw * self.current_vy
        world_vw = self.current_vw
        
        # 积分更新位置和角度
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw += world_vw * dt
        
        # 角度归一化
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # 创建并发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # 位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 姿态 (yaw角转四元数)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # 速度 (机体坐标系)
        odom_msg.twist.twist.linear.x = self.current_vx
        odom_msg.twist.twist.linear.y = self.current_vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_vw
        
        # 协方差矩阵 (简单设置)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36
        
        # 对角线元素设置小的不确定性
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        odom_msg.twist.covariance[0] = 0.1  # vx  
        odom_msg.twist.covariance[7] = 0.1  # vy
        odom_msg.twist.covariance[35] = 0.1 # vw
        
        # 发布里程计
        self.odom_publisher.publish(odom_msg)
        
        # 发布TF变换: odom -> base_link
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
        # 调试信息
        if self.current_vx != 0 or self.current_vy != 0 or self.current_vw != 0:
            self.get_logger().debug(
                f'位置: x={self.x:.3f}m, y={self.y:.3f}m, yaw={math.degrees(self.yaw):.1f}°'
            )
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    node = ChassisSimNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('底盘仿真节点停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
