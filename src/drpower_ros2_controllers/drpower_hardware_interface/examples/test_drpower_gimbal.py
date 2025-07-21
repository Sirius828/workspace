#!/usr/bin/env python3
"""
DrPower云台控制示例

演示如何控制二轴云台进行平滑运动
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time


class DrPowerGimbalController(Node):
    def __init__(self):
        super().__init__('drpower_gimbal_controller')
        
        # 创建云台action客户端
        self.gimbal_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/gimbal_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.joint_names = ['pan_joint', 'tilt_joint']
        self.get_logger().info("DrPower云台控制器已启动")
    
    def wait_for_server(self):
        """等待action服务器"""
        self.get_logger().info("等待云台服务器...")
        self.gimbal_action_client.wait_for_server()
        self.get_logger().info("云台服务器已就绪")
    
    def send_gimbal_command(self, pan_angle, tilt_angle, duration=2.0):
        """发送云台控制命令"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [pan_angle, tilt_angle]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"云台移动到: Pan={math.degrees(pan_angle):.1f}°, Tilt={math.degrees(tilt_angle):.1f}°")
        return self.gimbal_action_client.send_goal_async(goal_msg)
    
    def scan_pattern(self):
        """执行扫描模式"""
        self.get_logger().info("=== 开始扫描模式 ===")
        
        # 定义扫描点
        scan_points = [
            (0, 0),                    # 中心
            (math.pi/4, math.pi/6),    # 右上
            (-math.pi/4, math.pi/6),   # 左上
            (-math.pi/4, -math.pi/6),  # 左下
            (math.pi/4, -math.pi/6),   # 右下
            (0, 0),                    # 回中心
        ]
        
        for pan, tilt in scan_points:
            future = self.send_gimbal_command(pan, tilt, 2.0)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.5)
    
    def smooth_circle(self):
        """平滑圆形运动"""
        self.get_logger().info("=== 开始圆形运动 ===")
        
        # 生成圆形轨迹
        waypoints = []
        times = []
        
        radius_pan = math.pi/3   # 水平半径
        radius_tilt = math.pi/6  # 垂直半径
        
        for i in range(13):  # 完整一圈
            angle = i * 2 * math.pi / 12
            pan = radius_pan * math.cos(angle)
            tilt = radius_tilt * math.sin(angle)
            
            waypoints.append([pan, tilt])
            times.append((i + 1) * 0.8)  # 每个点0.8秒
        
        # 发送轨迹
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        for positions, duration in zip(waypoints, times):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            goal_msg.trajectory.points.append(point)
        
        future = self.gimbal_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
    
    def tracking_simulation(self):
        """模拟目标跟踪"""
        self.get_logger().info("=== 模拟目标跟踪 ===")
        
        # 模拟目标在"8"字形运动
        for i in range(25):
            t = i * 0.4  # 时间参数
            
            # 8字形轨迹
            pan = math.pi/4 * math.sin(t)
            tilt = math.pi/8 * math.sin(2*t)
            
            future = self.send_gimbal_command(pan, tilt, 0.4)
            rclpy.spin_until_future_complete(self, future)
    
    def run_demo(self):
        """运行演示"""
        self.wait_for_server()
        
        # 回中心位置
        future = self.send_gimbal_command(0, 0, 2.0)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1)
        
        # 执行各种运动模式
        self.scan_pattern()
        time.sleep(1)
        
        self.smooth_circle()
        time.sleep(1)
        
        self.tracking_simulation()
        time.sleep(1)
        
        # 回中心结束
        future = self.send_gimbal_command(0, 0, 2.0)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info("云台演示完成！")


def main():
    rclpy.init()
    
    try:
        controller = DrPowerGimbalController()
        controller.run_demo()
    except KeyboardInterrupt:
        print("演示被中断")
    except Exception as e:
        print(f"演示过程中出现错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
