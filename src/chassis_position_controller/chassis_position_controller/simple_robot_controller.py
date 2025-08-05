#!/usr/bin/env python3

"""
简化的机器人控制库

这个库提供了一个简化的接口来控制机器人，
适用于快速脚本开发和自动化任务。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
import math
import time
import threading
from typing import Optional, Tuple, List

class SimpleRobotController:
    """
    简化的机器人控制器
    
    使用方法:
    1. robot = SimpleRobotController()
    2. robot.start()
    3. robot.move_to(x, y)
    4. robot.stop()
    """
    
    def __init__(self):
        self._node = None
        self._spin_thread = None
        self._running = False
        
    def start(self) -> bool:
        """启动控制器"""
        try:
            rclpy.init()
            self._node = RobotControllerNode()
            
            # 启动后台线程
            self._spin_thread = threading.Thread(
                target=lambda: rclpy.spin(self._node), 
                daemon=True
            )
            self._spin_thread.start()
            self._running = True
            
            # 等待初始化
            time.sleep(2)
            print("🤖 Robot controller started")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start controller: {e}")
            return False
    
    def stop(self):
        """停止控制器"""
        if self._running:
            self._running = False
            if self._node:
                self._node.destroy_node()
            rclpy.shutdown()
            print("👋 Robot controller stopped")
    
    def get_position(self) -> Tuple[float, float, float]:
        """获取当前位置 (x, y, yaw_degrees)"""
        if not self._running or not self._node:
            return (0.0, 0.0, 0.0)
        return self._node.get_current_position()
    
    def move_to(self, x: float, y: float, yaw_deg: float = None, 
                speed: float = None, timeout: float = 30.0) -> bool:
        """移动到指定位置"""
        if not self._running or not self._node:
            print("❌ Controller not started")
            return False
        return self._node.move_to_position(x, y, yaw_deg, speed, timeout)
    
    def move_forward(self, distance: float, speed: float = None) -> bool:
        """向前移动指定距离"""
        if not self._running or not self._node:
            print("❌ Controller not started")
            return False
        return self._node.move_forward(distance, speed)
    
    def turn_to(self, yaw_deg: float, angular_speed: float = None) -> bool:
        """转向指定角度"""
        if not self._running or not self._node:
            print("❌ Controller not started")
            return False
        return self._node.turn_to_angle(yaw_deg, angular_speed)
    
    def set_speed(self, speed: float):
        """设置速度限制"""
        if self._running and self._node:
            self._node.set_speed_limit(speed)
    
    def execute_path(self, waypoints: List[Tuple[float, float, float]], 
                    speed: float = None) -> bool:
        """执行路径"""
        if not self._running or not self._node:
            print("❌ Controller not started")
            return False
        return self._node.execute_path(waypoints, speed)

class RobotControllerNode(Node):
    """内部机器人控制节点"""
    
    def __init__(self):
        super().__init__('simple_robot_controller')
        
        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.target_speed_pub = self.create_publisher(TwistStamped, '/target_pose_with_speed', 10)
        self.speed_override_pub = self.create_publisher(Float32, '/target_speed_override', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/chassis/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(
            Bool, '/position_controller/target_reached', self.status_callback, 10)
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_reached = False
        
    def odom_callback(self, msg):
        """更新当前位置"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def status_callback(self, msg):
        """处理目标到达状态"""
        self.target_reached = msg.data
    
    def get_current_position(self) -> Tuple[float, float, float]:
        """获取当前位置"""
        return (self.current_x, self.current_y, math.degrees(self.current_yaw))
    
    def move_to_position(self, x: float, y: float, yaw_deg: float = None, 
                        speed: float = None, timeout: float = 30.0) -> bool:
        """移动到指定位置"""
        if yaw_deg is None:
            yaw_deg = math.degrees(self.current_yaw)
        
        print(f"🎯 Moving to: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)")
        if speed:
            print(f"⚡ Speed: {speed:.2f} m/s")
        
        if speed is not None:
            self._send_speed_target(x, y, yaw_deg, speed)
        else:
            self._send_basic_target(x, y, yaw_deg)
        
        return self._wait_for_target(timeout)
    
    def move_forward(self, distance: float, speed: float = None, timeout: float = 30.0) -> bool:
        """向前移动指定距离"""
        target_x = self.current_x + distance * math.cos(self.current_yaw)
        target_y = self.current_y + distance * math.sin(self.current_yaw)
        yaw_deg = math.degrees(self.current_yaw)
        
        print(f"➡️ Moving forward: {distance:.2f}m")
        return self.move_to_position(target_x, target_y, yaw_deg, speed, timeout)
    
    def turn_to_angle(self, yaw_deg: float, angular_speed: float = None, timeout: float = 15.0) -> bool:
        """转向指定角度"""
        print(f"🔄 Turning to: {yaw_deg:.1f}°")
        
        if angular_speed is not None:
            self._send_speed_target(self.current_x, self.current_y, yaw_deg, None, angular_speed)
        else:
            self._send_basic_target(self.current_x, self.current_y, yaw_deg)
        
        return self._wait_for_target(timeout)
    
    def set_speed_limit(self, speed: float):
        """设置全局速度限制"""
        msg = Float32()
        msg.data = speed
        self.speed_override_pub.publish(msg)
        print(f"⚡ Speed limit set: {speed:.2f} m/s")
    
    def execute_path(self, waypoints: list, speed: float = None) -> bool:
        """执行路径点序列"""
        print(f"🛤️ Executing path with {len(waypoints)} waypoints")
        
        for i, (x, y, yaw_deg) in enumerate(waypoints, 1):
            print(f"📍 Waypoint {i}/{len(waypoints)}: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)")
            
            if not self.move_to_position(x, y, yaw_deg, speed):
                print(f"❌ Failed to reach waypoint {i}")
                return False
                
            print(f"✅ Waypoint {i} reached")
            time.sleep(0.5)
        
        print("🎉 Path execution completed!")
        return True
    
    def _send_basic_target(self, x: float, y: float, yaw_deg: float):
        """发送基础目标位置"""
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        
        self.target_pose_pub.publish(msg)
        self.target_reached = False
    
    def _send_speed_target(self, x: float, y: float, yaw_deg: float, 
                          linear_speed: float = None, angular_speed: float = None):
        """发送带速度信息的目标"""
        msg = TwistStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = linear_speed if linear_speed else 0.0
        
        yaw_rad = math.radians(yaw_deg)
        msg.twist.angular.z = yaw_rad
        msg.twist.angular.x = angular_speed if angular_speed else 0.0
        msg.twist.angular.y = 0.0
        
        self.target_speed_pub.publish(msg)
        self.target_reached = False
    
    def _wait_for_target(self, timeout: float) -> bool:
        """等待到达目标"""
        start_time = time.time()
        
        while not self.target_reached and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.target_reached

# 使用示例
if __name__ == '__main__':
    # 创建控制器
    robot = SimpleRobotController()
    
    try:
        # 启动控制器
        if robot.start():
            print(f"📍 当前位置: {robot.get_position()}")
            
            # 移动到指定位置
            robot.move_to(1.0, 0.0)
            
            # 向前移动
            robot.move_forward(0.5)
            
            # 转向
            robot.turn_to(90)
            
            # 执行路径
            path = [(2.0, 0.0, 0), (2.0, 1.0, 90), (1.0, 1.0, 180)]
            robot.execute_path(path, speed=0.3)
            
            print(f"📍 最终位置: {robot.get_position()}")
        
    except KeyboardInterrupt:
        print("\n👋 用户中断")
    finally:
        robot.stop()
