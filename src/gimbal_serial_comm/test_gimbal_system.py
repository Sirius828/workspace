#!/usr/bin/env python3
"""
完整的gimbal系统测试脚本
检查从trajectory_planner到serial_comm的完整数据流
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time

class GimbalSystemTester(Node):
    def __init__(self):
        super().__init__('gimbal_system_tester')
        
        # 订阅所有相关话题
        self.create_subscription(
            JointState,
            '/gimbal_joint_states',
            self.joint_states_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/gimbal_serial_preview',
            self.serial_preview_callback,
            10
        )
        
        # 发布目标位置
        self.target_pub = self.create_publisher(
            Point,
            '/target_position',
            10
        )
        
        # 计数器
        self.joint_states_count = 0
        self.serial_preview_count = 0
        
        # 创建定时器发送测试目标
        self.timer = self.create_timer(2.0, self.publish_test_targets)
        
        # 状态报告定时器
        self.status_timer = self.create_timer(5.0, self.report_status)
        
        self.test_targets = [
            (400, 300),   # 中心
            (200, 200),   # 左上
            (600, 200),   # 右上
            (600, 400),   # 右下
            (200, 400),   # 左下
        ]
        self.current_target_index = 0
        
        self.get_logger().info("Gimbal系统测试节点已启动")
        
    def joint_states_callback(self, msg):
        """接收关节状态数据"""
        self.joint_states_count += 1
        if self.joint_states_count <= 3:  # 只显示前几个消息
            yaw_deg = math.degrees(msg.position[0])
            pitch_deg = math.degrees(msg.position[1])
            self.get_logger().info(f"关节状态 #{self.joint_states_count}: Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°")
        
    def serial_preview_callback(self, msg):
        """接收串口预览数据"""
        self.serial_preview_count += 1
        if self.serial_preview_count <= 3:  # 只显示前几个消息
            self.get_logger().info(f"串口数据 #{self.serial_preview_count}: {msg.data}")
        
    def publish_test_targets(self):
        """发布测试目标位置"""
        if self.current_target_index < len(self.test_targets):
            target = self.test_targets[self.current_target_index]
            
            msg = Point()
            msg.x = float(target[0])
            msg.y = float(target[1])
            msg.z = 0.0
            
            self.target_pub.publish(msg)
            
            self.get_logger().info(f"发送目标位置 #{self.current_target_index + 1}: ({target[0]}, {target[1]})")
            
            self.current_target_index += 1
        else:
            self.get_logger().info("所有测试目标已发送完毕")
            
    def report_status(self):
        """报告系统状态"""
        self.get_logger().info(f"系统状态 - 关节状态接收: {self.joint_states_count}, 串口数据接收: {self.serial_preview_count}")
        
        if self.joint_states_count > 0 and self.serial_preview_count > 0:
            self.get_logger().info("✓ 系统正常工作：trajectory_planner -> serial_comm 数据流畅通")
        elif self.joint_states_count > 0:
            self.get_logger().warn("⚠ 只接收到关节状态，串口通信可能有问题")
        elif self.serial_preview_count > 0:
            self.get_logger().warn("⚠ 只接收到串口数据，关节状态可能有问题")
        else:
            self.get_logger().error("✗ 系统异常：未接收到任何数据")

def main(args=None):
    rclpy.init(args=args)
    
    node = GimbalSystemTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("测试结束")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
