#!/usr/bin/env python3
"""
测试串口通信节点完整性
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time

class SerialCommTester(Node):
    def __init__(self):
        super().__init__('serial_comm_tester')
        
        # 订阅串口预览话题
        self.create_subscription(
            String,
            '/gimbal_serial_preview',
            self.serial_preview_callback,
            10
        )
        
        # 发布测试关节状态
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/gimbal_joint_states',
            10
        )
        
        # 创建定时器发送测试数据
        self.timer = self.create_timer(0.1, self.publish_test_data)
        
        self.test_time = 0.0
        self.received_count = 0
        
        self.get_logger().info("串口通信测试节点已启动")
        
    def serial_preview_callback(self, msg):
        """接收串口预览数据"""
        self.received_count += 1
        self.get_logger().info(f"接收到串口数据 #{self.received_count}: {msg.data}")
        
    def publish_test_data(self):
        """发布测试关节状态数据"""
        # 生成正弦波测试数据
        self.test_time += 0.1
        yaw_angle = math.sin(self.test_time) * 30.0  # ±30度
        pitch_angle = math.cos(self.test_time) * 20.0  # ±20度
        
        # 创建JointState消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal_base_link"
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [math.radians(yaw_angle), math.radians(pitch_angle)]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        
        # 发布消息
        self.joint_states_pub.publish(msg)
        
        # 每秒记录一次状态
        if int(self.test_time * 10) % 10 == 0:
            self.get_logger().info(f"发送测试数据: Yaw={yaw_angle:.2f}°, Pitch={pitch_angle:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    
    node = SerialCommTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("测试结束")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
