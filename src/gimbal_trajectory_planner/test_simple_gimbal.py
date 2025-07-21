#!/usr/bin/env python3
"""
简化版云台节点测试脚本
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import JointState
import math
import time

class SimpleGimbalTester(Node):
    def __init__(self):
        super().__init__('simple_gimbal_tester')
        
        # 发布者
        self.point_pub = self.create_publisher(Point, '/target_position', 10)
        self.corner_pub = self.create_publisher(Point32, '/corner_points', 10)
        
        # 订阅者
        self.joint_sub = self.create_subscription(
            JointState, 
            '/gimbal_joint_states', 
            self.joint_callback, 
            10
        )
        
        self.test_mode = 'single_point'  # 'single_point' 或 'corners'
        self.test_time = 0.0
        self.received_count = 0
        
        # 定时器
        self.test_timer = self.create_timer(0.1, self.run_test)
        
        self.get_logger().info("简化版云台节点测试器启动")
        self.get_logger().info("测试模式: 单点目标位置")
        
    def joint_callback(self, msg):
        """接收关节状态回调"""
        if len(msg.position) >= 2:
            yaw_deg = math.degrees(msg.position[0])
            pitch_deg = math.degrees(msg.position[1])
            self.received_count += 1
            
            if self.received_count % 10 == 0:  # 每10条消息记录一次
                self.get_logger().info(f"接收到关节状态 #{self.received_count}: "
                                     f"Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°")
    
    def run_test(self):
        """运行测试"""
        self.test_time += 0.1
        
        if self.test_mode == 'single_point':
            self.test_single_point()
        elif self.test_mode == 'corners':
            self.test_corners()
            
    def test_single_point(self):
        """测试单点模式"""
        # 生成圆形轨迹测试数据
        radius = 200  # 像素
        center_x, center_y = 640, 360  # 图像中心
        
        angle = self.test_time * 0.5  # 慢速旋转
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        # 发布目标点
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        
        self.point_pub.publish(point)
        
        # 每5秒记录一次状态
        if int(self.test_time * 10) % 50 == 0:
            self.get_logger().info(f"发送目标位置: ({x:.1f}, {y:.1f})")
            
    def test_corners(self):
        """测试角点模式"""
        # 定义矩形的四个角点 (像素坐标)
        corners = [
            (440, 260),  # 左上
            (840, 260),  # 右上
            (840, 460),  # 右下
            (440, 460),  # 左下
        ]
        
        # 每3秒发送一个角点
        corner_interval = 3.0
        corner_index = int(self.test_time / corner_interval) % len(corners)
        
        if self.test_time % corner_interval < 0.1:  # 只在时间间隔开始时发送
            x, y = corners[corner_index]
            
            point = Point32()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            
            self.corner_pub.publish(point)
            self.get_logger().info(f"发送角点 {corner_index + 1}/4: ({x}, {y})")
            
    def switch_test_mode(self):
        """切换测试模式"""
        if self.test_mode == 'single_point':
            self.test_mode = 'corners'
            self.get_logger().info("切换到角点测试模式")
        else:
            self.test_mode = 'single_point'
            self.get_logger().info("切换到单点测试模式")
        
        self.test_time = 0.0

def main(args=None):
    rclpy.init(args=args)
    
    tester = SimpleGimbalTester()
    
    try:
        # 运行30秒单点测试
        print("开始单点圆形轨迹测试...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < 30:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
        # 切换到角点测试
        print("切换到角点测试...")
        tester.switch_test_mode()
        
        # 运行20秒角点测试
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 20:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
        print("测试完成!")
        
    except KeyboardInterrupt:
        tester.get_logger().info("测试被用户中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
