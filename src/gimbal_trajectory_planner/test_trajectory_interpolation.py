#!/usr/bin/env python3
"""
轨迹插值功能测试脚本
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState
import math
import time

class TrajectoryInterpolationTester(Node):
    def __init__(self):
        super().__init__('trajectory_interpolation_tester')
        
        # 发布者
        self.midline_pub = self.create_publisher(Point32, '/midline_points', 10)
        
        # 订阅者
        self.joint_sub = self.create_subscription(
            JointState, 
            '/gimbal_joint_states', 
            self.joint_callback, 
            10
        )
        
        self.test_phase = 0
        self.test_time = 0.0
        self.received_count = 0
        self.angles_history = []
        
        # 定时器
        self.test_timer = self.create_timer(0.5, self.run_test)  # 每0.5秒发送一个点
        
        self.get_logger().info("轨迹插值测试器启动")
        self.get_logger().info("将发送一系列中线点测试轨迹插值功能")
        
    def joint_callback(self, msg):
        """接收关节状态回调"""
        if len(msg.position) >= 2:
            yaw_deg = math.degrees(msg.position[0])
            pitch_deg = math.degrees(msg.position[1])
            self.received_count += 1
            
            # 记录角度历史
            self.angles_history.append((time.time(), yaw_deg, pitch_deg))
            
            # 保持最近50个记录
            if len(self.angles_history) > 50:
                self.angles_history.pop(0)
            
            if self.received_count % 25 == 0:  # 每0.5秒记录一次 (50Hz/25)
                self.get_logger().info(f"插值角度 #{self.received_count}: "
                                     f"Yaw={yaw_deg:.3f}°, Pitch={pitch_deg:.3f}°")
    
    def run_test(self):
        """运行测试"""
        self.test_time += 0.5
        
        if self.test_phase == 0:
            self.test_straight_line()
        elif self.test_phase == 1:
            self.test_circular_motion()
        elif self.test_phase == 2:
            self.test_step_changes()
        else:
            self.analyze_results()
            
    def test_straight_line(self):
        """测试直线轨迹"""
        if self.test_time <= 5.0:  # 前5秒
            # 从左到右的直线运动
            progress = self.test_time / 5.0
            x = 300 + progress * 400  # 从300到700像素
            y = 360  # 固定在中心
            
            self.send_midline_point(x, y)
            
            if self.test_time == 0.5:
                self.get_logger().info("测试阶段1: 直线轨迹 (水平移动)")
        else:
            self.test_phase = 1
            self.test_time = 0.0
            self.get_logger().info("测试阶段2: 圆形轨迹")
            
    def test_circular_motion(self):
        """测试圆形轨迹"""
        if self.test_time <= 8.0:  # 8秒圆形运动
            # 圆形轨迹
            angle = self.test_time * math.pi / 2  # 每4秒一圈
            radius = 150
            center_x, center_y = 640, 360
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            self.send_midline_point(x, y)
        else:
            self.test_phase = 2
            self.test_time = 0.0
            self.get_logger().info("测试阶段3: 阶跃变化")
            
    def test_step_changes(self):
        """测试阶跃变化"""
        if self.test_time <= 6.0:  # 6秒阶跃测试
            # 每2秒跳到一个新位置
            positions = [
                (400, 300),   # 左上
                (880, 300),   # 右上
                (880, 500),   # 右下
                (400, 500),   # 左下
            ]
            
            pos_index = int(self.test_time / 2.0)
            if pos_index < len(positions):
                x, y = positions[pos_index]
                self.send_midline_point(x, y)
                
                if self.test_time % 2.0 < 0.5:  # 只在刚跳转时记录
                    self.get_logger().info(f"阶跃到位置: ({x}, {y})")
        else:
            self.test_phase = 3
            self.get_logger().info("测试完成，分析结果...")
            
    def send_midline_point(self, x, y):
        """发送中线点"""
        point = Point32()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        
        self.midline_pub.publish(point)
        
    def analyze_results(self):
        """分析测试结果"""
        if len(self.angles_history) > 10:
            # 计算角度变化的平滑度
            angle_changes = []
            for i in range(1, len(self.angles_history)):
                prev_time, prev_yaw, prev_pitch = self.angles_history[i-1]
                curr_time, curr_yaw, curr_pitch = self.angles_history[i]
                
                dt = curr_time - prev_time
                if dt > 0:
                    yaw_rate = abs(curr_yaw - prev_yaw) / dt
                    pitch_rate = abs(curr_pitch - prev_pitch) / dt
                    angle_changes.append((yaw_rate, pitch_rate))
            
            if angle_changes:
                avg_yaw_rate = sum(c[0] for c in angle_changes) / len(angle_changes)
                avg_pitch_rate = sum(c[1] for c in angle_changes) / len(angle_changes)
                max_yaw_rate = max(c[0] for c in angle_changes)
                max_pitch_rate = max(c[1] for c in angle_changes)
                
                self.get_logger().info("=== 轨迹插值测试结果 ===")
                self.get_logger().info(f"总接收消息数: {self.received_count}")
                self.get_logger().info(f"平均偏航角速度: {avg_yaw_rate:.2f} °/s")
                self.get_logger().info(f"平均俯仰角速度: {avg_pitch_rate:.2f} °/s")
                self.get_logger().info(f"最大偏航角速度: {max_yaw_rate:.2f} °/s")
                self.get_logger().info(f"最大俯仰角速度: {max_pitch_rate:.2f} °/s")
                
                # 评估平滑性
                if max_yaw_rate < 50 and max_pitch_rate < 50:
                    self.get_logger().info("✅ 轨迹插值效果良好：运动平滑")
                else:
                    self.get_logger().info("⚠️  轨迹插值需要调优：运动较急")
        
        # 停止测试
        self.test_timer.cancel()
        self.get_logger().info("测试完成，可以Ctrl+C退出")

def main(args=None):
    rclpy.init(args=args)
    
    tester = TrajectoryInterpolationTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("测试被用户中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
