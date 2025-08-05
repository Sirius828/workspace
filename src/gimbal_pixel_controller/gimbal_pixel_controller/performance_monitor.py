#!/usr/bin/env python3
"""
云台跟踪系统性能监控脚本
监控跟踪延迟、帧率、控制频率等关键指标
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time
import statistics
from collections import deque

class GimbalPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('gimbal_performance_monitor')
        
        # 性能统计数据
        self.target_timestamps = deque(maxlen=100)
        self.control_timestamps = deque(maxlen=100)
        self.latencies = deque(maxlen=100)
        
        # 订阅目标位置和控制指令
        self.target_sub = self.create_subscription(
            Point,
            '/target_position_pixel',
            self.target_callback,
            10
        )
        
        self.control_sub = self.create_subscription(
            Twist,
            '/cmd_gimbal',
            self.control_callback,
            10
        )
        
        # 定期输出性能报告
        self.timer = self.create_timer(2.0, self.report_performance)
        
        self.get_logger().info('Gimbal performance monitor started')
    
    def target_callback(self, msg):
        """记录目标检测时间戳"""
        current_time = time.time()
        self.target_timestamps.append(current_time)
    
    def control_callback(self, msg):
        """记录控制指令时间戳"""
        current_time = time.time()
        self.control_timestamps.append(current_time)
        
        # 计算从目标检测到控制指令的延迟
        if self.target_timestamps:
            latency = current_time - self.target_timestamps[-1]
            self.latencies.append(latency * 1000)  # 转换为毫秒
    
    def report_performance(self):
        """输出性能报告"""
        if len(self.target_timestamps) < 2 or len(self.control_timestamps) < 2:
            return
        
        # 计算目标检测频率
        target_intervals = []
        for i in range(1, len(self.target_timestamps)):
            interval = self.target_timestamps[i] - self.target_timestamps[i-1]
            target_intervals.append(interval)
        
        # 计算控制频率
        control_intervals = []
        for i in range(1, len(self.control_timestamps)):
            interval = self.control_timestamps[i] - self.control_timestamps[i-1]
            control_intervals.append(interval)
        
        if target_intervals and control_intervals and self.latencies:
            target_fps = 1.0 / statistics.mean(target_intervals)
            control_fps = 1.0 / statistics.mean(control_intervals)
            avg_latency = statistics.mean(self.latencies)
            max_latency = max(self.latencies)
            min_latency = min(self.latencies)
            
            self.get_logger().info(
                f'性能报告 - '
                f'目标检测: {target_fps:.1f}Hz | '
                f'控制频率: {control_fps:.1f}Hz | '
                f'平均延迟: {avg_latency:.1f}ms | '
                f'最大延迟: {max_latency:.1f}ms | '
                f'最小延迟: {min_latency:.1f}ms'
            )

def main(args=None):
    rclpy.init(args=args)
    monitor = GimbalPerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('性能监控已停止')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
