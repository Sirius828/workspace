#!/usr/bin/env python3
"""
导航阻塞诊断工具 - 专门检测长距离导航时的里程计阻塞问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math
import threading
import psutil
import os
from collections import deque

class NavigationBlockingDiagnostic(Node):
    def __init__(self):
        super().__init__('nav_blocking_diagnostic')
        
        # 发布器
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        
        # 订阅器
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 数据记录
        self.odom_timestamps = deque(maxlen=2000)
        self.joint_timestamps = deque(maxlen=2000)
        self.cpu_usage_log = deque(maxlen=1000)
        self.memory_usage_log = deque(maxlen=1000)
        
        # 状态监控
        self.latest_odom = None
        self.latest_joint = None
        self.monitoring = False
        self.test_start_time = None
        
        # 创建监控线程
        self.monitor_thread = None
        
        self.get_logger().info("🚨 导航阻塞诊断工具启动")
        
    def odom_callback(self, msg):
        current_time = time.time()
        self.odom_timestamps.append(current_time)
        self.latest_odom = msg
        
        # 检测阻塞
        if self.monitoring and len(self.odom_timestamps) >= 2:
            time_gap = current_time - self.odom_timestamps[-2]
            if time_gap > 0.2:  # 超过200ms间隔认为有阻塞
                self.get_logger().warn(f"🚨 里程计阻塞检测: {time_gap*1000:.1f}ms 间隔")
        
    def joint_callback(self, msg):
        current_time = time.time()
        self.joint_timestamps.append(current_time)
        self.latest_joint = msg
        
    def start_system_monitoring(self):
        """启动系统资源监控"""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_system_resources)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
    def stop_system_monitoring(self):
        """停止系统资源监控"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            
    def _monitor_system_resources(self):
        """监控系统资源使用情况"""
        while self.monitoring:
            try:
                # CPU使用率
                cpu_percent = psutil.cpu_percent(interval=0.1)
                
                # 内存使用率
                memory = psutil.virtual_memory()
                memory_percent = memory.percent
                
                # 记录数据
                timestamp = time.time()
                self.cpu_usage_log.append((timestamp, cpu_percent))
                self.memory_usage_log.append((timestamp, memory_percent))
                
                # 检测资源异常
                if cpu_percent > 80:
                    self.get_logger().warn(f"⚠️  高CPU使用率: {cpu_percent:.1f}%")
                    
                if memory_percent > 80:
                    self.get_logger().warn(f"⚠️  高内存使用率: {memory_percent:.1f}%")
                    
                time.sleep(0.1)  # 10Hz监控频率
                
            except Exception as e:
                self.get_logger().error(f"资源监控错误: {e}")
                break
    
    def send_goal_pose(self, x, y, yaw=0.0):
        """发送目标位置"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "odom"
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # 转换yaw为四元数
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f"🎯 发送目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        
    def analyze_data_flow(self, start_time, end_time):
        """分析数据流质量"""
        # 过滤时间范围内的数据
        odom_in_range = [t for t in self.odom_timestamps if start_time <= t <= end_time]
        joint_in_range = [t for t in self.joint_timestamps if start_time <= t <= end_time]
        
        print(f"\n📊 数据流分析 ({end_time - start_time:.1f}秒):")
        
        # 里程计分析
        if len(odom_in_range) >= 2:
            odom_intervals = [odom_in_range[i+1] - odom_in_range[i] 
                            for i in range(len(odom_in_range)-1)]
            
            avg_odom_interval = sum(odom_intervals) / len(odom_intervals) * 1000
            max_odom_interval = max(odom_intervals) * 1000
            odom_freq = len(odom_in_range) / (end_time - start_time)
            
            # 统计阻塞次数
            blocking_count = sum(1 for interval in odom_intervals if interval > 0.2)
            
            print(f"📈 里程计数据:")
            print(f"   总数据点: {len(odom_in_range)}")
            print(f"   平均频率: {odom_freq:.1f} Hz")
            print(f"   平均间隔: {avg_odom_interval:.1f}ms")
            print(f"   最大间隔: {max_odom_interval:.1f}ms")
            print(f"   阻塞次数: {blocking_count} 次 (>200ms)")
            
            if blocking_count > 0:
                print(f"   ❌ 检测到 {blocking_count} 次严重阻塞!")
            elif max_odom_interval > 100:
                print(f"   ⚠️  存在较长延迟")
            else:
                print(f"   ✅ 数据流稳定")
        else:
            print(f"   ❌ 里程计数据不足")
            
        # 关节状态分析
        if len(joint_in_range) >= 2:
            joint_freq = len(joint_in_range) / (end_time - start_time)
            print(f"⚙️  关节状态:")
            print(f"   总数据点: {len(joint_in_range)}")
            print(f"   平均频率: {joint_freq:.1f} Hz")
        
    def analyze_system_resources(self, start_time, end_time):
        """分析系统资源使用"""
        # 过滤时间范围内的资源数据
        cpu_in_range = [(t, usage) for t, usage in self.cpu_usage_log if start_time <= t <= end_time]
        memory_in_range = [(t, usage) for t, usage in self.memory_usage_log if start_time <= t <= end_time]
        
        print(f"\n💻 系统资源分析:")
        
        if cpu_in_range:
            cpu_values = [usage for _, usage in cpu_in_range]
            avg_cpu = sum(cpu_values) / len(cpu_values)
            max_cpu = max(cpu_values)
            
            print(f"🔥 CPU使用率:")
            print(f"   平均: {avg_cpu:.1f}%")
            print(f"   最高: {max_cpu:.1f}%")
            
            if max_cpu > 90:
                print(f"   ❌ CPU资源严重不足!")
            elif max_cpu > 70:
                print(f"   ⚠️  CPU负载较高")
            else:
                print(f"   ✅ CPU负载正常")
                
        if memory_in_range:
            memory_values = [usage for _, usage in memory_in_range]
            avg_memory = sum(memory_values) / len(memory_values)
            max_memory = max(memory_values)
            
            print(f"💾 内存使用率:")
            print(f"   平均: {avg_memory:.1f}%")
            print(f"   最高: {max_memory:.1f}%")
    
    def run_short_command_test(self):
        """运行短期命令测试作为基准"""
        print("\n🏃 短期命令测试 (基准测试)")
        print("=" * 50)
        
        # 清空数据
        self.odom_timestamps.clear()
        self.joint_timestamps.clear()
        
        # 启动监控
        self.start_system_monitoring()
        
        test_start = time.time()
        
        # 发送简单速度命令
        print("📤 发送简单速度命令...")
        cmd = Twist()
        cmd.linear.x = 0.1
        self.cmd_pub.publish(cmd)
        
        # 监控5秒
        duration = 5.0
        while time.time() - test_start < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            
        # 停止
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        
        test_end = time.time()
        self.stop_system_monitoring()
        
        # 分析结果
        self.analyze_data_flow(test_start, test_end)
        self.analyze_system_resources(test_start, test_end)
        
    def run_goal_navigation_test(self):
        """运行目标导航测试"""
        print("\n🎯 目标导航测试 (阻塞检测)")
        print("=" * 50)
        
        if not self.latest_odom:
            print("❌ 无法获取当前位置，跳过测试")
            return
            
        # 记录当前位置
        current_pos = self.latest_odom.pose.pose.position
        print(f"📍 当前位置: x={current_pos.x:.2f}, y={current_pos.y:.2f}")
        
        # 清空数据
        self.odom_timestamps.clear()
        self.joint_timestamps.clear()
        self.cpu_usage_log.clear()
        self.memory_usage_log.clear()
        
        # 启动监控
        self.start_system_monitoring()
        
        test_start = time.time()
        
        # 发送长距离目标
        target_x = current_pos.x + 1.0  # 前进1米
        target_y = current_pos.y + 0.5  # 侧向0.5米
        
        print(f"🚀 发送长距离目标: x={target_x:.2f}, y={target_y:.2f}")
        self.send_goal_pose(target_x, target_y)
        
        # 监控导航过程
        duration = 15.0  # 监控15秒
        print(f"👀 监控导航过程 {duration}秒...")
        
        last_report_time = test_start
        
        while time.time() - test_start < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # 每3秒报告一次状态
            if time.time() - last_report_time >= 3.0:
                if self.latest_odom:
                    pos = self.latest_odom.pose.pose.position
                    vel = self.latest_odom.twist.twist
                    distance_to_goal = math.sqrt((pos.x - target_x)**2 + (pos.y - target_y)**2)
                    
                    print(f"📊 状态更新: 位置=({pos.x:.2f}, {pos.y:.2f}), "
                          f"速度={vel.linear.x:.2f}m/s, 距目标={distance_to_goal:.2f}m")
                          
                last_report_time = time.time()
                
                # 检查是否到达目标
                if self.latest_odom:
                    pos = self.latest_odom.pose.pose.position
                    distance_to_goal = math.sqrt((pos.x - target_x)**2 + (pos.y - target_y)**2)
                    if distance_to_goal < 0.1:
                        print("🎉 已到达目标!")
                        break
        
        test_end = time.time()
        self.stop_system_monitoring()
        
        # 发送停止命令
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # 分析结果
        self.analyze_data_flow(test_start, test_end)
        self.analyze_system_resources(test_start, test_end)
        
        # 检测阻塞模式
        self.detect_blocking_patterns(test_start, test_end)
        
    def detect_blocking_patterns(self, start_time, end_time):
        """检测阻塞模式"""
        print(f"\n🔍 阻塞模式检测:")
        
        # 分析里程计时间间隔
        odom_in_range = [t for t in self.odom_timestamps if start_time <= t <= end_time]
        
        if len(odom_in_range) >= 2:
            intervals = [odom_in_range[i+1] - odom_in_range[i] 
                        for i in range(len(odom_in_range)-1)]
            
            # 检测不同程度的阻塞
            minor_blocks = sum(1 for interval in intervals if 0.1 < interval <= 0.2)
            major_blocks = sum(1 for interval in intervals if 0.2 < interval <= 0.5)
            severe_blocks = sum(1 for interval in intervals if interval > 0.5)
            
            print(f"📈 阻塞统计:")
            print(f"   轻微阻塞 (100-200ms): {minor_blocks} 次")
            print(f"   主要阻塞 (200-500ms): {major_blocks} 次")
            print(f"   严重阻塞 (>500ms): {severe_blocks} 次")
            
            # 检测阻塞时机
            if len(intervals) > 10:
                # 检查前10%的时间是否有阻塞 (启动阶段)
                startup_intervals = intervals[:len(intervals)//10]
                startup_blocks = sum(1 for interval in startup_intervals if interval > 0.2)
                
                if startup_blocks > 0:
                    print(f"🚨 启动阶段检测到 {startup_blocks} 次阻塞 - 这可能是导航算法初始化导致的!")
                    
            # 分析CPU峰值与阻塞的关联
            cpu_in_range = [(t, usage) for t, usage in self.cpu_usage_log if start_time <= t <= end_time]
            if cpu_in_range:
                high_cpu_times = [t for t, usage in cpu_in_range if usage > 80]
                
                # 检查高CPU时段是否对应阻塞
                cpu_related_blocks = 0
                for interval_start in [odom_in_range[i] for i, interval in enumerate(intervals) if interval > 0.2]:
                    for cpu_time in high_cpu_times:
                        if abs(interval_start - cpu_time) < 1.0:  # 1秒内的关联
                            cpu_related_blocks += 1
                            break
                
                if cpu_related_blocks > 0:
                    print(f"💻 发现 {cpu_related_blocks} 次阻塞与高CPU使用率相关")

def main():
    rclpy.init()
    
    diagnostic = NavigationBlockingDiagnostic()
    
    print("🚨 导航阻塞诊断工具")
    print("📋 专门检测长距离导航时的里程计阻塞问题")
    print("🎯 这将帮助找出goal_pose导航时的性能瓶颈")
    
    try:
        # 等待初始数据
        print("\n⏳ 等待传感器数据...")
        timeout = 5.0
        start_wait = time.time()
        
        while (not diagnostic.latest_odom or not diagnostic.latest_joint) and \
              (time.time() - start_wait < timeout):
            rclpy.spin_once(diagnostic, timeout_sec=0.1)
        
        if not diagnostic.latest_odom or not diagnostic.latest_joint:
            print("❌ 传感器数据超时，请检查系统状态")
            return
        
        print("✅ 传感器数据准备就绪")
        
        # 运行基准测试
        diagnostic.run_short_command_test()
        time.sleep(3)
        
        # 运行导航阻塞测试
        input("\n按Enter开始目标导航测试 (确保机器人有足够空间移动)...")
        diagnostic.run_goal_navigation_test()
        
        print("\n🎉 导航阻塞诊断完成!")
        print("💡 如果发现阻塞问题，建议:")
        print("   1. 优化导航算法的计算频率")
        print("   2. 增加硬件接口的线程优先级")
        print("   3. 使用实时内核或调整调度策略")
        
    except KeyboardInterrupt:
        print("\n🛑 诊断被用户中断")
    except Exception as e:
        print(f"\n❌ 诊断发生错误: {e}")
    finally:
        # 确保停止机器人
        cmd = Twist()
        diagnostic.cmd_pub.publish(cmd)
        diagnostic.stop_system_monitoring()
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
