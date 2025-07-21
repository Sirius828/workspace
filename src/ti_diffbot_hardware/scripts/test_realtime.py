#!/usr/bin/env python3
"""
实时性能测试工具 - 测试控制回路响应时间和延迟
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math
from collections import deque
import threading

class RealTimeTest(Node):
    def __init__(self):
        super().__init__('realtime_test')
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 订阅反馈
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 测试数据
        self.command_times = deque(maxlen=1000)
        self.odom_response_times = deque(maxlen=1000)
        self.joint_response_times = deque(maxlen=1000)
        
        # 当前状态
        self.latest_odom = None
        self.latest_joint = None
        self.test_running = False
        
        # 命令序列锁
        self.command_lock = threading.Lock()
        self.pending_command = None
        self.command_sent_time = None
        
        self.get_logger().info("🚀 实时性能测试工具启动")
        
    def odom_callback(self, msg):
        current_time = time.time()
        self.latest_odom = msg
        
        # 记录响应时间
        with self.command_lock:
            if self.command_sent_time is not None:
                response_time = current_time - self.command_sent_time
                self.odom_response_times.append(response_time * 1000)  # 转换为毫秒
        
    def joint_callback(self, msg):
        current_time = time.time()
        self.latest_joint = msg
        
        # 记录响应时间
        with self.command_lock:
            if self.command_sent_time is not None:
                response_time = current_time - self.command_sent_time
                self.joint_response_times.append(response_time * 1000)  # 转换为毫秒
                
    def send_command_and_measure(self, linear, angular):
        """发送命令并测量响应时间"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        with self.command_lock:
            self.command_sent_time = time.time()
            self.cmd_pub.publish(cmd)
            self.command_times.append(self.command_sent_time)
        
        return self.command_sent_time
        
    def calculate_stats(self, times):
        """计算统计数据"""
        if not times:
            return {"min": 0, "max": 0, "avg": 0, "count": 0}
            
        times_list = list(times)
        return {
            "min": min(times_list),
            "max": max(times_list),
            "avg": sum(times_list) / len(times_list),
            "count": len(times_list)
        }
    
    def run_response_test(self):
        """运行响应时间测试"""
        print("\n🧪 响应时间测试 - 发送不同速度命令并测量反馈延迟")
        print("=" * 60)
        
        # 清空之前的数据
        self.odom_response_times.clear()
        self.joint_response_times.clear()
        
        test_commands = [
            (0.1, 0.0),    # 前进
            (0.0, 0.0),    # 停止
            (-0.1, 0.0),   # 后退
            (0.0, 0.0),    # 停止
            (0.0, 0.5),    # 左转
            (0.0, 0.0),    # 停止
            (0.0, -0.5),   # 右转
            (0.0, 0.0),    # 停止
            (0.1, 0.3),    # 弧线运动
            (0.0, 0.0),    # 停止
        ]
        
        print(f"📋 测试序列: {len(test_commands)} 个命令")
        
        for i, (linear, angular) in enumerate(test_commands):
            print(f"📤 命令 {i+1}/{len(test_commands)}: 线速度={linear:.1f}, 角速度={angular:.1f}")
            
            # 发送命令
            cmd_time = self.send_command_and_measure(linear, angular)
            
            # 等待响应并收集数据
            wait_time = 1.0  # 等待1秒收集响应
            end_time = time.time() + wait_time
            
            while time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.01)
            
            # 清除pending状态，准备下一个命令
            with self.command_lock:
                self.command_sent_time = None
            
            time.sleep(0.5)  # 命令间隔
        
        # 分析结果
        print("\n📊 响应时间分析:")
        print("-" * 60)
        
        odom_stats = self.calculate_stats(self.odom_response_times)
        joint_stats = self.calculate_stats(self.joint_response_times)
        
        print(f"🗺️  里程计响应时间 (ms):")
        print(f"   最小: {odom_stats['min']:.1f}")
        print(f"   最大: {odom_stats['max']:.1f}")
        print(f"   平均: {odom_stats['avg']:.1f}")
        print(f"   样本数: {odom_stats['count']}")
        
        print(f"⚙️  关节状态响应时间 (ms):")
        print(f"   最小: {joint_stats['min']:.1f}")
        print(f"   最大: {joint_stats['max']:.1f}")
        print(f"   平均: {joint_stats['avg']:.1f}")
        print(f"   样本数: {joint_stats['count']}")
        
        # 性能评估
        print(f"\n🏥 性能评估:")
        avg_response = min(odom_stats['avg'], joint_stats['avg'])
        
        if avg_response < 50:
            print("   ✅ 优秀: 响应时间 < 50ms")
        elif avg_response < 100:
            print("   ✅ 良好: 响应时间 < 100ms")
        elif avg_response < 200:
            print("   ⚠️  一般: 响应时间 < 200ms")
        else:
            print("   ❌ 差: 响应时间 > 200ms")
            print("   💡 建议优化硬件接口或降低控制频率")
    
    def run_throughput_test(self, duration=10.0):
        """运行吞吐量测试"""
        print(f"\n🚄 吞吐量测试 - {duration}秒连续命令发送")
        print("=" * 60)
        
        command_count = 0
        start_time = time.time()
        
        # 正弦波形速度命令
        frequency = 1.0  # 1Hz正弦波
        amplitude = 0.2  # 最大速度
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            linear = amplitude * math.sin(2 * math.pi * frequency * t)
            angular = amplitude * 0.5 * math.cos(2 * math.pi * frequency * t)
            
            self.send_command_and_measure(linear, angular)
            command_count += 1
            
            # 短暂延迟模拟高频控制
            time.sleep(0.02)  # 50Hz控制频率
            
            # 处理消息
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # 停止机器人
        self.send_command_and_measure(0.0, 0.0)
        
        actual_duration = time.time() - start_time
        command_rate = command_count / actual_duration
        
        print(f"📈 吞吐量结果:")
        print(f"   发送命令数: {command_count}")
        print(f"   实际时间: {actual_duration:.1f}s")
        print(f"   命令频率: {command_rate:.1f} Hz")
        print(f"   目标频率: 50 Hz")
        
        if command_rate >= 45:
            print("   ✅ 优秀: 接近目标频率")
        elif command_rate >= 30:
            print("   ✅ 良好: 可接受的控制频率")
        else:
            print("   ⚠️  需要优化: 控制频率过低")
    
    def run_step_response_test(self):
        """运行阶跃响应测试"""
        print("\n📈 阶跃响应测试 - 测试速度变化的响应特性")
        print("=" * 60)
        
        if not self.latest_odom:
            print("❌ 无里程计数据，跳过测试")
            return
        
        # 记录初始位置
        initial_pos = self.latest_odom.pose.pose.position
        
        print("📍 初始位置记录完成")
        print("🚀 发送前进命令...")
        
        # 发送阶跃命令
        step_speed = 0.2  # 0.2 m/s
        self.send_command_and_measure(step_speed, 0.0)
        
        # 记录响应过程
        positions = []
        velocities = []
        timestamps = []
        
        start_time = time.time()
        test_duration = 3.0
        
        while time.time() - start_time < test_duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.latest_odom:
                current_time = time.time() - start_time
                pos = self.latest_odom.pose.pose.position
                vel = self.latest_odom.twist.twist.linear
                
                # 计算距离初始位置的距离
                distance = math.sqrt((pos.x - initial_pos.x)**2 + (pos.y - initial_pos.y)**2)
                
                timestamps.append(current_time)
                positions.append(distance)
                velocities.append(vel.x)
        
        # 停止机器人
        self.send_command_and_measure(0.0, 0.0)
        
        # 分析响应特性
        if len(velocities) > 10:
            # 找到90%响应时间
            target_velocity = step_speed * 0.9
            response_time_90 = None
            
            for i, vel in enumerate(velocities):
                if abs(vel) >= target_velocity:
                    response_time_90 = timestamps[i]
                    break
            
            final_distance = positions[-1] if positions else 0
            final_velocity = velocities[-1] if velocities else 0
            
            print(f"📊 阶跃响应分析:")
            print(f"   目标速度: {step_speed:.2f} m/s")
            print(f"   最终速度: {final_velocity:.2f} m/s")
            print(f"   90%响应时间: {response_time_90:.2f}s" if response_time_90 else "   90%响应时间: 未达到")
            print(f"   总移动距离: {final_distance:.3f}m")
            
            if response_time_90 and response_time_90 < 0.5:
                print("   ✅ 响应快速")
            elif response_time_90 and response_time_90 < 1.0:
                print("   ✅ 响应正常")
            else:
                print("   ⚠️  响应较慢")

def main():
    rclpy.init()
    
    test_node = RealTimeTest()
    
    print("🤖 实时性能测试工具")
    print("📋 测试内容:")
    print("   1. 响应时间测试 - 命令到反馈的延迟")
    print("   2. 吞吐量测试 - 高频命令处理能力")
    print("   3. 阶跃响应测试 - 速度变化响应特性")
    print("\n⚠️  请确保机器人有足够的活动空间")
    print("🔧 测试前请确保系统正常运行")
    
    try:
        # 等待初始数据
        print("\n⏳ 等待传感器数据...")
        timeout = 5.0
        start_wait = time.time()
        
        while (not test_node.latest_odom or not test_node.latest_joint) and \
              (time.time() - start_wait < timeout):
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        if not test_node.latest_odom or not test_node.latest_joint:
            print("❌ 传感器数据超时，请检查系统状态")
            return
        
        print("✅ 传感器数据准备就绪")
        
        # 运行测试
        test_node.run_response_test()
        time.sleep(2)
        
        test_node.run_throughput_test(duration=5.0)
        time.sleep(2)
        
        test_node.run_step_response_test()
        
        print("\n🎉 所有测试完成！")
        
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试发生错误: {e}")
    finally:
        # 确保停止机器人
        cmd = Twist()
        test_node.cmd_pub.publish(cmd)
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
