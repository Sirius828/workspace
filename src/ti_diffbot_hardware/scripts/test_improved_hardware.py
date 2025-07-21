#!/usr/bin/env python3
"""
改进的硬件响应测试工具 - 更宽松的检测逻辑
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math
from collections import deque

class ImprovedHardwareTest(Node):
    def __init__(self):
        super().__init__('improved_hardware_test')
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 订阅反馈
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 数据存储
        self.velocity_history = deque(maxlen=500)  # (时间, 线速度, 角速度)
        self.position_history = deque(maxlen=500)  # (时间, x, y, 朝向)
        
        # 当前状态
        self.latest_odom = None
        self.latest_joint = None
        
        self.get_logger().info("🔧 改进的硬件响应测试工具启动")
        
    def odom_callback(self, msg):
        timestamp = time.time()
        pos = msg.pose.pose.position
        vel = msg.twist.twist
        
        # 记录速度历史
        self.velocity_history.append((timestamp, vel.linear.x, vel.angular.z))
        
        # 记录位置历史
        self.position_history.append((timestamp, pos.x, pos.y))
        
        self.latest_odom = msg
        
    def joint_callback(self, msg):
        self.latest_joint = msg
        
    def send_command_and_monitor(self, linear, angular, duration=3.0, description=""):
        """发送命令并监控响应"""
        print(f"\n🧪 测试: {description}")
        print(f"   目标: 线速度={linear:.2f}m/s, 角速度={angular:.2f}rad/s")
        
        # 记录初始状态
        initial_pos = None
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            initial_pos = (pos.x, pos.y)
        
        # 清空速度历史，准备记录新的响应
        self.velocity_history.clear()
        
        # 发送命令
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        
        cmd_time = time.time()
        print(f"   📤 命令发送时间: {cmd_time:.3f}")
        
        # 监控响应
        max_linear_vel = 0.0
        max_angular_vel = 0.0
        response_detected = False
        first_response_time = None
        
        start_monitor = time.time()
        while time.time() - start_monitor < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.velocity_history:
                latest_vel = self.velocity_history[-1]
                vel_time, v_linear, v_angular = latest_vel
                
                # 记录最大速度
                max_linear_vel = max(max_linear_vel, abs(v_linear))
                max_angular_vel = max(max_angular_vel, abs(v_angular))
                
                # 检测是否有响应（使用更宽松的阈值）
                if not response_detected:
                    if (abs(v_linear) > 0.02 and linear != 0) or \
                       (abs(v_angular) > 0.05 and angular != 0) or \
                       (abs(v_linear) < 0.01 and abs(v_angular) < 0.01 and linear == 0 and angular == 0):
                        response_detected = True
                        first_response_time = vel_time - cmd_time
                        print(f"   ✅ 首次响应检测: {first_response_time*1000:.1f}ms后")
            
            time.sleep(0.02)  # 50Hz监控频率
        
        # 分析结果
        print(f"   📊 监控结果:")
        print(f"     最大线速度: {max_linear_vel:.3f}m/s (目标: {abs(linear):.3f})")
        print(f"     最大角速度: {max_angular_vel:.3f}rad/s (目标: {abs(angular):.3f})")
        
        if response_detected:
            print(f"     ✅ 响应检测成功，延迟: {first_response_time*1000:.1f}ms")
        else:
            print(f"     ❌ 未检测到明显响应")
        
        # 检查位置变化
        if initial_pos and self.latest_odom:
            final_pos = self.latest_odom.pose.pose.position
            distance_moved = math.sqrt(
                (final_pos.x - initial_pos[0])**2 + 
                (final_pos.y - initial_pos[1])**2
            )
            print(f"     📏 位置变化: {distance_moved:.4f}m")
            
            if distance_moved > 0.001:  # 1mm阈值
                print(f"     ✅ 确认机器人有实际移动")
            else:
                print(f"     ⚠️  位置变化很小")
        
        return response_detected, first_response_time if response_detected else None
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🚀 综合硬件响应测试")
        print("=" * 60)
        
        test_cases = [
            (0.05, 0.0, "慢速前进"),
            (0.0, 0.0, "停止1"),
            (0.1, 0.0, "中速前进"), 
            (0.0, 0.0, "停止2"),
            (-0.05, 0.0, "慢速后退"),
            (0.0, 0.0, "停止3"),
            (0.0, 0.2, "慢速左转"),
            (0.0, 0.0, "停止4"),
            (0.0, -0.2, "慢速右转"),
            (0.0, 0.0, "停止5"),
            (0.05, 0.1, "前进+左转"),
            (0.0, 0.0, "最终停止")
        ]
        
        successful_tests = 0
        response_times = []
        
        for linear, angular, description in test_cases:
            success, resp_time = self.send_command_and_monitor(
                linear, angular, duration=2.0, description=description)
            
            if success:
                successful_tests += 1
                if resp_time:
                    response_times.append(resp_time)
            
            time.sleep(1.0)  # 测试间隔
        
        # 总结
        print(f"\n📋 测试总结:")
        print(f"   成功率: {successful_tests}/{len(test_cases)} ({successful_tests/len(test_cases)*100:.1f}%)")
        
        if response_times:
            avg_response = sum(response_times) * 1000 / len(response_times)
            min_response = min(response_times) * 1000
            max_response = max(response_times) * 1000
            
            print(f"   平均响应时间: {avg_response:.1f}ms")
            print(f"   响应时间范围: {min_response:.1f}ms - {max_response:.1f}ms")
            
            if avg_response < 100:
                print("   ✅ 响应性能良好")
            else:
                print("   ⚠️  响应性能需要优化")
        
        return successful_tests / len(test_cases)
    
    def run_realtime_monitoring(self, duration=10.0):
        """运行实时监控测试"""
        print(f"\n🔄 实时监控测试 ({duration}秒)")
        print("=" * 60)
        
        print("📊 发送连续变化的命令，实时监控响应...")
        
        start_time = time.time()
        command_count = 0
        velocity_samples = []
        
        while time.time() - start_time < duration:
            # 生成时变命令
            t = time.time() - start_time
            
            # 缓慢变化的速度命令
            linear = 0.08 * math.sin(2 * math.pi * 0.2 * t)  # 0.2Hz正弦波
            angular = 0.15 * math.cos(2 * math.pi * 0.15 * t)  # 0.15Hz余弦波
            
            # 发送命令
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)
            command_count += 1
            
            # 收集数据
            rclpy.spin_once(self, timeout_sec=0.001)
            
            if self.latest_odom:
                vel = self.latest_odom.twist.twist
                velocity_samples.append((t, linear, angular, vel.linear.x, vel.angular.z))
            
            time.sleep(0.05)  # 20Hz命令频率
        
        # 停止机器人
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # 分析实时性能
        if velocity_samples:
            print(f"📈 实时性能分析:")
            print(f"   发送命令数: {command_count}")
            print(f"   速度样本数: {len(velocity_samples)}")
            
            # 计算跟踪误差
            linear_errors = []
            angular_errors = []
            
            for sample in velocity_samples:
                t, cmd_linear, cmd_angular, actual_linear, actual_angular = sample
                linear_errors.append(abs(actual_linear - cmd_linear))
                angular_errors.append(abs(actual_angular - cmd_angular))
            
            avg_linear_error = sum(linear_errors) / len(linear_errors)
            avg_angular_error = sum(angular_errors) / len(angular_errors)
            
            print(f"   平均线速度误差: {avg_linear_error:.4f}m/s")
            print(f"   平均角速度误差: {avg_angular_error:.4f}rad/s")
            
            if avg_linear_error < 0.02 and avg_angular_error < 0.05:
                print("   ✅ 跟踪精度优秀")
            elif avg_linear_error < 0.05 and avg_angular_error < 0.1:
                print("   ✅ 跟踪精度良好")
            else:
                print("   ⚠️  跟踪精度需要改进")
    
    def run_step_response_analysis(self):
        """运行详细的阶跃响应分析"""
        print(f"\n📈 详细阶跃响应分析")
        print("=" * 60)
        
        # 测试前进阶跃
        print("🚀 前进阶跃响应测试...")
        
        # 确保初始静止
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        time.sleep(1.0)
        
        # 清空历史
        self.velocity_history.clear()
        
        # 发送阶跃命令
        target_speed = 0.1
        cmd.linear.x = target_speed
        self.cmd_pub.publish(cmd)
        
        start_time = time.time()
        
        # 记录响应过程
        response_data = []
        
        for i in range(150):  # 记录3秒，20Hz
            rclpy.spin_once(self, timeout_sec=0.001)
            
            if self.latest_odom:
                elapsed = time.time() - start_time
                actual_speed = self.latest_odom.twist.twist.linear.x
                response_data.append((elapsed, actual_speed))
            
            time.sleep(0.02)
        
        # 停止机器人
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        
        # 分析响应特性
        if response_data:
            print(f"📊 阶跃响应特性:")
            
            # 寻找10%, 63%, 90%响应时间
            target_10 = target_speed * 0.1
            target_63 = target_speed * 0.63
            target_90 = target_speed * 0.9
            
            time_10 = time_63 = time_90 = None
            
            for elapsed, speed in response_data:
                if time_10 is None and abs(speed) >= target_10:
                    time_10 = elapsed
                if time_63 is None and abs(speed) >= target_63:
                    time_63 = elapsed
                if time_90 is None and abs(speed) >= target_90:
                    time_90 = elapsed
            
            print(f"   目标速度: {target_speed:.3f}m/s")
            if time_10: print(f"   10%响应时间: {time_10:.3f}s")
            if time_63: print(f"   63%响应时间: {time_63:.3f}s")  
            if time_90: print(f"   90%响应时间: {time_90:.3f}s")
            
            final_speed = response_data[-1][1] if response_data else 0
            print(f"   最终速度: {final_speed:.3f}m/s")
            print(f"   稳态误差: {abs(final_speed - target_speed):.3f}m/s")
            
            if time_90 and time_90 < 1.0:
                print("   ✅ 响应速度优秀")
            elif time_90 and time_90 < 2.0:
                print("   ✅ 响应速度良好")
            else:
                print("   ⚠️  响应速度较慢")

def main():
    rclpy.init()
    
    test_node = ImprovedHardwareTest()
    
    print("🔧 改进的硬件响应测试工具")
    print("📋 更宽松的检测逻辑，更准确的性能评估")
    
    try:
        # 等待初始数据
        print("\n⏳ 等待传感器数据...")
        timeout = 5.0
        start_wait = time.time()
        
        while (not test_node.latest_odom or not test_node.latest_joint) and \
              (time.time() - start_wait < timeout):
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        if not test_node.latest_odom:
            print("❌ 里程计数据超时，请检查系统状态")
            return
        
        print("✅ 传感器数据准备就绪")
        
        # 运行测试
        success_rate = test_node.run_comprehensive_test()
        time.sleep(2)
        
        if success_rate > 0.7:  # 成功率超过70%继续详细测试
            test_node.run_realtime_monitoring(duration=8.0)
            time.sleep(2)
            test_node.run_step_response_analysis()
        else:
            print("⚠️  基础测试成功率较低，跳过高级测试")
        
        print("\n🎉 改进测试完成！")
        
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
