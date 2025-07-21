#!/usr/bin/env python3
"""
时间戳同步测试工具 - 测试命令发送与里程计响应的时间同步
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math
from collections import deque

class TimestampSyncTest(Node):
    def __init__(self):
        super().__init__('timestamp_sync_test')
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 订阅反馈
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 测试数据
        self.command_log = deque(maxlen=1000)  # (时间戳, 线速度, 角速度)
        self.odom_log = deque(maxlen=1000)     # (时间戳, 位置x, 位置y, 线速度, 角速度)
        
        # 当前状态
        self.latest_odom = None
        self.latest_joint = None
        
        self.get_logger().info("⏱️  时间戳同步测试工具启动")
        
    def odom_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds / 1e9
        pos = msg.pose.pose.position
        vel = msg.twist.twist
        
        self.odom_log.append((
            timestamp, 
            pos.x, pos.y, 
            vel.linear.x, vel.angular.z
        ))
        
        self.latest_odom = msg
        
    def joint_callback(self, msg):
        self.latest_joint = msg
        
    def send_command(self, linear, angular):
        """发送命令并记录时间戳"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        self.cmd_pub.publish(cmd)
        self.command_log.append((timestamp, linear, angular))
        
        return timestamp
        
    def analyze_response_delay(self, command_time, target_velocity, tolerance=0.05):
        """分析响应延迟"""
        if not self.odom_log:
            return None
            
        # 寻找达到目标速度的时间点
        for odom_entry in self.odom_log:
            odom_time, _, _, v_linear, v_angular = odom_entry
            
            if odom_time > command_time:  # 命令发出之后的数据
                if (abs(v_linear - target_velocity) < tolerance or 
                    abs(v_angular - target_velocity) < tolerance):
                    return odom_time - command_time
                    
        return None
    
    def run_step_response_test(self):
        """运行阶跃响应时间测试"""
        print("\n📈 阶跃响应时间测试")
        print("=" * 50)
        
        # 清空历史数据
        self.command_log.clear()
        self.odom_log.clear()
        
        test_cases = [
            (0.2, 0.0, "前进"),
            (0.0, 0.0, "停止"), 
            (-0.2, 0.0, "后退"),
            (0.0, 0.0, "停止"),
            (0.0, 0.5, "左转"),
            (0.0, 0.0, "停止"),
            (0.0, -0.5, "右转"),
            (0.0, 0.0, "停止")
        ]
        
        response_times = []
        
        for linear, angular, description in test_cases:
            print(f"\n🧪 测试: {description} (线速度={linear}, 角速度={angular})")
            
            # 发送命令
            cmd_time = self.send_command(linear, angular)
            
            # 等待并收集响应数据
            start_wait = time.time()
            response_detected = False
            
            while time.time() - start_wait < 2.0:  # 等待2秒
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 检查是否达到目标速度
                if self.odom_log:
                    latest_odom = self.odom_log[-1]
                    _, _, _, v_linear, v_angular = latest_odom
                    
                    # 检查是否接近目标速度
                    if (abs(v_linear - linear) < 0.05 and 
                        abs(v_angular - angular) < 0.05):
                        response_time = latest_odom[0] - cmd_time
                        response_times.append(response_time)
                        print(f"   ✅ 响应时间: {response_time*1000:.1f}ms")
                        response_detected = True
                        break
            
            if not response_detected:
                print(f"   ❌ 2秒内未检测到响应")
                
            time.sleep(0.5)  # 命令间隔
        
        # 停止机器人
        self.send_command(0.0, 0.0)
        
        # 分析结果
        if response_times:
            avg_response = sum(response_times) / len(response_times) * 1000  # 转换为ms
            min_response = min(response_times) * 1000
            max_response = max(response_times) * 1000
            
            print(f"\n📊 响应时间统计:")
            print(f"   平均响应时间: {avg_response:.1f}ms")
            print(f"   最快响应: {min_response:.1f}ms")
            print(f"   最慢响应: {max_response:.1f}ms")
            
            if avg_response < 50:
                print("   ✅ 优秀: 响应时间 < 50ms")
            elif avg_response < 100:
                print("   ✅ 良好: 响应时间 < 100ms")
            elif avg_response < 200:
                print("   ⚠️  一般: 响应时间 < 200ms")
            else:
                print("   ❌ 需要优化: 响应时间 > 200ms")
        else:
            print("   ❌ 未能测量到有效响应时间")
    
    def run_continuous_command_test(self):
        """运行连续命令测试"""
        print("\n🔄 连续命令响应测试")
        print("=" * 50)
        
        # 清空历史数据
        self.command_log.clear()
        self.odom_log.clear()
        
        print("🚀 发送连续变化的速度命令...")
        
        start_time = time.time()
        test_duration = 10.0
        command_count = 0
        response_delays = []
        
        while time.time() - start_time < test_duration:
            # 生成正弦波速度命令
            t = time.time() - start_time
            linear = 0.1 * math.sin(2 * math.pi * 0.5 * t)  # 0.5Hz正弦波
            angular = 0.2 * math.cos(2 * math.pi * 0.3 * t)  # 0.3Hz余弦波
            
            cmd_time = self.send_command(linear, angular)
            command_count += 1
            
            # 处理消息
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # 短暂延迟
            time.sleep(0.05)  # 20Hz命令频率
        
        # 停止机器人
        self.send_command(0.0, 0.0)
        
        # 分析数据同步性
        print(f"📈 测试结果:")
        print(f"   发送命令数: {command_count}")
        print(f"   收到里程计数据: {len(self.odom_log)}")
        
        if len(self.odom_log) > 10:
            # 分析数据流连续性
            odom_times = [entry[0] for entry in self.odom_log]
            time_gaps = [odom_times[i+1] - odom_times[i] for i in range(len(odom_times)-1)]
            
            avg_gap = sum(time_gaps) / len(time_gaps) * 1000  # ms
            max_gap = max(time_gaps) * 1000
            
            print(f"   里程计数据间隔: 平均{avg_gap:.1f}ms, 最大{max_gap:.1f}ms")
            
            if avg_gap < 20:
                print("   ✅ 里程计更新频率优秀 (>50Hz)")
            elif avg_gap < 50:
                print("   ✅ 里程计更新频率良好 (>20Hz)")
            elif avg_gap < 100:
                print("   ⚠️  里程计更新频率一般 (>10Hz)")
            else:
                print("   ❌ 里程计更新频率过低 (<10Hz)")
        else:
            print("   ❌ 里程计数据不足，可能存在严重延迟")
    
    def run_command_echo_test(self):
        """运行命令回显测试"""
        print("\n🔊 命令-反馈同步测试")
        print("=" * 50)
        
        # 清空历史数据
        self.command_log.clear() 
        self.odom_log.clear()
        
        test_commands = [
            (0.1, 0.0),   # 慢速前进
            (0.2, 0.0),   # 快速前进
            (0.0, 0.3),   # 慢速转向
            (0.0, 0.6),   # 快速转向
            (0.1, 0.2),   # 组合运动
            (0.0, 0.0)    # 停止
        ]
        
        sync_results = []
        
        for linear, angular in test_commands:
            print(f"\n📤 发送命令: 线速度={linear:.1f}, 角速度={angular:.1f}")
            
            # 发送命令
            cmd_time = self.send_command(linear, angular)
            
            # 监控反馈同步
            sync_detected = False
            wait_start = time.time()
            
            while time.time() - wait_start < 1.0:  # 等待1秒
                rclpy.spin_once(self, timeout_sec=0.01)
                
                if self.odom_log:
                    latest_odom = self.odom_log[-1]
                    odom_time, _, _, v_linear, v_angular = latest_odom
                    
                    # 检查命令是否反映在速度中
                    if odom_time > cmd_time:
                        linear_error = abs(v_linear - linear)
                        angular_error = abs(v_angular - angular)
                        
                        if linear_error < 0.05 and angular_error < 0.05:
                            sync_time = odom_time - cmd_time
                            sync_results.append(sync_time)
                            print(f"   ✅ 同步延迟: {sync_time*1000:.1f}ms")
                            sync_detected = True
                            break
            
            if not sync_detected:
                print(f"   ❌ 1秒内未检测到同步")
                
            time.sleep(1.0)  # 命令间隔
        
        # 分析同步性能
        if sync_results:
            avg_sync = sum(sync_results) / len(sync_results) * 1000
            print(f"\n📊 同步性能:")
            print(f"   平均同步延迟: {avg_sync:.1f}ms")
            print(f"   成功同步率: {len(sync_results)}/{len(test_commands)} ({len(sync_results)/len(test_commands)*100:.1f}%)")
            
            if avg_sync < 30:
                print("   ✅ 同步性能优秀")
            elif avg_sync < 100:
                print("   ✅ 同步性能良好") 
            else:
                print("   ⚠️  同步性能需要改进")
        else:
            print("   ❌ 未检测到有效的命令-反馈同步")

def main():
    rclpy.init()
    
    test_node = TimestampSyncTest()
    
    print("⏱️  时间戳同步测试工具")
    print("📋 测试目标: 验证命令发送与里程计响应的时间同步性")
    print("🔧 这将帮助诊断导航系统中的延迟问题")
    
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
        
        # 运行测试序列
        test_node.run_step_response_test()
        time.sleep(2)
        
        test_node.run_continuous_command_test()
        time.sleep(2)
        
        test_node.run_command_echo_test()
        
        print("\n🎉 时间戳同步测试完成！")
        print("💡 如果发现同步延迟过大，请检查:")
        print("   1. 硬件接口的时间戳处理")
        print("   2. 串口通信的稳定性")
        print("   3. 控制回路的执行频率")
        
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
