#!/usr/bin/env python3
"""
里程计诊断工具 - 实时监控里程计更新频率和数据质量
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math
from collections import deque

class OdomDiagnostic(Node):
    def __init__(self):
        super().__init__('odom_diagnostic')
        
        # 订阅里程计和关节状态
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 发布测试命令
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 数据存储
        self.odom_times = deque(maxlen=100)  # 最近100次里程计时间戳
        self.joint_times = deque(maxlen=100)  # 最近100次关节状态时间戳
        self.odom_positions = deque(maxlen=100)  # 最近100次位置
        
        # 统计变量
        self.odom_count = 0
        self.joint_count = 0
        self.last_odom = None
        self.last_joint = None
        self.start_time = time.time()
        
        # 创建定时器
        self.timer = self.create_timer(1.0, self.print_diagnostics)
        
        self.get_logger().info("🔍 里程计诊断工具启动")
        self.get_logger().info("📊 监控频率、延迟和数据质量...")
        
    def odom_callback(self, msg):
        current_time = time.time()
        self.odom_times.append(current_time)
        self.odom_count += 1
        self.last_odom = msg
        
        # 记录位置用于变化检测
        pos = msg.pose.pose.position
        self.odom_positions.append((pos.x, pos.y, current_time))
        
    def joint_callback(self, msg):
        current_time = time.time()
        self.joint_times.append(current_time)
        self.joint_count += 1
        self.last_joint = msg
        
    def calculate_frequency(self, times):
        """计算消息频率"""
        if len(times) < 2:
            return 0.0
        
        # 计算最近10秒的频率
        current_time = time.time()
        recent_times = [t for t in times if current_time - t <= 10.0]
        
        if len(recent_times) < 2:
            return 0.0
            
        time_span = recent_times[-1] - recent_times[0]
        if time_span > 0:
            return (len(recent_times) - 1) / time_span
        return 0.0
    
    def check_position_updates(self):
        """检查位置是否在更新"""
        if len(self.odom_positions) < 2:
            return "无数据"
            
        # 检查最近的位置变化
        recent_positions = list(self.odom_positions)[-10:]  # 最近10个位置
        
        if len(recent_positions) < 2:
            return "数据不足"
        
        # 计算位置变化
        first_pos = recent_positions[0]
        last_pos = recent_positions[-1]
        
        dx = last_pos[0] - first_pos[0]
        dy = last_pos[1] - first_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        time_span = last_pos[2] - first_pos[2]
        
        if distance < 1e-6:
            return f"静止 (最近{time_span:.1f}s无位置变化)"
        else:
            speed = distance / time_span if time_span > 0 else 0
            return f"运动中 (速度: {speed:.3f}m/s)"
    
    def print_diagnostics(self):
        """打印诊断信息"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        print("\n" + "="*60)
        print(f"🔍 里程计诊断报告 (运行时间: {elapsed:.1f}s)")
        print("="*60)
        
        # 频率统计
        odom_freq = self.calculate_frequency(self.odom_times)
        joint_freq = self.calculate_frequency(self.joint_times)
        
        print(f"📊 消息频率:")
        print(f"  里程计: {odom_freq:.1f} Hz (总计: {self.odom_count})")
        print(f"  关节状态: {joint_freq:.1f} Hz (总计: {self.joint_count})")
        
        # 数据延迟检查
        if self.last_odom:
            odom_age = current_time - (self.odom_times[-1] if self.odom_times else current_time)
            print(f"📡 数据新鲜度:")
            print(f"  里程计延迟: {odom_age:.3f}s")
            
            if odom_age > 1.0:
                print("  ⚠️  里程计数据过时！")
            elif odom_age > 0.1:
                print("  ⚠️  里程计更新较慢")
            else:
                print("  ✅ 里程计数据新鲜")
        
        # 位置数据分析
        if self.last_odom:
            pos = self.last_odom.pose.pose.position
            vel = self.last_odom.twist.twist
            print(f"📍 当前状态:")
            print(f"  位置: x={pos.x:.3f}m, y={pos.y:.3f}m")
            print(f"  速度: v={vel.linear.x:.3f}m/s, ω={vel.angular.z:.3f}rad/s")
            
            position_status = self.check_position_updates()
            print(f"  运动状态: {position_status}")
        
        # 关节状态分析
        if self.last_joint and len(self.last_joint.position) >= 2:
            print(f"⚙️  关节状态:")
            print(f"  左轮位置: {self.last_joint.position[0]:.3f}rad")
            print(f"  右轮位置: {self.last_joint.position[1]:.3f}rad")
            
            if len(self.last_joint.velocity) >= 2:
                print(f"  左轮速度: {self.last_joint.velocity[0]:.3f}rad/s")
                print(f"  右轮速度: {self.last_joint.velocity[1]:.3f}rad/s")
        
        # 健康度评估
        print(f"🏥 系统健康度:")
        health_issues = []
        
        if odom_freq < 10:
            health_issues.append("里程计频率过低")
        if joint_freq < 10:
            health_issues.append("关节状态频率过低")
        if self.odom_times and (current_time - self.odom_times[-1]) > 1.0:
            health_issues.append("里程计数据过时")
            
        if health_issues:
            print(f"  ❌ 发现问题: {', '.join(health_issues)}")
            print("  💡 建议:")
            print("    1. 检查硬件接口是否正常运行")
            print("    2. 确认控制器状态: ros2 control list_controllers")
            print("    3. 查看节点日志是否有错误信息")
        else:
            print("  ✅ 系统运行正常")
    
    def send_test_command(self, linear=0.1, angular=0.0, duration=3.0):
        """发送测试命令"""
        print(f"\n🚀 发送测试命令: 线速度={linear}m/s, 角速度={angular}rad/s, 持续{duration}s")
        
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10Hz
        
        while time.time() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            
        # 停止
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        print("✅ 测试命令发送完成")

def main():
    rclpy.init()
    
    diagnostic = OdomDiagnostic()
    
    print("🤖 里程计诊断工具")
    print("📊 实时监控里程计和关节状态...")
    print("💡 提示: 您可以在其他终端发送运动命令来测试响应")
    print("   例如: ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.1}}' --once")
    print("\n按 Ctrl+C 停止监控")
    
    try:
        # 运行5秒基础监控
        for i in range(5):
            rclpy.spin_once(diagnostic, timeout_sec=1.0)
            
        # 可选：发送测试命令
        print("\n❓ 是否发送测试命令来验证响应? (程序将自动发送前进命令)")
        print("   如果您想手动测试，请在另一个终端发送命令")
        
        # 继续监控
        rclpy.spin(diagnostic)
        
    except KeyboardInterrupt:
        print("\n🛑 诊断工具停止")
    finally:
        # 确保停止机器人
        cmd = Twist()
        diagnostic.cmd_pub.publish(cmd)
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
