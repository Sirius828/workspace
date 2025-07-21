#!/usr/bin/env python3
"""
性能对比测试 - 测试优化前后的处理频率差异
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import threading
from collections import deque
import serial

class PerformanceComparison(Node):
    def __init__(self):
        super().__init__('performance_comparison')
        
        # ROS数据订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 数据收集
        self.ros_odom_times = deque(maxlen=1000)
        self.ros_joint_times = deque(maxlen=1000)
        self.serial_data_times = deque(maxlen=1000)
        
        # 串口监控
        self.serial_port = "/dev/ttyCH341USB0"
        self.baudrate = 115200
        self.serial_conn = None
        self.serial_running = False
        
        self.get_logger().info("🔬 性能对比测试工具启动")
        
    def odom_callback(self, msg):
        self.ros_odom_times.append(time.time())
        
    def joint_callback(self, msg):
        self.ros_joint_times.append(time.time())
    
    def start_serial_monitoring(self):
        """启动串口监控线程"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.serial_running = True
            
            thread = threading.Thread(target=self.serial_monitor_thread)
            thread.daemon = True
            thread.start()
            
            print(f"✅ 串口监控启动: {self.serial_port}")
            return True
            
        except Exception as e:
            print(f"❌ 串口连接失败: {e}")
            return False
    
    def serial_monitor_thread(self):
        """串口监控线程"""
        while self.serial_running and self.serial_conn:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line and ',' in line:
                    self.serial_data_times.append(time.time())
            except:
                pass
    
    def stop_serial_monitoring(self):
        """停止串口监控"""
        self.serial_running = False
        if self.serial_conn:
            self.serial_conn.close()
    
    def calculate_frequency(self, times, duration=10.0):
        """计算指定时间内的频率"""
        if len(times) < 2:
            return 0.0
        
        current_time = time.time()
        recent_times = [t for t in times if current_time - t <= duration]
        
        if len(recent_times) < 2:
            return 0.0
        
        time_span = recent_times[-1] - recent_times[0]
        if time_span > 0:
            return (len(recent_times) - 1) / time_span
        return 0.0
    
    def run_performance_test(self, duration=30.0):
        """运行性能测试"""
        print(f"\n🚀 开始性能测试 ({duration}秒)")
        print("="*60)
        
        # 启动串口监控
        if not self.start_serial_monitoring():
            print("❌ 无法启动串口监控，测试中止")
            return
        
        # 清空历史数据
        self.ros_odom_times.clear()
        self.ros_joint_times.clear()
        self.serial_data_times.clear()
        
        print("📊 数据收集中...")
        start_time = time.time()
        
        # 定期显示进度
        last_report = start_time
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            current_time = time.time()
            if current_time - last_report >= 5.0:
                elapsed = current_time - start_time
                progress = (elapsed / duration) * 100
                
                serial_freq = self.calculate_frequency(self.serial_data_times, 5.0)
                ros_odom_freq = self.calculate_frequency(self.ros_odom_times, 5.0)
                
                print(f"⏱️  进度: {progress:.1f}% | 串口: {serial_freq:.1f}Hz | ROS里程计: {ros_odom_freq:.1f}Hz")
                last_report = current_time
        
        # 停止串口监控
        self.stop_serial_monitoring()
        
        # 分析结果
        self.analyze_results(duration)
    
    def analyze_results(self, duration):
        """分析测试结果"""
        print(f"\n📈 性能分析结果 ({duration}秒测试)")
        print("="*60)
        
        # 计算频率
        serial_freq = self.calculate_frequency(self.serial_data_times, duration)
        ros_odom_freq = self.calculate_frequency(self.ros_odom_times, duration)
        ros_joint_freq = self.calculate_frequency(self.ros_joint_times, duration)
        
        print(f"📡 数据源频率:")
        print(f"   串口原始数据: {serial_freq:.1f} Hz")
        print(f"   ROS里程计话题: {ros_odom_freq:.1f} Hz")
        print(f"   ROS关节状态话题: {ros_joint_freq:.1f} Hz")
        
        # 计算处理效率
        if serial_freq > 0:
            odom_efficiency = (ros_odom_freq / serial_freq) * 100
            joint_efficiency = (ros_joint_freq / serial_freq) * 100
            
            print(f"\n⚡ 处理效率:")
            print(f"   里程计处理效率: {odom_efficiency:.1f}%")
            print(f"   关节状态处理效率: {joint_efficiency:.1f}%")
            
            # 性能评估
            print(f"\n🏥 性能评估:")
            if odom_efficiency >= 90:
                print("   ✅ 优秀: 处理效率 >= 90%")
            elif odom_efficiency >= 70:
                print("   ✅ 良好: 处理效率 >= 70%")
            elif odom_efficiency >= 50:
                print("   ⚠️  一般: 处理效率 >= 50%")
            else:
                print("   ❌ 差: 处理效率 < 50%")
                print("   💡 建议优化硬件接口处理逻辑")
        
        # 数据统计
        print(f"\n📊 数据统计:")
        print(f"   串口数据包: {len(self.serial_data_times)}")
        print(f"   里程计消息: {len(self.ros_odom_times)}")
        print(f"   关节状态消息: {len(self.ros_joint_times)}")
        
        # 延迟估算
        if len(self.serial_data_times) > 0 and len(self.ros_odom_times) > 0:
            # 简单估算：假设ROS消息对应最近的串口数据
            avg_serial_interval = duration / len(self.serial_data_times) if len(self.serial_data_times) > 0 else 0
            avg_ros_interval = duration / len(self.ros_odom_times) if len(self.ros_odom_times) > 0 else 0
            
            estimated_delay = avg_ros_interval - avg_serial_interval
            
            print(f"\n⏱️  延迟估算:")
            print(f"   平均串口间隔: {avg_serial_interval*1000:.1f}ms")
            print(f"   平均ROS间隔: {avg_ros_interval*1000:.1f}ms")
            print(f"   估算处理延迟: {estimated_delay*1000:.1f}ms")

def main():
    rclpy.init()
    
    tester = PerformanceComparison()
    
    print("🔬 硬件接口性能对比测试")
    print("📋 测试目标:")
    print("   1. 对比串口数据接收频率 vs ROS话题发布频率")
    print("   2. 计算数据处理效率")
    print("   3. 估算处理延迟")
    print("   4. 为优化提供依据")
    
    try:
        # 等待ROS数据
        print("\n⏳ 等待ROS数据...")
        timeout = 5.0
        start_wait = time.time()
        
        while len(tester.ros_odom_times) < 5 and (time.time() - start_wait < timeout):
            rclpy.spin_once(tester, timeout_sec=0.1)
        
        if len(tester.ros_odom_times) < 5:
            print("❌ ROS数据超时，请确保机器人系统运行正常")
            return
        
        print("✅ ROS数据准备就绪")
        
        # 运行性能测试
        tester.run_performance_test(duration=20.0)
        
        print("\n🎉 性能测试完成！")
        
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试发生错误: {e}")
    finally:
        tester.stop_serial_monitoring()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
