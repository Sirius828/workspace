#!/usr/bin/env python3
"""
底层串口监控工具 - 直接监控串口数据和编码器转换
不依赖里程计话题，直接查看硬件接口内部的数据处理
"""

import serial
import time
import threading
import math
from collections import deque

class LowLevelMonitor:
    def __init__(self, port="/dev/ttyCH341USB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # 编码器参数（需要与硬件接口一致）
        self.ticks_per_rev = 8192.0
        self.gear_ratio = 36.0
        self.rad_per_tick = 2.0 * math.pi / (self.ticks_per_rev * self.gear_ratio)
        
        # 数据存储
        self.raw_data = deque(maxlen=1000)
        self.encoder_data = deque(maxlen=1000)
        self.position_data = deque(maxlen=1000)
        
        # 编码器状态
        self.last_left_tick = None
        self.last_right_tick = None
        self.left_position = 0.0
        self.right_position = 0.0
        
        print("🔧 底层串口监控工具")
        print(f"📡 串口: {port}, 波特率: {baudrate}")
        print(f"⚙️  编码器参数: {self.ticks_per_rev} ticks/rev, 减速比: {self.gear_ratio}")
        print(f"📐 转换系数: {self.rad_per_tick:.8f} rad/tick")
        
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"✅ 串口连接成功: {self.port}")
            return True
        except Exception as e:
            print(f"❌ 串口连接失败: {e}")
            return False
    
    def parse_serial_data(self, line):
        """解析串口数据"""
        try:
            # 期望格式: "left_encoder,right_encoder,left_rpm,right_rpm"
            parts = line.strip().split(',')
            if len(parts) >= 4:
                left_tick = int(parts[0])
                right_tick = int(parts[1])
                left_rpm = float(parts[2])
                right_rpm = float(parts[3])
                
                return {
                    'timestamp': time.time(),
                    'left_tick': left_tick,
                    'right_tick': right_tick,
                    'left_rpm': left_rpm,
                    'right_rpm': right_rpm,
                    'raw': line.strip()
                }
        except Exception as e:
            print(f"⚠️  数据解析错误: {line.strip()} - {e}")
        
        return None
    
    def calculate_position(self, left_tick, right_tick):
        """计算位置（模拟硬件接口的逻辑）"""
        if self.last_left_tick is None:
            # 首次初始化
            self.last_left_tick = left_tick
            self.last_right_tick = right_tick
            print(f"🎯 编码器初始化: 左={left_tick}, 右={right_tick}")
            return None
        
        # 计算增量
        left_delta = left_tick - self.last_left_tick
        right_delta = right_tick - self.last_right_tick
        
        # 右轮方向修正（根据您的机器人配置）
        right_delta = -right_delta
        
        # 更新位置
        self.left_position += left_delta * self.rad_per_tick
        self.right_position += right_delta * self.rad_per_tick
        
        # 保存当前值
        self.last_left_tick = left_tick
        self.last_right_tick = right_tick
        
        return {
            'left_delta': left_delta,
            'right_delta': right_delta,
            'left_pos': self.left_position,
            'right_pos': self.right_position
        }
    
    def read_serial_data(self):
        """读取串口数据线程"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore')
                    if line:
                        data = self.parse_serial_data(line)
                        if data:
                            self.raw_data.append(data)
                            
                            # 计算位置
                            pos_data = self.calculate_position(
                                data['left_tick'], 
                                data['right_tick']
                            )
                            
                            if pos_data:
                                self.encoder_data.append({
                                    'timestamp': data['timestamp'],
                                    'left_tick': data['left_tick'],
                                    'right_tick': data['right_tick'],
                                    'left_delta': pos_data['left_delta'],
                                    'right_delta': pos_data['right_delta'],
                                    'left_pos': pos_data['left_pos'],
                                    'right_pos': pos_data['right_pos'],
                                    'left_rpm': data['left_rpm'],
                                    'right_rpm': data['right_rpm']
                                })
                
                time.sleep(0.001)  # 1ms延迟
                
            except Exception as e:
                print(f"⚠️  串口读取错误: {e}")
                time.sleep(0.1)
    
    def send_command(self, left_rpm, right_rpm):
        """发送速度命令"""
        if self.serial_conn:
            try:
                command = f"{left_rpm:.2f},{right_rpm:.2f}\n"
                self.serial_conn.write(command.encode())
                print(f"📤 发送命令: {command.strip()}")
            except Exception as e:
                print(f"❌ 命令发送失败: {e}")
    
    def print_status(self):
        """打印当前状态"""
        if not self.encoder_data:
            print("📊 等待编码器数据...")
            return
        
        latest = self.encoder_data[-1]
        
        print(f"\n📊 实时状态 ({time.strftime('%H:%M:%S')})")
        print(f"🔗 串口数据: 左编码器={latest['left_tick']}, 右编码器={latest['right_tick']}")
        print(f"📈 编码器增量: 左={latest['left_delta']}, 右={latest['right_delta']}")
        print(f"📍 累积位置: 左={latest['left_pos']:.4f}rad, 右={latest['right_pos']:.4f}rad")
        print(f"🔄 RPM反馈: 左={latest['left_rpm']:.2f}, 右={latest['right_rpm']:.2f}")
        
        # 计算频率
        if len(self.raw_data) >= 2:
            time_span = self.raw_data[-1]['timestamp'] - self.raw_data[0]['timestamp']
            if time_span > 0:
                frequency = len(self.raw_data) / time_span
                print(f"📡 数据频率: {frequency:.1f} Hz")
    
    def start_monitoring(self):
        """开始监控"""
        if not self.connect_serial():
            return False
        
        self.running = True
        
        # 启动数据读取线程
        read_thread = threading.Thread(target=self.read_serial_data)
        read_thread.daemon = True
        read_thread.start()
        
        print("\n🚀 开始底层监控...")
        print("💡 提示:")
        print("   - 观察编码器数据是否实时更新")
        print("   - 发送测试命令看是否有响应")
        print("   - 按 Ctrl+C 停止监控")
        
        try:
            start_time = time.time()
            last_print_time = start_time
            
            while True:
                current_time = time.time()
                
                # 每秒打印一次状态
                if current_time - last_print_time >= 1.0:
                    self.print_status()
                    last_print_time = current_time
                
                # 每5秒发送一次测试命令
                if int(current_time - start_time) % 10 == 5 and int(current_time - start_time) != int(last_print_time - start_time):
                    print(f"\n🧪 发送测试命令...")
                    self.send_command(20.0, 20.0)  # 前进
                    time.sleep(2)
                    self.send_command(0.0, 0.0)    # 停止
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🛑 监控停止")
        finally:
            self.running = False
            if self.serial_conn:
                self.send_command(0.0, 0.0)  # 确保停止
                self.serial_conn.close()
    
    def analyze_data(self):
        """分析收集的数据"""
        if not self.encoder_data:
            print("❌ 没有数据可分析")
            return
        
        print(f"\n📊 数据分析报告")
        print(f"=" * 50)
        print(f"📈 总数据点: {len(self.encoder_data)}")
        
        if len(self.encoder_data) >= 2:
            first = self.encoder_data[0]
            last = self.encoder_data[-1]
            time_span = last['timestamp'] - first['timestamp']
            
            print(f"⏱️  时间跨度: {time_span:.1f}秒")
            print(f"📡 平均频率: {len(self.encoder_data)/time_span:.1f} Hz")
            
            # 位置变化
            left_change = last['left_pos'] - first['left_pos']
            right_change = last['right_pos'] - first['right_pos']
            
            print(f"📍 位置变化:")
            print(f"   左轮: {left_change:.4f} rad")
            print(f"   右轮: {right_change:.4f} rad")
            
            # 检查是否有运动
            if abs(left_change) > 0.01 or abs(right_change) > 0.01:
                print("✅ 检测到机器人运动")
            else:
                print("⚠️  未检测到明显运动")

def main():
    # 尝试不同的串口
    possible_ports = ["/dev/ttyCH341USB0", "/dev/ttyCH341USB1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    
    monitor = None
    for port in possible_ports:
        print(f"🔍 尝试连接串口: {port}")
        monitor = LowLevelMonitor(port)
        if monitor.connect_serial():
            monitor.serial_conn.close()  # 先关闭，让start_monitoring重新打开
            break
        monitor = None
    
    if not monitor:
        print("❌ 找不到可用的串口，请检查:")
        print("   1. 硬件连接是否正确")
        print("   2. 串口权限: sudo chmod 666 /dev/ttyUSB*")
        print("   3. 是否有其他程序占用串口")
        return
    
    try:
        monitor.start_monitoring()
    finally:
        if monitor:
            monitor.analyze_data()

if __name__ == "__main__":
    main()
