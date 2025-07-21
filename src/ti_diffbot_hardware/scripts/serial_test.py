#!/usr/bin/env python3
"""
串口通信测试工具
用于测试和调试ti_diffbot_hardware的串口通信协议
"""

import serial
import time
import sys
import threading

class SerialTester:
    def __init__(self, port='/dev/ttyCH341USB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1.0
            )
            print(f"✅ 成功连接到串口: {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"❌ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 串口已断开")
    
    def send_command(self, rpm_left, rpm_right):
        """发送速度命令"""
        if not self.ser or not self.ser.is_open:
            print("❌ 串口未连接")
            return False
        
        try:
            command = f"{rpm_left:.2f},{rpm_right:.2f}\n"
            self.ser.write(command.encode('utf-8'))
            print(f"📤 发送: {command.strip()}")
            return True
        except Exception as e:
            print(f"❌ 发送失败: {e}")
            return False
    
    def read_feedback(self):
        """读取反馈数据"""
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    print(f"📥 接收: {line}")
                    return self.parse_feedback(line)
        except Exception as e:
            print(f"❌ 读取失败: {e}")
        return None
    
    def parse_feedback(self, line):
        """解析反馈数据"""
        try:
            parts = line.split(',')
            if len(parts) >= 4:
                data = {
                    'left_encoder': int(parts[0]),
                    'right_encoder': int(parts[1]),
                    'left_rpm': float(parts[2]),
                    'right_rpm': float(parts[3])
                }
                print(f"   📊 编码器: L={data['left_encoder']}, R={data['right_encoder']}")
                print(f"   🔄 转速: L={data['left_rpm']:.2f}RPM, R={data['right_rpm']:.2f}RPM")
                return data
            else:
                print(f"⚠️  数据格式错误，期望4个字段，收到{len(parts)}个")
        except Exception as e:
            print(f"❌ 解析错误: {e}")
        return None
    
    def monitor_mode(self):
        """监听模式 - 持续读取数据"""
        print("🔍 进入监听模式 (按Ctrl+C退出)")
        self.running = True
        
        def read_loop():
            while self.running:
                self.read_feedback()
                time.sleep(0.05)  # 20Hz
        
        read_thread = threading.Thread(target=read_loop)
        read_thread.daemon = True
        read_thread.start()
        
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 退出监听模式")
            self.running = False
    
    def interactive_mode(self):
        """交互模式 - 手动发送命令"""
        print("🎮 进入交互模式")
        print("输入格式: <左轮RPM>,<右轮RPM>")
        print("示例: 10.5,-8.2")
        print("输入 'q' 退出")
        
        while True:
            try:
                user_input = input("➤ 输入命令: ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                if ',' in user_input:
                    parts = user_input.split(',')
                    if len(parts) == 2:
                        try:
                            rpm_left = float(parts[0])
                            rpm_right = float(parts[1])
                            self.send_command(rpm_left, rpm_right)
                            
                            # 读取反馈
                            time.sleep(0.1)
                            self.read_feedback()
                        except ValueError:
                            print("❌ 请输入有效的数字")
                    else:
                        print("❌ 请输入两个数值，用逗号分隔")
                else:
                    print("❌ 请使用逗号分隔左右轮转速")
                    
            except KeyboardInterrupt:
                break
        
        print("👋 退出交互模式")

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyCH341USB0'
    
    tester = SerialTester(port)
    
    if not tester.connect():
        return
    
    try:
        print("\n📋 选择模式:")
        print("1. 监听模式 - 持续接收数据")
        print("2. 交互模式 - 手动发送命令")
        print("3. 测试序列 - 自动测试")
        
        choice = input("请选择 (1/2/3): ").strip()
        
        if choice == '1':
            tester.monitor_mode()
        elif choice == '2':
            tester.interactive_mode()
        elif choice == '3':
            print("🧪 执行测试序列...")
            # 停止
            tester.send_command(0, 0)
            time.sleep(1)
            
            # 前进
            print("⬆️  前进测试")
            tester.send_command(10, 10)
            time.sleep(2)
            tester.read_feedback()
            
            # 左转
            print("⬅️  左转测试")
            tester.send_command(-5, 5)
            time.sleep(2)
            tester.read_feedback()
            
            # 右转
            print("➡️  右转测试")
            tester.send_command(5, -5)
            time.sleep(2)
            tester.read_feedback()
            
            # 停止
            print("⏹️  停止测试")
            tester.send_command(0, 0)
            tester.read_feedback()
            
        else:
            print("❌ 无效选择")
    
    finally:
        tester.disconnect()

if __name__ == "__main__":
    main()
