#!/usr/bin/env python3
"""
串口测试工具 - 用于诊断串口通信问题
"""

import serial
import time
import sys
import argparse

def test_serial_connection(port, baudrate, timeout=1.0):
    """测试串口连接和数据接收"""
    try:
        print(f"🔌 尝试连接串口: {port}, 波特率: {baudrate}")
        
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        
        print(f"✅ 串口连接成功!")
        print(f"📊 串口配置: {ser}")
        print(f"⏰ 等待数据... (按Ctrl+C退出)")
        print("-" * 50)
        
        data_count = 0
        start_time = time.time()
        
        while True:
            try:
                # 读取一行数据
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    data_count += 1
                    current_time = time.time() - start_time
                    
                    print(f"[{current_time:6.2f}s] 数据 #{data_count}: {line}")
                    
                    # 尝试解析数据格式
                    try:
                        parts = line.split(',')
                        if len(parts) >= 4:
                            left_enc = int(parts[0])
                            right_enc = int(parts[1]) 
                            left_rpm = float(parts[2])
                            right_rpm = float(parts[3])
                            
                            print(f"    📈 解析成功: 左编码器={left_enc}, 右编码器={right_enc}")
                            print(f"    🔄 转速: 左轮={left_rpm:.2f}RPM, 右轮={right_rpm:.2f}RPM")
                        else:
                            print(f"    ⚠️  数据格式错误: 期望4个字段，得到{len(parts)}个")
                    except Exception as e:
                        print(f"    ❌ 解析失败: {e}")
                    
                    print()
                else:
                    # 没有数据，短暂等待
                    time.sleep(0.01)
                    
            except KeyboardInterrupt:
                print(f"\n🛑 用户中断")
                break
            except Exception as e:
                print(f"❌ 读取数据错误: {e}")
                time.sleep(0.1)
                
        print(f"\n📊 统计信息:")
        print(f"   总接收数据: {data_count} 条")
        print(f"   运行时间: {time.time() - start_time:.2f} 秒")
        
        ser.close()
        print("🔌 串口已关闭")
        
    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        print("\n🔧 可能的解决方案:")
        print("   1. 检查设备是否已连接: ls -la /dev/ttyCH341USB0")
        print("   2. 检查用户权限: groups $USER")
        print("   3. 添加用户到dialout组: sudo usermod -a -G dialout $USER")
        print("   4. 检查设备权限: sudo chmod 666 /dev/ttyCH341USB0")
        return False
    except Exception as e:
        print(f"❌ 未知错误: {e}")
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(description='串口通信测试工具')
    parser.add_argument('--port', default='/dev/ttyCH341USB0', help='串口设备路径')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--timeout', type=float, default=1.0, help='读取超时 (秒)')
    
    args = parser.parse_args()
    
    print("🚀 串口测试工具启动")
    print("=" * 50)
    
    success = test_serial_connection(args.port, args.baudrate, args.timeout)
    
    if not success:
        sys.exit(1)

if __name__ == '__main__':
    main()
