#!/usr/bin/env python3
"""
原始串口数据测试脚本
直接读取串口的所有数据，不进行任何解析
用于调试串口连接和查看STM32发送的原始数据
"""

import serial
import time
import sys


def test_raw_serial():
    """测试原始串口数据接收"""
    serial_port = "/dev/ttyTHS1"
    baud_rate = 115200
    
    print(f"开始测试串口: {serial_port}, 波特率: {baud_rate}")
    print("按 Ctrl+C 停止测试")
    print("-" * 50)
    
    try:
        # 打开串口
        ser = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            timeout=1.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        print(f"串口 {serial_port} 打开成功")
        print("等待数据...")
        print()
        
        line_count = 0
        
        while True:
            # 尝试读取一行数据
            if ser.in_waiting > 0:
                try:
                    # 读取原始字节数据
                    raw_data = ser.readline()
                    
                    if raw_data:
                        line_count += 1
                        
                        # 尝试解码为字符串
                        try:
                            decoded_data = raw_data.decode('utf-8').strip()
                            print(f"[{line_count:04d}] 收到数据: {decoded_data}")
                            
                            # 尝试解析数据
                            if decoded_data:
                                parts = decoded_data.split(',')
                                print(f"       分割后: {len(parts)} 个部分: {parts}")
                                
                                # 如果是7个部分，尝试解析为STM32格式
                                if len(parts) == 7:
                                    try:
                                        timestamp = int(parts[0])
                                        pos_x = float(parts[1])
                                        pos_y = float(parts[2])
                                        yaw = float(parts[3])
                                        vel_x = float(parts[4])
                                        vel_y = float(parts[5])
                                        omega = float(parts[6])
                                        
                                        print(f"       解析成功:")
                                        print(f"         时间戳: {timestamp} ms")
                                        print(f"         位置: ({pos_x:.6f}, {pos_y:.6f}) m")
                                        print(f"         偏航角: {yaw:.6f} rad")
                                        print(f"         速度: ({vel_x:.6f}, {vel_y:.6f}) m/s")
                                        print(f"         角速度: {omega:.6f} rad/s")
                                        
                                    except ValueError as e:
                                        print(f"       数值解析失败: {e}")
                                else:
                                    print(f"       数据格式不匹配 (期望7个值，收到{len(parts)}个)")
                                        
                        except UnicodeDecodeError as e:
                            print(f"[{line_count:04d}] 解码失败: {e}")
                            print(f"       原始字节: {raw_data}")
                            
                        print()  # 空行分隔
                        
                except Exception as e:
                    print(f"读取数据时出错: {e}")
            else:
                time.sleep(0.01)  # 短暂等待
                
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        return False
    except KeyboardInterrupt:
        print("\n测试停止")
        return True
    except Exception as e:
        print(f"未知错误: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == '__main__':
    test_raw_serial()
