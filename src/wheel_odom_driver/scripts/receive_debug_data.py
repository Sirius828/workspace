#!/usr/bin/env python3
"""
串口数据接收调试脚本
接收串口发送的所有数据，不做任何处理直接打印
用于调试云台俯仰角PID参数
"""

import serial
import time
import sys


def receive_serial_data():
    """接收并打印串口数据"""
    print("串口数据接收调试工具")
    print("=" * 50)
    print("用途: 调试云台俯仰角PID参数")
    print("串口: /dev/ttyTHS1")
    print("波特率: 115200")
    print("按 Ctrl+C 停止")
    print("=" * 50)
    
    try:
        # 连接到串口
        ser = serial.Serial(
            port='/dev/ttyTHS1',
            baudrate=115200,
            timeout=1.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        print(f"串口 {ser.name} 连接成功")
        print("开始接收数据...\n")
        
        line_count = 0
        start_time = time.time()
        
        while True:
            try:
                # 读取一行数据
                if ser.in_waiting > 0:
                    # 读取原始字节数据
                    raw_data = ser.readline()
                    
                    if raw_data:
                        line_count += 1
                        current_time = time.time()
                        elapsed_time = current_time - start_time
                        
                        # 尝试解码为字符串
                        try:
                            decoded_data = raw_data.decode('utf-8').strip()
                            
                            # 打印时间戳、行号和数据
                            print(f"[{elapsed_time:8.3f}s] [{line_count:04d}] {decoded_data}")
                            
                        except UnicodeDecodeError:
                            # 如果解码失败，打印原始字节数据
                            print(f"[{elapsed_time:8.3f}s] [{line_count:04d}] RAW: {raw_data}")
                            
                else:
                    # 短暂等待，避免CPU占用过高
                    time.sleep(0.001)
                    
            except serial.SerialException as e:
                print(f"\n串口错误: {e}")
                print("尝试重新连接...")
                time.sleep(1.0)
                
                # 尝试重新连接
                try:
                    ser.close()
                    ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1.0)
                    print("重新连接成功")
                except:
                    print("重新连接失败")
                    break
                    
            except Exception as e:
                print(f"\n其他错误: {e}")
                break
                
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        print("请检查:")
        print("1. 串口设备是否存在: ls -l /dev/ttyTHS*")
        print("2. 串口权限: sudo chmod 666 /dev/ttyTHS1")
        print("3. 是否有其他程序占用串口")
        return False
        
    except KeyboardInterrupt:
        print(f"\n\n接收停止")
        print(f"总共接收了 {line_count} 行数据")
        return True
        
    except Exception as e:
        print(f"\n未知错误: {e}")
        return False
        
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")


def main():
    """主函数"""
    print("云台PID调试 - 串口数据接收器")
    
    # 检查串口设备
    import os
    if not os.path.exists('/dev/ttyTHS1'):
        print("错误: /dev/ttyTHS1 设备不存在")
        print("可用的串口设备:")
        os.system("ls -l /dev/ttyTHS* 2>/dev/null || echo '没有找到ttyTHS设备'")
        os.system("ls -l /dev/ttyUSB* 2>/dev/null || echo '没有找到ttyUSB设备'")
        os.system("ls -l /dev/ttyACM* 2>/dev/null || echo '没有找到ttyACM设备'")
        return
    
    # 开始接收数据
    success = receive_serial_data()
    
    if success:
        print("数据接收完成")
    else:
        print("数据接收异常结束")


if __name__ == '__main__':
    main()
