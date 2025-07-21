#!/usr/bin/env python3
"""
简洁版串口监控脚本
专门用于云台PID调试，实时显示接收到的数据
"""

import serial
import time


def monitor_serial():
    """监控串口数据"""
    try:
        ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)
        print("云台PID调试监控器")
        print(f"串口: {ser.name} @ 115200")
        print("实时数据显示 (Ctrl+C停止):")
        print("-" * 60)
        
        count = 0
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    count += 1
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"{timestamp} [{count:04d}] {line}")
            
    except KeyboardInterrupt:
        print(f"\n监控停止，共接收 {count} 条数据")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'ser' in locals():
            ser.close()


if __name__ == '__main__':
    monitor_serial()
