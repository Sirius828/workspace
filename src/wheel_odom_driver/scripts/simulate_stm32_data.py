#!/usr/bin/env python3
"""
轮式里程计数据模拟器
用于测试时模拟STM32发送的数据
"""

import serial
import time
import math

def simulate_odom_data():
    """模拟里程计数据"""
    try:
        # 连接到串口 (这里模拟连接到STM32)
        # 实际使用时，这部分代码应该在STM32上
        ser = serial.Serial('/dev/ttyTHS0', 115200, timeout=1.0)
        
        # 模拟数据
        x, y, yaw = 0.0, 0.0, 0.0
        vx, vy, vyaw = 0.1, 0.0, 0.0  # 模拟向前运动
        
        print("开始发送模拟里程计数据...")
        print("格式: x位移,y位移,偏航角,x速度,y速度,角速度")
        
        while True:
            # 更新位置 (简单的运动模型)
            dt = 0.1  # 100ms
            x += vx * dt * math.cos(yaw) - vy * dt * math.sin(yaw)
            y += vx * dt * math.sin(yaw) + vy * dt * math.cos(yaw)
            yaw += vyaw * dt
            
            # 构造数据字符串
            data = f"{x:.3f},{y:.3f},{yaw:.3f},{vx:.3f},{vy:.3f},{vyaw:.3f}\n"
            
            # 发送数据
            ser.write(data.encode('utf-8'))
            print(f"发送: {data.strip()}")
            
            time.sleep(0.1)  # 10Hz
            
    except KeyboardInterrupt:
        print("\n停止发送数据")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    simulate_odom_data()
