#!/usr/bin/env python3
"""
统一串口管理测试脚本
模拟STM32发送9字段数据到串口管理器
"""

import serial
import time
import math

def main():
    # 串口配置
    port = '/dev/ttyTHS0'
    baud_rate = 115200
    
    try:
        # 打开串口
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f'已连接到串口 {port}，波特率 {baud_rate}')
        
        # 模拟数据
        timestamp = 0
        x, y, yaw = 0.0, 0.0, 0.0
        vx, vy, vyaw = 0.0, 0.0, 0.0
        gimbal_yaw, gimbal_pitch = 0.0, 0.0
        
        print('开始发送测试数据...')
        print('格式: 时间戳,x位移,y位移,偏航角,x速度,y速度,角速度,云台偏航角,云台俯仰角')
        
        while True:
            # 更新时间戳
            timestamp += 20  # 20ms增量
            
            # 模拟底盘运动
            t = timestamp / 1000.0  # 转换为秒
            x = 0.5 * math.sin(t * 0.1)
            y = 0.3 * math.cos(t * 0.1)
            yaw = 0.2 * math.sin(t * 0.05)
            vx = 0.1 * math.sin(t * 0.2)
            vy = 0.05 * math.cos(t * 0.2)
            vyaw = 0.1 * math.sin(t * 0.1)
            
            # 模拟云台运动
            gimbal_yaw = 10.0 * math.sin(t * 0.3)
            gimbal_pitch = 5.0 * math.cos(t * 0.4)
            
            # 构建数据字符串
            data_str = f'{timestamp},{x:.3f},{y:.3f},{yaw:.3f},{vx:.3f},{vy:.3f},{vyaw:.3f},{gimbal_yaw:.1f},{gimbal_pitch:.1f}\\n'
            
            # 发送数据
            ser.write(data_str.encode('utf-8'))
            
            print(f'发送: {data_str.strip()}')
            
            # 等待50ms (20Hz)
            time.sleep(0.05)
            
    except serial.SerialException as e:
        print(f'串口错误: {e}')
    except KeyboardInterrupt:
        print('\\n测试中断')
    finally:
        if 'ser' in locals():
            ser.close()
            print('串口已关闭')

if __name__ == '__main__':
    main()
