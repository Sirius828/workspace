#!/usr/bin/env python3
"""
模拟STM32发送数据的脚本
用于测试串口通信和里程计节点
"""

import serial
import time
import math


def simulate_stm32_odom_data():
    """模拟STM32发送里程计数据"""
    print("开始模拟STM32数据发送...")
    print("格式: 时间戳,x位移,y位移,偏航角,x速度,y速度,角速度")
    print("按 Ctrl+C 停止")
    print("-" * 50)
    
    # 模拟运动参数
    x, y, yaw = 0.0, 0.0, 0.0
    vx, vy, vyaw = 0.1, 0.0, 0.05  # 向前运动并缓慢转向
    
    start_time = time.time()
    
    try:
        # 连接到串口
        ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1.0)
        print(f"串口连接成功: {ser.name}")
        print()
        
        count = 0
        
        while True:
            current_time = time.time()
            dt = 0.1  # 100ms
            
            # 更新位置 (简单的运动模型)
            x += vx * dt * math.cos(yaw) - vy * dt * math.sin(yaw)
            y += vx * dt * math.sin(yaw) + vy * dt * math.cos(yaw)
            yaw += vyaw * dt
            
            # 保持偏航角在 [-π, π] 范围内
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
            
            # 计算时间戳 (毫秒)
            timestamp_ms = int((current_time - start_time) * 1000)
            
            # 构造STM32格式的数据字符串
            data = f"{timestamp_ms},{x:.6f},{y:.6f},{yaw:.6f},{vx:.6f},{vy:.6f},{vyaw:.6f}\n"
            
            # 发送数据
            ser.write(data.encode('utf-8'))
            ser.flush()  # 确保数据立即发送
            
            count += 1
            print(f"[{count:04d}] 发送: {data.strip()}")
            
            # 每隔10次显示解析后的数据
            if count % 10 == 0:
                print(f"       当前状态: 位置({x:.3f}, {y:.3f}), 偏航{yaw:.3f}rad ({math.degrees(yaw):.1f}°)")
                print()
            
            time.sleep(0.1)  # 10Hz发送频率
            
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n停止发送数据")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
            print("串口已关闭")


if __name__ == '__main__':
    simulate_stm32_odom_data()
