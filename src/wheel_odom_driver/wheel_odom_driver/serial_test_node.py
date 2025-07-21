#!/usr/bin/env python3
"""
串口测试节点
用于测试和调试串口通信
"""

import rclpy
from rclpy.node import Node
import serial
import threading
import time


class SerialTestNode(Node):
    def __init__(self):
        super().__init__('serial_test_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyTHS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # 串口对象
        self.serial_conn = None
        
        # 初始化串口
        self.init_serial()
        
        # 启动串口读取线程
        if self.serial_conn:
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        
        self.get_logger().info(f'串口测试节点已启动，串口: {self.serial_port}，波特率: {self.baud_rate}')
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.get_logger().info(f'串口 {self.serial_port} 连接成功')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.serial_port}: {e}')
            self.serial_conn = None
            return False
    
    def read_serial_data(self):
        """读取串口数据的线程函数"""
        while rclpy.ok():
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    # 读取一行数据
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f'收到数据: {line}')
                        # 尝试解析数据
                        self.parse_test_data(line)
                except serial.SerialException as e:
                    self.get_logger().error(f'串口读取错误: {e}')
                    break
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f'数据解码错误: {e}')
                except Exception as e:
                    self.get_logger().error(f'读取串口数据时发生未知错误: {e}')
            else:
                time.sleep(0.1)
    
    def parse_test_data(self, data_str):
        """测试解析里程计数据"""
        try:
            # 分割数据
            parts = data_str.split(',')
            if len(parts) == 7:
                timestamp_ms = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                yaw = float(parts[3])
                vx = float(parts[4])
                vy = float(parts[5])
                vyaw = float(parts[6])
                
                self.get_logger().info(
                    f'解析成功 - 时间戳: {timestamp_ms}ms, 位置: ({x:.3f}, {y:.3f}), 偏航角: {yaw:.3f}, '
                    f'速度: ({vx:.3f}, {vy:.3f}), 角速度: {vyaw:.3f}'
                )
            else:
                self.get_logger().warn(f'数据格式错误，期望7个值，收到{len(parts)}个')
                
        except ValueError as e:
            self.get_logger().warn(f'数据解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'解析测试数据时发生错误: {e}')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        serial_test_node = SerialTestNode()
        rclpy.spin(serial_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'serial_test_node' in locals():
            serial_test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
