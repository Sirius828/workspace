#!/usr/bin/env python3
"""
统一串口管理节点
功能：
1. 通过串口从STM32接收9字段数据(时间戳,x位移,y位移,偏航角,x速度,y速度,角速度,云台偏航角,云台俯仰角)
2. 解析并分发数据到不同话题：
   - 底盘里程计数据 -> /wheel/odom
   - 云台角度数据 -> /gimbal/current_angle
3. 订阅控制命令并统一发送到STM32：
   - 底盘控制命令: /cmd_vel
   - 云台角度命令: /gimbal/angle_cmd
4. 统一管理串口，避免多节点冲突
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import serial
import threading
import time
import math
from tf_transformations import quaternion_from_euler


class UnifiedSerialManagerNode(Node):
    def __init__(self):
        super().__init__('unified_serial_manager')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyTHS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # 里程计相关参数
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        
        # 云台相关参数
        self.declare_parameter('publish_gimbal_feedback', True)
        
        # 控制参数
        self.declare_parameter('cmd_vel_timeout', 2.0)  # cmd_vel超时时间(秒)
        self.declare_parameter('gimbal_cmd_timeout', 5.0)  # 云台命令超时时间(秒)
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_gimbal_feedback = self.get_parameter('publish_gimbal_feedback').get_parameter_value().bool_value
        
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.gimbal_cmd_timeout = self.get_parameter('gimbal_cmd_timeout').get_parameter_value().double_value
        
        # === 发布器 ===
        # 里程计发布器
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        
        # 云台当前角度发布器
        if self.publish_gimbal_feedback:
            self.gimbal_angle_pub = self.create_publisher(Vector3, '/gimbal/current_angle', 10)
        
        # TF广播器
        # if self.publish_tf:
        #     self.tf_broadcaster = TransformBroadcaster(self)
        
        # === 订阅器 ===
        # 底盘速度命令订阅器
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 云台角度命令订阅器
        self.gimbal_cmd_sub = self.create_subscription(
            Vector3,
            '/gimbal/angle_cmd',
            self.gimbal_cmd_callback,
            10
        )
        
        # === 数据存储 ===
        # 里程计数据
        self.odom_data = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vyaw': 0.0,
            'timestamp': None
        }
        
        # 云台角度数据
        self.gimbal_data = {
            'yaw': 0.0,
            'pitch': 0.0,
            'timestamp': None
        }
        
        # 控制命令数据
        self.cmd_vel_data = {
            'vx': 0.0,
            'vy': 0.0,
            'omega': 0.0,
            'timestamp': None
        }
        
        self.gimbal_cmd_data = {
            'yaw': 0.0,
            'pitch': 0.0,
            'timestamp': None
        }
        
        # === 串口相关 ===
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        
        # 初始化串口
        self.init_serial()
        
        # 启动串口读取线程
        self.serial_read_thread = threading.Thread(target=self.read_serial_data)
        self.serial_read_thread.daemon = True
        self.serial_read_thread.start()
        
        # 创建定时器
        self.publish_timer = self.create_timer(0.02, self.publish_data)  # 50Hz发布数据
        self.send_cmd_timer = self.create_timer(0.05, self.send_commands)  # 20Hz发送命令
        self.timeout_check_timer = self.create_timer(0.1, self.check_timeouts)  # 10Hz检查超时
        
        self.get_logger().info(f'统一串口管理节点已启动，串口: {self.serial_port}，波特率: {self.baud_rate}')
    
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
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.serial_port}: {e}')
            self.serial_conn = None
    
    def read_serial_data(self):
        """读取串口数据的线程函数"""
        while rclpy.ok():
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    # 读取一行数据
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.parse_received_data(line)
                except serial.SerialException as e:
                    self.get_logger().error(f'串口读取错误: {e}')
                    self.reconnect_serial()
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f'数据解码错误: {e}')
                except Exception as e:
                    self.get_logger().error(f'读取串口数据时发生未知错误: {e}')
            else:
                time.sleep(0.1)
                # 尝试重新连接串口
                if not self.serial_conn or not self.serial_conn.is_open:
                    self.reconnect_serial()
    
    def reconnect_serial(self):
        """重新连接串口"""
        try:
            if self.serial_conn:
                self.serial_conn.close()
            time.sleep(1.0)  # 等待1秒再重连
            self.init_serial()
        except Exception as e:
            self.get_logger().error(f'重连串口失败: {e}')
    
    def parse_received_data(self, data_str):
        """解析从STM32接收的数据
        期望格式: "时间戳,x位移,y位移,偏航角,x速度,y速度,角速度,云台偏航角,云台俯仰角"
        """
        try:
            # 分割数据
            parts = data_str.split(',')
            if len(parts) != 9:
                self.get_logger().warn(f'数据格式错误，期望9个值，收到{len(parts)}个: {data_str}')
                return
            
            current_time = self.get_clock().now()
            
            # 解析里程计数据
            timestamp_ms = int(parts[0])    # STM32时间戳
            self.odom_data['x'] = float(parts[1])        # x位移 (m)
            self.odom_data['y'] = float(parts[2])        # y位移 (m)
            self.odom_data['yaw'] = float(parts[3])      # 偏航角 (rad)
            self.odom_data['vx'] = float(parts[4])       # x速度 (m/s)
            self.odom_data['vy'] = float(parts[5])       # y速度 (m/s)
            self.odom_data['vyaw'] = float(parts[6])     # 角速度 (rad/s)
            self.odom_data['timestamp'] = current_time
            
            # 解析云台角度数据
            self.gimbal_data['yaw'] = float(parts[7])    # 云台偏航角 (度)
            self.gimbal_data['pitch'] = float(parts[8])  # 云台俯仰角 (度)
            self.gimbal_data['timestamp'] = current_time
            
            # 调试输出 (可选)
            # self.get_logger().debug(f'解析成功: 里程计({self.odom_data[\"x\"]:.3f}, {self.odom_data[\"y\"]:.3f}), 云台({self.gimbal_data[\"yaw\"]:.1f}°, {self.gimbal_data[\"pitch\"]:.1f}°)')
            
        except ValueError as e:
            self.get_logger().warn(f'数据解析错误: {e}, 数据: {data_str}')
        except Exception as e:
            self.get_logger().error(f'解析接收数据时发生错误: {e}')
    
    def cmd_vel_callback(self, msg):
        """底盘速度命令回调函数"""
        current_time = self.get_clock().now()
        self.cmd_vel_data['vx'] = msg.linear.x
        self.cmd_vel_data['vy'] = msg.linear.y
        self.cmd_vel_data['omega'] = msg.angular.z
        self.cmd_vel_data['timestamp'] = current_time
        
        # self.get_logger().debug(f'收到cmd_vel: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, omega={msg.angular.z:.3f}')
    
    def gimbal_cmd_callback(self, msg):
        """云台角度命令回调函数"""
        current_time = self.get_clock().now()
        self.gimbal_cmd_data['yaw'] = msg.x    # 偏航角(度)
        self.gimbal_cmd_data['pitch'] = msg.y  # 俯仰角(度)
        self.gimbal_cmd_data['timestamp'] = current_time
        
        # self.get_logger().debug(f'收到gimbal_cmd: yaw={msg.x:.1f}°, pitch={msg.y:.1f}°')
    
    def check_timeouts(self):
        """检查命令超时"""
        current_time = self.get_clock().now()
        
        # 检查cmd_vel超时
        if (self.cmd_vel_data['timestamp'] and 
            (current_time - self.cmd_vel_data['timestamp']).nanoseconds / 1e9 > self.cmd_vel_timeout):
            # 超时，设置为零
            self.cmd_vel_data['vx'] = 0.0
            self.cmd_vel_data['vy'] = 0.0
            self.cmd_vel_data['omega'] = 0.0
            self.get_logger().warn('cmd_vel超时，已停止底盘运动')
        
        # 云台命令通常不需要超时停止，因为它是位置控制而非速度控制
        # 但可以根据需要添加超时逻辑
    
    def send_commands(self):
        """发送控制命令到STM32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            # 获取当前命令数据
            vx = self.cmd_vel_data['vx']
            vy = self.cmd_vel_data['vy'] 
            omega = self.cmd_vel_data['omega']
            
            gimbal_yaw = self.gimbal_cmd_data['yaw']
            gimbal_pitch = self.gimbal_cmd_data['pitch']
            
            # 构建发送数据格式："vx,vy,omega,gimbal_yaw,gimbal_pitch"
            cmd_str = f"{vx:.3f},{vy:.3f},{omega:.3f},{gimbal_yaw:.1f},{gimbal_pitch:.1f}\n"
            
            # 发送数据
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write(cmd_str.encode('utf-8'))
                    # self.get_logger().debug(f'发送命令: {cmd_str.strip()}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'串口发送错误: {e}')
            self.reconnect_serial()
        except Exception as e:
            self.get_logger().error(f'发送命令时发生错误: {e}')
    
    def publish_data(self):
        """发布数据到ROS话题"""
        current_time = self.get_clock().now()
        
        # 发布里程计数据
        if self.odom_data['timestamp']:
            self.publish_odom(current_time)
        
        # 发布云台角度数据
        if self.publish_gimbal_feedback and self.gimbal_data['timestamp']:
            self.publish_gimbal_angle(current_time)
    
    def publish_odom(self, current_time):
        """发布里程计消息"""
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # 位置信息
        odom_msg.pose.pose.position.x = self.odom_data['x']
        odom_msg.pose.pose.position.y = self.odom_data['y']
        odom_msg.pose.pose.position.z = 0.0
        
        # 四元数表示的偏航角
        quat = quaternion_from_euler(0, 0, self.odom_data['yaw'])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # 速度信息
        odom_msg.twist.twist.linear.x = self.odom_data['vx']
        odom_msg.twist.twist.linear.y = self.odom_data['vy']
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.odom_data['vyaw']
        
        # 设置协方差矩阵
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36
        
        # 位置协方差 (x, y, yaw)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        # 速度协方差 (vx, vy, vyaw)
        odom_msg.twist.covariance[0] = 0.01   # vx
        odom_msg.twist.covariance[7] = 0.01   # vy
        odom_msg.twist.covariance[35] = 0.01  # vyaw
        
        # 发布里程计消息
        self.odom_pub.publish(odom_msg)
        
        # 发布TF变换
        # if self.publish_tf:
        #     self.publish_transform(current_time)
    
    def publish_gimbal_angle(self, current_time):
        """发布云台当前角度"""
        angle_msg = Vector3()
        angle_msg.x = self.gimbal_data['yaw']     # 偏航角(度)
        angle_msg.y = self.gimbal_data['pitch']   # 俯仰角(度)
        angle_msg.z = 0.0                         # 滚转角(暂未使用)
        
        self.gimbal_angle_pub.publish(angle_msg)
    
    def publish_transform(self, current_time):
        """发布TF变换"""
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        # 位置
        transform.transform.translation.x = self.odom_data['x']
        transform.transform.translation.y = self.odom_data['y']
        transform.transform.translation.z = 0.0
        
        # 旋转
        quat = quaternion_from_euler(0, 0, self.odom_data['yaw'])
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        # 广播变换
        self.tf_broadcaster.sendTransform(transform)
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        serial_manager_node = UnifiedSerialManagerNode()
        rclpy.spin(serial_manager_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'serial_manager_node' in locals():
            serial_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
