#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        # 参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic_name', '/gimbal_joint_states'),
                ('yaw_joint_name', 'gimbal_yaw_joint'),
                ('pitch_joint_name', 'gimbal_pitch_joint'),
                ('serial_port', '/dev/ttyCH341USB0'),
                ('baudrate', 115200),
            ]
        )
        p = self.get_parameters([
            'topic_name', 'yaw_joint_name', 'pitch_joint_name', 'serial_port', 'baudrate'
        ])
        self.topic_name   = p[0].value
        self.yaw_name     = p[1].value
        self.pitch_name   = p[2].value
        self.serial_port  = p[3].value
        self.baudrate     = p[4].value

        # 串口初始化
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Opened serial port {self.serial_port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port: {e}')
            self.ser = None

        # 订阅关节状态
        self.create_subscription(
            JointState,
            self.topic_name,
            self.joint_state_callback,
            10
        )

        # Publisher for previewing serial data with reliable QoS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.serial_preview_pub = self.create_publisher(
            String,
            'gimbal_serial_preview',
            qos
        )

        # Initialize latest joint angles
        self.latest_yaw = None
        self.latest_pitch = None
        
        # Declare send frequency parameter
        self.declare_parameter('send_frequency', 600.0)
        self.send_frequency = self.get_parameter('send_frequency').value
        
        # Timer for status logging (reduce frequency to avoid spam)
        self.status_timer = self.create_timer(5.0, self.log_status)

        # Timer-driven send at configured frequency
        self.send_timer = self.create_timer(1.0 / self.send_frequency, self.send_serial_data)

    def joint_state_callback(self, msg: JointState):
        # 找到 yaw/pitch 索引
        try:
            iy = msg.name.index(self.yaw_name)
            ip = msg.name.index(self.pitch_name)
        except ValueError:
            self.get_logger().warn(f'Joint names {self.yaw_name}/{self.pitch_name} not found in message: {msg.name}')
            return

        # 取弧度→度
        yaw_rad   = msg.position[iy]
        pitch_rad = msg.position[ip]
        yaw_deg   = math.degrees(yaw_rad)
        pitch_deg = math.degrees(pitch_rad)

        # Store latest angles
        self.latest_yaw = -yaw_deg
        self.latest_pitch = pitch_deg

    def send_serial_data(self):
        # Only send if angles are available
        if self.latest_yaw is None or self.latest_pitch is None:
            return
            
        # Clamp angles to allowable ranges
        yaw_to_send   = max(-45.0, min(45.0, self.latest_yaw))
        pitch_to_send = max(-30.0, min(30.0, self.latest_pitch))
        
        # Format output string
        out_str = f'y{yaw_to_send:.2f},p{pitch_to_send:.2f}\r'
        
        # Publish preview
        preview_msg = String()
        preview_msg.data = out_str
        self.serial_preview_pub.publish(preview_msg)

        # Reduce log frequency to avoid spam at 1000Hz
        # Only log every 1000th message (once per second)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 1
            
        if self._log_counter % 1000 == 0:
            self.get_logger().info(f'Preview serial data (1000Hz): "{out_str.strip()}"')
        
        # Send over serial
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(out_str.encode('utf-8'))
                # Only log errors, not every successful send
            except Exception as e:
                self.get_logger().error(f'Failed to write serial: {e}')
        else:
            # Only log every 1000th debug message
            if self._log_counter % 1000 == 0:
                self.get_logger().debug(f'Serial port not available, preview: "{out_str.strip()}"')
            
    def log_status(self):
        """定期记录状态信息"""
        if self.latest_yaw is not None and self.latest_pitch is not None:
            self.get_logger().info(f'Current gimbal angles: Yaw={self.latest_yaw:.2f}°, Pitch={self.latest_pitch:.2f}°')
        else:
            self.get_logger().info('Waiting for gimbal joint state data...')

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
