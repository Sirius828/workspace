#!/usr/bin/env python3
"""
简化的云台控制节点
只负责发布云台角度命令到 /gimbal/angle_cmd 话题
不再直接操作串口，串口通信由 unified_serial_manager_node 统一管理
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
import time


class SimplifiedGimbalController(Node):
    def __init__(self):
        super().__init__('simplified_gimbal_controller')
        
        # 参数声明
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_horizontal_fov', 53.35)
        self.declare_parameter('camera_vertical_fov', 41.30)
        self.declare_parameter('use_camera_info', True)
        
        # PID参数
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('ki_yaw', 0.1)
        self.declare_parameter('kd_yaw', 0.5)
        self.declare_parameter('kp_pitch', 2.0)
        self.declare_parameter('ki_pitch', 0.1)
        self.declare_parameter('kd_pitch', 0.5)
        
        # 控制参数
        self.declare_parameter('dead_zone_pixels', 10)
        self.declare_parameter('max_angle_step', 2.0)
        self.declare_parameter('yaw_limit_min', -90.0)
        self.declare_parameter('yaw_limit_max', 90.0)
        self.declare_parameter('pitch_limit_min', -30.0)
        self.declare_parameter('pitch_limit_max', 30.0)
        
        # 目标跟踪参数
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('control_frequency', 20.0)
        
        # 获取参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.h_fov = math.radians(self.get_parameter('camera_horizontal_fov').value)
        self.v_fov = math.radians(self.get_parameter('camera_vertical_fov').value)
        self.use_camera_info = self.get_parameter('use_camera_info').value
        
        # PID参数
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value
        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.ki_pitch = self.get_parameter('ki_pitch').value
        self.kd_pitch = self.get_parameter('kd_pitch').value
        
        # 控制参数
        self.dead_zone = self.get_parameter('dead_zone_pixels').value
        self.max_step = self.get_parameter('max_angle_step').value
        self.yaw_min = self.get_parameter('yaw_limit_min').value
        self.yaw_max = self.get_parameter('yaw_limit_max').value
        self.pitch_min = self.get_parameter('pitch_limit_min').value
        self.pitch_max = self.get_parameter('pitch_limit_max').value
        
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # 云台状态
        self.target_yaw = 0.0      # 目标绝对角度
        self.target_pitch = 0.0    # 目标绝对角度
        self.current_yaw = 0.0     # 当前实际角度（从串口管理器反馈）
        self.current_pitch = 0.0   # 当前实际角度（从串口管理器反馈）
        
        # PID控制状态
        self.last_error_yaw = 0.0
        self.last_error_pitch = 0.0
        self.integral_yaw = 0.0
        self.integral_pitch = 0.0
        self.last_time = self.get_clock().now()
        
        # 图像中心
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # === 发布器 ===
        # 云台角度命令发布器（发送到统一串口管理器）
        self.angle_cmd_pub = self.create_publisher(Vector3, '/gimbal/angle_cmd', 10)
        
        # === 订阅器 ===
        # 目标像素位置订阅器
        self.target_pixel_sub = self.create_subscription(
            Point,
            '/target_pixel',
            self.target_pixel_callback,
            10
        )
        
        # 直接角度控制订阅器
        self.direct_angle_sub = self.create_subscription(
            Vector3,
            '/gimbal/direct_angle',
            self.direct_angle_callback,
            10
        )
        
        # 云台当前角度反馈订阅器
        self.current_angle_sub = self.create_subscription(
            Vector3,
            '/gimbal/current_angle',
            self.current_angle_callback,
            10
        )
        
        # 相机信息订阅器（用于动态更新FOV）
        if self.use_camera_info:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/camera/camera_info',
                self.camera_info_callback,
                10
            )
        
        # 创建控制定时器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # 状态变量
        self.last_target_time = None
        self.control_active = False
        
        self.get_logger().info(f'简化云台控制节点已启动，控制频率: {self.control_frequency}Hz')
        self.get_logger().info(f'图像尺寸: {self.image_width}x{self.image_height}, FOV: {math.degrees(self.h_fov):.1f}°x{math.degrees(self.v_fov):.1f}°')
        self.get_logger().info(f'角度限制: 偏航[{self.yaw_min}°, {self.yaw_max}°], 俯仰[{self.pitch_min}°, {self.pitch_max}°]')
    
    def camera_info_callback(self, msg):
        """相机信息回调，动态更新FOV"""
        if self.use_camera_info:
            self.image_width = msg.width
            self.image_height = msg.height
            self.image_center_x = self.image_width // 2
            self.image_center_y = self.image_height // 2
            
            # 从相机内参计算FOV（如果有）
            if msg.k[0] > 0 and msg.k[4] > 0:  # fx和fy
                fx, fy = msg.k[0], msg.k[4]
                self.h_fov = 2 * math.atan(self.image_width / (2 * fx))
                self.v_fov = 2 * math.atan(self.image_height / (2 * fy))
                
                self.get_logger().debug(f'从相机内参更新FOV: {math.degrees(self.h_fov):.1f}°x{math.degrees(self.v_fov):.1f}°')
    
    def target_pixel_callback(self, msg):
        """目标像素位置回调"""
        if not self.tracking_enabled:
            return
        
        target_x = msg.x
        target_y = msg.y
        
        # 记录目标时间
        self.last_target_time = self.get_clock().now()
        
        # 计算像素偏差
        pixel_error_x = target_x - self.image_center_x
        pixel_error_y = target_y - self.image_center_y
        
        # 检查死区
        if abs(pixel_error_x) < self.dead_zone and abs(pixel_error_y) < self.dead_zone:
            # 在死区内，不需要调整
            return
        
        # 将像素偏差转换为角度偏差
        angle_error_yaw = self.pixel_to_angle_horizontal(pixel_error_x)
        angle_error_pitch = -self.pixel_to_angle_vertical(pixel_error_y)  # 注意方向
        
        # 使用PID控制计算目标角度调整
        delta_yaw, delta_pitch = self.compute_pid_control(angle_error_yaw, angle_error_pitch)
        
        # 更新目标角度
        new_target_yaw = self.target_yaw + delta_yaw
        new_target_pitch = self.target_pitch + delta_pitch
        
        # 应用角度限制
        self.target_yaw = self.clamp_angle(new_target_yaw, self.yaw_min, self.yaw_max)
        self.target_pitch = self.clamp_angle(new_target_pitch, self.pitch_min, self.pitch_max)
        
        # 激活控制
        self.control_active = True
        
        self.get_logger().debug(f'目标像素({target_x:.0f}, {target_y:.0f}) -> 角度偏差({math.degrees(angle_error_yaw):.1f}°, {math.degrees(angle_error_pitch):.1f}°) -> 目标角度({self.target_yaw:.1f}°, {self.target_pitch:.1f}°)')
    
    def direct_angle_callback(self, msg):
        """直接角度控制回调"""
        # 直接设置目标角度
        self.target_yaw = self.clamp_angle(msg.x, self.yaw_min, self.yaw_max)
        self.target_pitch = self.clamp_angle(msg.y, self.pitch_min, self.pitch_max)
        
        # 激活控制
        self.control_active = True
        
        self.get_logger().info(f'直接角度控制: 目标角度({self.target_yaw:.1f}°, {self.target_pitch:.1f}°)')
    
    def current_angle_callback(self, msg):
        """当前角度反馈回调"""
        self.current_yaw = msg.x
        self.current_pitch = msg.y
        
        # self.get_logger().debug(f'当前角度: ({self.current_yaw:.1f}°, {self.current_pitch:.1f}°)')
    
    def pixel_to_angle_horizontal(self, pixel_offset):
        """将水平像素偏移转换为角度"""
        angle_per_pixel = self.h_fov / self.image_width
        return pixel_offset * angle_per_pixel
    
    def pixel_to_angle_vertical(self, pixel_offset):
        """将垂直像素偏移转换为角度"""
        angle_per_pixel = self.v_fov / self.image_height
        return pixel_offset * angle_per_pixel
    
    def compute_pid_control(self, error_yaw, error_pitch):
        """计算PID控制输出"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return 0.0, 0.0
        
        # PID控制 - 偏航
        self.integral_yaw += error_yaw * dt
        # 限制积分项防止积分饱和
        self.integral_yaw = max(-1.0, min(1.0, self.integral_yaw))
        
        derivative_yaw = (error_yaw - self.last_error_yaw) / dt
        output_yaw = (self.kp_yaw * error_yaw + 
                     self.ki_yaw * self.integral_yaw + 
                     self.kd_yaw * derivative_yaw)
        
        # PID控制 - 俯仰
        self.integral_pitch += error_pitch * dt
        # 限制积分项防止积分饱和
        self.integral_pitch = max(-1.0, min(1.0, self.integral_pitch))
        
        derivative_pitch = (error_pitch - self.last_error_pitch) / dt
        output_pitch = (self.kp_pitch * error_pitch + 
                       self.ki_pitch * self.integral_pitch + 
                       self.kd_pitch * derivative_pitch)
        
        # 限制最大步长
        output_yaw = max(-self.max_step, min(self.max_step, math.degrees(output_yaw)))
        output_pitch = max(-self.max_step, min(self.max_step, math.degrees(output_pitch)))
        
        # 更新上一次误差和时间
        self.last_error_yaw = error_yaw
        self.last_error_pitch = error_pitch
        self.last_time = current_time
        
        return output_yaw, output_pitch
    
    def clamp_angle(self, angle, min_angle, max_angle):
        """限制角度范围"""
        return max(min_angle, min(max_angle, angle))
    
    def control_loop(self):
        """控制循环"""
        if not self.control_active:
            return
        
        # 检查是否长时间没有目标更新（用于跟踪模式）
        if self.last_target_time:
            current_time = self.get_clock().now()
            time_since_target = (current_time - self.last_target_time).nanoseconds / 1e9
            
            if time_since_target > 2.0:  # 2秒没有目标更新
                # 可以选择停止控制或保持当前角度
                # self.control_active = False
                # self.get_logger().info('目标跟踪超时，停止控制')
                pass
        
        # 发布角度命令
        self.publish_angle_command()
    
    def publish_angle_command(self):
        """发布角度命令"""
        cmd_msg = Vector3()
        cmd_msg.x = self.target_yaw    # 偏航角(度)
        cmd_msg.y = self.target_pitch  # 俯仰角(度)
        cmd_msg.z = 0.0               # 滚转角(暂未使用)
        
        self.angle_cmd_pub.publish(cmd_msg)
        
        # self.get_logger().debug(f'发布角度命令: ({self.target_yaw:.1f}°, {self.target_pitch:.1f}°)')
    
    def reset_control(self):
        """重置控制状态"""
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.integral_yaw = 0.0
        self.integral_pitch = 0.0
        self.last_error_yaw = 0.0
        self.last_error_pitch = 0.0
        self.control_active = False
        
        self.get_logger().info('云台控制状态已重置')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        gimbal_controller = SimplifiedGimbalController()
        rclpy.spin(gimbal_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gimbal_controller' in locals():
            gimbal_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
