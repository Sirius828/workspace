#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Twist
import math
import time

class PIDController:
    """简单的PID控制器"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        
    def update(self, error, current_time):
        """更新PID控制器"""
        if self.previous_time is None:
            self.previous_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.previous_time
            
        if dt <= 0.0:
            dt = 0.001  # 避免除零
            
        # 积分项
        self.integral += error * dt
        
        # 微分项
        derivative = (error - self.previous_error) / dt
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 输出限制
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
            
        # 更新历史值
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

class GimbalPixelController(Node):
    def __init__(self):
        super().__init__('gimbal_pixel_controller')
        
        # 参数声明
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('target_pixel_x', 320.0)  # 图像中心x
        self.declare_parameter('target_pixel_y', 240.0)  # 图像中心y
        
        # PID参数
        self.declare_parameter('yaw_kp', 0.001)    # 偏航角比例增益
        self.declare_parameter('yaw_ki', 0.0001)   # 偏航角积分增益
        self.declare_parameter('yaw_kd', 0.0002)   # 偏航角微分增益
        self.declare_parameter('pitch_kp', 0.001)  # 俯仰角比例增益
        self.declare_parameter('pitch_ki', 0.0001) # 俯仰角积分增益
        self.declare_parameter('pitch_kd', 0.0002) # 俯仰角微分增益
        
        # 控制限制
        self.declare_parameter('max_angle_change', 0.1)    # 最大角度变化（弧度）
        self.declare_parameter('min_confidence', 0.5)      # 最小置信度阈值
        self.declare_parameter('control_timeout', 2.0)     # 控制超时时间（秒）
        
        # 云台角度限制
        self.declare_parameter('yaw_min', -1.57)    # -90度
        self.declare_parameter('yaw_max', 1.57)     # +90度
        self.declare_parameter('pitch_min', -0.52)  # -30度
        self.declare_parameter('pitch_max', 0.52)   # +30度
        
        # 获取参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.target_pixel_x = self.get_parameter('target_pixel_x').value
        self.target_pixel_y = self.get_parameter('target_pixel_y').value
        
        # PID控制器初始化
        max_change = self.get_parameter('max_angle_change').value
        self.yaw_pid = PIDController(
            kp=self.get_parameter('yaw_kp').value,
            ki=self.get_parameter('yaw_ki').value,
            kd=self.get_parameter('yaw_kd').value,
            output_limit=max_change
        )
        
        self.pitch_pid = PIDController(
            kp=self.get_parameter('pitch_kp').value,
            ki=self.get_parameter('pitch_ki').value,
            kd=self.get_parameter('pitch_kd').value,
            output_limit=max_change
        )
        
        # 控制参数
        self.min_confidence = self.get_parameter('min_confidence').value
        self.control_timeout = self.get_parameter('control_timeout').value
        
        # 云台角度限制
        self.yaw_min = self.get_parameter('yaw_min').value
        self.yaw_max = self.get_parameter('yaw_max').value
        self.pitch_min = self.get_parameter('pitch_min').value
        self.pitch_max = self.get_parameter('pitch_max').value
        
        # 云台当前状态
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.last_target_time = None
        self.tracking_active = False
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 订阅者和发布者
        self.target_sub = self.create_subscription(
            Point,
            '/target_position_pixel',
            self.target_callback,
            qos
        )
        
        self.gimbal_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_gimbal',
            10
        )
        
        # 定时器：监控控制超时
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        
        self.get_logger().info('Gimbal pixel controller started')
        self.get_logger().info(f'Target pixel: ({self.target_pixel_x}, {self.target_pixel_y})')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
    
    def target_callback(self, msg):
        """目标位置回调函数"""
        current_time = time.time()
        
        # 检查置信度（存储在z坐标中）
        confidence = msg.z
        if confidence < self.min_confidence:
            self.get_logger().debug(f'Low confidence: {confidence:.3f}, skipping control')
            return
        
        # 计算像素误差
        pixel_error_x = msg.x - self.target_pixel_x  # 正值表示目标在右侧
        pixel_error_y = msg.y - self.target_pixel_y  # 正值表示目标在下方
        
        # 像素误差归一化（转换为角度误差的近似）
        # 这个映射需要根据相机视场角调整
        fov_h = 1.0  # 水平视场角（弧度），需要根据实际相机调整
        fov_v = 0.75 # 垂直视场角（弧度），需要根据实际相机调整
        
        # 将像素误差转换为角度误差
        angle_error_yaw = (pixel_error_x / self.image_width) * fov_h
        angle_error_pitch = -(pixel_error_y / self.image_height) * fov_v  # 负号：向上为正
        
        # PID控制计算
        yaw_output = self.yaw_pid.update(angle_error_yaw, current_time)
        pitch_output = self.pitch_pid.update(angle_error_pitch, current_time)
        
        # 更新云台角度
        new_yaw = self.current_yaw + yaw_output
        new_pitch = self.current_pitch + pitch_output
        
        # 角度限制
        new_yaw = max(self.yaw_min, min(self.yaw_max, new_yaw))
        new_pitch = max(self.pitch_min, min(self.pitch_max, new_pitch))
        
        # 发送云台控制指令
        gimbal_cmd = Twist()
        gimbal_cmd.angular.x = new_yaw    # yaw角度
        gimbal_cmd.angular.y = new_pitch  # pitch角度
        gimbal_cmd.angular.z = 0.0
        
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        # 更新状态
        self.current_yaw = new_yaw
        self.current_pitch = new_pitch
        self.last_target_time = current_time
        self.tracking_active = True
        
        # 日志输出
        self.get_logger().info(
            f'Target: ({msg.x:.1f}, {msg.y:.1f}) '
            f'Error: ({pixel_error_x:.1f}, {pixel_error_y:.1f}) '
            f'Gimbal: yaw={math.degrees(new_yaw):.1f}°, pitch={math.degrees(new_pitch):.1f}° '
            f'Conf: {confidence:.3f}'
        )
    
    def check_timeout(self):
        """检查控制超时"""
        if not self.tracking_active:
            return
            
        current_time = time.time()
        if (self.last_target_time is not None and 
            current_time - self.last_target_time > self.control_timeout):
            
            self.get_logger().warn('Target lost, stopping gimbal control')
            self.stop_tracking()
    
    def stop_tracking(self):
        """停止跟踪"""
        self.tracking_active = False
        
        # 重置PID控制器
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        
        # 可选：发送停止指令或保持当前位置
        gimbal_cmd = Twist()
        gimbal_cmd.angular.x = self.current_yaw
        gimbal_cmd.angular.y = self.current_pitch
        gimbal_cmd.angular.z = 0.0
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        self.get_logger().info('Tracking stopped')
    
    def reset_gimbal_position(self):
        """重置云台到中心位置"""
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        gimbal_cmd = Twist()
        gimbal_cmd.angular.x = 0.0
        gimbal_cmd.angular.y = 0.0
        gimbal_cmd.angular.z = 0.0
        self.gimbal_cmd_pub.publish(gimbal_cmd)
        
        self.stop_tracking()
        self.get_logger().info('Gimbal reset to center position')

def main(args=None):
    rclpy.init(args=args)
    node = GimbalPixelController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gimbal controller')
        node.reset_gimbal_position()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
