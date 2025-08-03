#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Twist
import math
import time
import numpy as np

class FuzzyPIDController:
    """模糊PID控制器"""
    def __init__(self, base_kp=1.0, base_ki=0.0, base_kd=0.0, output_limit=None, 
                 error_ranges=None, derror_ranges=None, fuzzy_rules=None):
        # 基础PID参数
        self.base_kp = base_kp
        self.base_ki = base_ki
        self.base_kd = base_kd
        self.output_limit = output_limit
        
        # 当前PID参数（会被模糊逻辑调整）
        self.kp = base_kp
        self.ki = base_ki
        self.kd = base_kd
        
        # PID内部状态
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        
        # 模糊规则参数（可从配置文件传入）
        if error_ranges is not None:
            self.error_ranges = error_ranges
        else:
            self.error_ranges = {
                'small': (-0.1, 0.1),     # 小误差范围
                'medium': (-0.3, 0.3),    # 中等误差范围
                'large': (-1.0, 1.0)      # 大误差范围
            }
        
        if derror_ranges is not None:
            self.derror_ranges = derror_ranges
        else:
            self.derror_ranges = {
                'small': (-0.05, 0.05),   # 小误差变化率
                'medium': (-0.15, 0.15),  # 中等误差变化率
                'large': (-0.5, 0.5)      # 大误差变化率
            }
        
        # 模糊规则矩阵（可从配置文件传入）
        if fuzzy_rules is not None:
            self.kp_rules = fuzzy_rules['kp']
            self.ki_rules = fuzzy_rules['ki']
            self.kd_rules = fuzzy_rules['kd']
        else:
            # 默认模糊规则矩阵
            self.kp_rules = [
                [1.2, 1.1, 1.0],  # error small
                [1.3, 1.2, 1.1],  # error medium
                [1.5, 1.4, 1.2]   # error large
            ]
            self.ki_rules = [
                [1.0, 0.9, 0.8],  # error small
                [1.1, 1.0, 0.9],  # error medium  
                [1.2, 1.1, 1.0]   # error large
            ]
            self.kd_rules = [
                [0.8, 1.0, 1.2],  # error small
                [0.9, 1.1, 1.3],  # error medium
                [1.0, 1.2, 1.4]   # error large
            ]
        
    def membership_function(self, value, range_small, range_medium, range_large):
        """计算隶属度函数"""
        # 三角形隶属度函数
        small_membership = max(0, 1 - abs(value) / range_small[1]) if abs(value) <= range_small[1] else 0
        
        if abs(value) <= range_medium[1]:
            if abs(value) <= range_small[1]:
                medium_membership = abs(value) / range_small[1]
            else:
                medium_membership = 1 - (abs(value) - range_small[1]) / (range_medium[1] - range_small[1])
        else:
            medium_membership = 0
            
        if abs(value) <= range_large[1]:
            if abs(value) <= range_medium[1]:
                large_membership = 0
            else:
                large_membership = (abs(value) - range_medium[1]) / (range_large[1] - range_medium[1])
        else:
            large_membership = 1
            
        return small_membership, medium_membership, large_membership
    
    def fuzzy_pid_adjustment(self, error, derror):
        """模糊PID参数调整"""
        # 计算误差的隶属度
        e_small, e_medium, e_large = self.membership_function(
            error, self.error_ranges['small'], self.error_ranges['medium'], self.error_ranges['large']
        )
        
        # 计算误差变化率的隶属度
        de_small, de_medium, de_large = self.membership_function(
            derror, self.derror_ranges['small'], self.derror_ranges['medium'], self.derror_ranges['large']
        )
        
        # 计算加权平均调整系数
        total_weight = 0
        kp_adj = ki_adj = kd_adj = 0
        
        error_memberships = [e_small, e_medium, e_large]
        derror_memberships = [de_small, de_medium, de_large]
        
        for i in range(3):
            for j in range(3):
                weight = error_memberships[i] * derror_memberships[j]
                total_weight += weight
                kp_adj += weight * self.kp_rules[i][j]
                ki_adj += weight * self.ki_rules[i][j]
                kd_adj += weight * self.kd_rules[i][j]
        
        if total_weight > 0:
            kp_adj /= total_weight
            ki_adj /= total_weight
            kd_adj /= total_weight
        else:
            kp_adj = ki_adj = kd_adj = 1.0
        
        # 应用调整系数
        self.kp = self.base_kp * kp_adj
        self.ki = self.base_ki * ki_adj
        self.kd = self.base_kd * kd_adj
        
        return kp_adj, ki_adj, kd_adj
        
    def update(self, error, current_time):
        """更新模糊PID控制器"""
        if self.previous_time is None:
            self.previous_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.previous_time
            
        if dt <= 0.0:
            dt = 0.001  # 避免除零
            
        # 计算误差变化率
        derror = (error - self.previous_error) / dt
        
        # 模糊PID参数调整
        kp_adj, ki_adj, kd_adj = self.fuzzy_pid_adjustment(error, derror)
        
        # 积分项
        self.integral += error * dt
        
        # 积分饱和限制
        if self.output_limit is not None:
            max_integral = self.output_limit / (self.ki + 1e-6)
            self.integral = max(-max_integral, min(max_integral, self.integral))
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derror
        
        # 输出限制
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
            
        # 更新历史值
        self.previous_error = error
        self.previous_time = current_time
        
        return output, kp_adj, ki_adj, kd_adj
    
    def reset(self):
        """重置模糊PID控制器"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        self.kp = self.base_kp
        self.ki = self.base_ki
        self.kd = self.base_kd

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
        self.declare_parameter('max_angle_change', 0.02)    # 最大角度变化（弧度）
        self.declare_parameter('min_confidence', 0.8)       # 最小置信度阈值
        self.declare_parameter('control_timeout', 2.0)      # 控制超时时间（秒）
        
        # 抖动抑制参数
        self.declare_parameter('deadzone_pixel', 3.0)       # 像素死区，小于此值不响应
        self.declare_parameter('filter_alpha', 0.7)         # 低通滤波器系数 (0-1)
        self.declare_parameter('min_movement_threshold', 0.001)  # 最小运动阈值（弧度）
        
        # 云台角度限制
        self.declare_parameter('yaw_min', -1.57)    # -90度
        self.declare_parameter('yaw_max', 1.57)     # +90度
        self.declare_parameter('pitch_min', -0.52)  # -30度
        self.declare_parameter('pitch_max', 0.52)   # +30度
        
        # 模糊逻辑参数声明
        self.declare_parameter('fuzzy_error_small', 0.1)
        self.declare_parameter('fuzzy_error_medium', 0.3)
        self.declare_parameter('fuzzy_error_large', 1.0)
        self.declare_parameter('fuzzy_derror_small', 0.05)
        self.declare_parameter('fuzzy_derror_medium', 0.15)
        self.declare_parameter('fuzzy_derror_large', 0.5)
        
        # 模糊规则矩阵参数声明
        self.declare_parameter('fuzzy_kp_rules.error_small', [1.2, 1.1, 1.0])
        self.declare_parameter('fuzzy_kp_rules.error_medium', [1.3, 1.2, 1.1])
        self.declare_parameter('fuzzy_kp_rules.error_large', [1.5, 1.4, 1.2])
        
        self.declare_parameter('fuzzy_ki_rules.error_small', [1.0, 0.9, 0.8])
        self.declare_parameter('fuzzy_ki_rules.error_medium', [1.1, 1.0, 0.9])
        self.declare_parameter('fuzzy_ki_rules.error_large', [1.2, 1.1, 1.0])
        
        self.declare_parameter('fuzzy_kd_rules.error_small', [0.8, 1.0, 1.2])
        self.declare_parameter('fuzzy_kd_rules.error_medium', [0.9, 1.1, 1.3])
        self.declare_parameter('fuzzy_kd_rules.error_large', [1.0, 1.2, 1.4])
        
        # 获取参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.target_pixel_x = self.get_parameter('target_pixel_x').value
        self.target_pixel_y = self.get_parameter('target_pixel_y').value
        
        # 从YAML获取模糊逻辑参数
        error_small = self.get_parameter('fuzzy_error_small').value
        error_medium = self.get_parameter('fuzzy_error_medium').value
        error_large = self.get_parameter('fuzzy_error_large').value
        derror_small = self.get_parameter('fuzzy_derror_small').value
        derror_medium = self.get_parameter('fuzzy_derror_medium').value
        derror_large = self.get_parameter('fuzzy_derror_large').value
        
        # 构建误差范围字典
        error_ranges = {
            'small': (-error_small, error_small),
            'medium': (-error_medium, error_medium),
            'large': (-error_large, error_large)
        }
        
        derror_ranges = {
            'small': (-derror_small, derror_small),
            'medium': (-derror_medium, derror_medium),
            'large': (-derror_large, derror_large)
        }
        
        # 从YAML获取模糊规则矩阵
        fuzzy_rules = {
            'kp': [
                self.get_parameter('fuzzy_kp_rules.error_small').value,
                self.get_parameter('fuzzy_kp_rules.error_medium').value,
                self.get_parameter('fuzzy_kp_rules.error_large').value
            ],
            'ki': [
                self.get_parameter('fuzzy_ki_rules.error_small').value,
                self.get_parameter('fuzzy_ki_rules.error_medium').value,
                self.get_parameter('fuzzy_ki_rules.error_large').value
            ],
            'kd': [
                self.get_parameter('fuzzy_kd_rules.error_small').value,
                self.get_parameter('fuzzy_kd_rules.error_medium').value,
                self.get_parameter('fuzzy_kd_rules.error_large').value
            ]
        }
        
        # PID控制器初始化 - 使用模糊PID控制器
        max_change = self.get_parameter('max_angle_change').value
        self.yaw_pid = FuzzyPIDController(
            base_kp=self.get_parameter('yaw_kp').value,
            base_ki=self.get_parameter('yaw_ki').value,
            base_kd=self.get_parameter('yaw_kd').value,
            output_limit=max_change,
            error_ranges=error_ranges,
            derror_ranges=derror_ranges,
            fuzzy_rules=fuzzy_rules
        )
        
        self.pitch_pid = FuzzyPIDController(
            base_kp=self.get_parameter('pitch_kp').value,
            base_ki=self.get_parameter('pitch_ki').value,
            base_kd=self.get_parameter('pitch_kd').value,
            output_limit=max_change,
            error_ranges=error_ranges,
            derror_ranges=derror_ranges,
            fuzzy_rules=fuzzy_rules
        )
        
        # 控制参数
        self.min_confidence = self.get_parameter('min_confidence').value
        self.control_timeout = self.get_parameter('control_timeout').value
        
        # 抖动抑制参数
        self.deadzone_pixel = self.get_parameter('deadzone_pixel').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.min_movement_threshold = self.get_parameter('min_movement_threshold').value
        
        # 滤波器状态
        self.filtered_error_x = 0.0
        self.filtered_error_y = 0.0
        
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
        
        self.get_logger().info('Gimbal pixel controller started with Fuzzy PID')
        self.get_logger().info(f'Target pixel: ({self.target_pixel_x}, {self.target_pixel_y})')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Fuzzy error ranges: Small={error_small}, Medium={error_medium}, Large={error_large}')
        self.get_logger().info(f'Base PID - Yaw: Kp={self.get_parameter("yaw_kp").value}, Ki={self.get_parameter("yaw_ki").value}, Kd={self.get_parameter("yaw_kd").value}')
        self.get_logger().info(f'Base PID - Pitch: Kp={self.get_parameter("pitch_kp").value}, Ki={self.get_parameter("pitch_ki").value}, Kd={self.get_parameter("pitch_kd").value}')
    
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
        
        # 死区检测 - 小误差时不响应，避免抖动
        if abs(pixel_error_x) < self.deadzone_pixel and abs(pixel_error_y) < self.deadzone_pixel:
            self.get_logger().debug(f'Within deadzone: ({pixel_error_x:.1f}, {pixel_error_y:.1f}), skipping control')
            return
        
        # 低通滤波器 - 平滑误差信号
        self.filtered_error_x = self.filter_alpha * self.filtered_error_x + (1 - self.filter_alpha) * pixel_error_x
        self.filtered_error_y = self.filter_alpha * self.filtered_error_y + (1 - self.filter_alpha) * pixel_error_y
        
        # 使用滤波后的误差
        pixel_error_x = self.filtered_error_x
        pixel_error_y = self.filtered_error_y
        
        # 像素误差归一化（转换为角度误差的近似）
        # 这个映射需要根据相机视场角调整
        fov_h = 1.0  # 水平视场角（弧度），需要根据实际相机调整
        fov_v = 0.75 # 垂直视场角（弧度），需要根据实际相机调整
        
        # 将像素误差转换为角度误差
        angle_error_yaw = (pixel_error_x / self.image_width) * fov_h
        angle_error_pitch = -(pixel_error_y / self.image_height) * fov_v  # 负号：向上为正
        
        # PID控制计算 - 模糊PID控制
        yaw_output, yaw_kp_adj, yaw_ki_adj, yaw_kd_adj = self.yaw_pid.update(angle_error_yaw, current_time)
        pitch_output, pitch_kp_adj, pitch_ki_adj, pitch_kd_adj = self.pitch_pid.update(angle_error_pitch, current_time)
        
        # 最小运动阈值检测 - 避免微小的无效运动
        if abs(yaw_output) < self.min_movement_threshold and abs(pitch_output) < self.min_movement_threshold:
            self.get_logger().debug(f'Movement below threshold: yaw={yaw_output:.6f}, pitch={pitch_output:.6f}')
            return
        
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
        
        # 日志输出 - 包含模糊PID调整信息
        self.get_logger().info(
            f'Target: ({msg.x:.1f}, {msg.y:.1f}) '
            f'Error: ({pixel_error_x:.1f}, {pixel_error_y:.1f}) '
            f'Gimbal: yaw={math.degrees(new_yaw):.1f}°, pitch={math.degrees(new_pitch):.1f}° '
            f'Conf: {confidence:.3f}'
        )
        
        # 详细的模糊PID调试信息（可选，低频率输出）
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 10 == 0:  # 每10次更新输出一次调试信息
            self.get_logger().debug(
                f'Fuzzy PID - Yaw: Kp_adj={yaw_kp_adj:.3f}, Ki_adj={yaw_ki_adj:.3f}, Kd_adj={yaw_kd_adj:.3f} | '
                f'Pitch: Kp_adj={pitch_kp_adj:.3f}, Ki_adj={pitch_ki_adj:.3f}, Kd_adj={pitch_kd_adj:.3f}'
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
        
        # 重置滤波器状态
        self.filtered_error_x = 0.0
        self.filtered_error_y = 0.0
        
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
