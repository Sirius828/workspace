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
        self.declare_parameter('yaw_min', -3.14)    # -180度
        self.declare_parameter('yaw_max', 3.14)     # +180度
        self.declare_parameter('pitch_min', -0.52)  # -30度
        self.declare_parameter('pitch_max', 0.52)   # +30度
        
        # YAW轴模糊逻辑参数声明
        self.declare_parameter('yaw_fuzzy_error_small', 0.1)
        self.declare_parameter('yaw_fuzzy_error_medium', 0.3)
        self.declare_parameter('yaw_fuzzy_error_large', 1.0)
        self.declare_parameter('yaw_fuzzy_derror_small', 0.05)
        self.declare_parameter('yaw_fuzzy_derror_medium', 0.15)
        self.declare_parameter('yaw_fuzzy_derror_large', 0.5)
        
        # PITCH轴模糊逻辑参数声明
        self.declare_parameter('pitch_fuzzy_error_small', 0.1)
        self.declare_parameter('pitch_fuzzy_error_medium', 0.3)
        self.declare_parameter('pitch_fuzzy_error_large', 1.0)
        self.declare_parameter('pitch_fuzzy_derror_small', 0.05)
        self.declare_parameter('pitch_fuzzy_derror_medium', 0.15)
        self.declare_parameter('pitch_fuzzy_derror_large', 0.5)
        
        # YAW轴模糊规则矩阵参数声明
        self.declare_parameter('yaw_fuzzy_kp_rules.error_small', [1.2, 1.1, 1.0])
        self.declare_parameter('yaw_fuzzy_kp_rules.error_medium', [1.3, 1.2, 1.1])
        self.declare_parameter('yaw_fuzzy_kp_rules.error_large', [1.5, 1.4, 1.2])
        
        self.declare_parameter('yaw_fuzzy_ki_rules.error_small', [1.0, 0.9, 0.8])
        self.declare_parameter('yaw_fuzzy_ki_rules.error_medium', [1.1, 1.0, 0.9])
        self.declare_parameter('yaw_fuzzy_ki_rules.error_large', [1.2, 1.1, 1.0])
        
        self.declare_parameter('yaw_fuzzy_kd_rules.error_small', [0.8, 1.0, 1.2])
        self.declare_parameter('yaw_fuzzy_kd_rules.error_medium', [0.9, 1.1, 1.3])
        self.declare_parameter('yaw_fuzzy_kd_rules.error_large', [1.0, 1.2, 1.4])
        
        # PITCH轴模糊规则矩阵参数声明
        self.declare_parameter('pitch_fuzzy_kp_rules.error_small', [1.2, 1.1, 1.0])
        self.declare_parameter('pitch_fuzzy_kp_rules.error_medium', [1.3, 1.2, 1.1])
        self.declare_parameter('pitch_fuzzy_kp_rules.error_large', [1.5, 1.4, 1.2])
        
        self.declare_parameter('pitch_fuzzy_ki_rules.error_small', [1.0, 0.9, 0.8])
        self.declare_parameter('pitch_fuzzy_ki_rules.error_medium', [1.1, 1.0, 0.9])
        self.declare_parameter('pitch_fuzzy_ki_rules.error_large', [1.2, 1.1, 1.0])
        
        self.declare_parameter('pitch_fuzzy_kd_rules.error_small', [0.8, 1.0, 1.2])
        self.declare_parameter('pitch_fuzzy_kd_rules.error_medium', [0.9, 1.1, 1.3])
        self.declare_parameter('pitch_fuzzy_kd_rules.error_large', [1.0, 1.2, 1.4])
        
        # 前馈控制参数声明
        self.declare_parameter('enable_feedforward', True)
        self.declare_parameter('velocity_feedforward_gain', 0.7)
        self.declare_parameter('acceleration_feedforward_gain', 0.3)
        self.declare_parameter('prediction_time', 0.1)
        self.declare_parameter('motion_detection_threshold', 3.0)
        self.declare_parameter('motion_history_length', 5)
        
        # 获取参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.target_pixel_x = self.get_parameter('target_pixel_x').value
        self.target_pixel_y = self.get_parameter('target_pixel_y').value
        
        # 从YAML获取模糊逻辑参数
        # 获取YAW轴模糊逻辑参数
        yaw_error_small = self.get_parameter('yaw_fuzzy_error_small').value
        yaw_error_medium = self.get_parameter('yaw_fuzzy_error_medium').value
        yaw_error_large = self.get_parameter('yaw_fuzzy_error_large').value
        yaw_derror_small = self.get_parameter('yaw_fuzzy_derror_small').value
        yaw_derror_medium = self.get_parameter('yaw_fuzzy_derror_medium').value
        yaw_derror_large = self.get_parameter('yaw_fuzzy_derror_large').value
        
        # 获取PITCH轴模糊逻辑参数
        pitch_error_small = self.get_parameter('pitch_fuzzy_error_small').value
        pitch_error_medium = self.get_parameter('pitch_fuzzy_error_medium').value
        pitch_error_large = self.get_parameter('pitch_fuzzy_error_large').value
        pitch_derror_small = self.get_parameter('pitch_fuzzy_derror_small').value
        pitch_derror_medium = self.get_parameter('pitch_fuzzy_derror_medium').value
        pitch_derror_large = self.get_parameter('pitch_fuzzy_derror_large').value
        
        # 构建YAW轴误差范围字典
        yaw_error_ranges = {
            'small': (-yaw_error_small, yaw_error_small),
            'medium': (-yaw_error_medium, yaw_error_medium),
            'large': (-yaw_error_large, yaw_error_large)
        }
        
        yaw_derror_ranges = {
            'small': (-yaw_derror_small, yaw_derror_small),
            'medium': (-yaw_derror_medium, yaw_derror_medium),
            'large': (-yaw_derror_large, yaw_derror_large)
        }
        
        # 构建PITCH轴误差范围字典
        pitch_error_ranges = {
            'small': (-pitch_error_small, pitch_error_small),
            'medium': (-pitch_error_medium, pitch_error_medium),
            'large': (-pitch_error_large, pitch_error_large)
        }
        
        pitch_derror_ranges = {
            'small': (-pitch_derror_small, pitch_derror_small),
            'medium': (-pitch_derror_medium, pitch_derror_medium),
            'large': (-pitch_derror_large, pitch_derror_large)
        }
        
        # 从YAML获取YAW轴模糊规则矩阵
        yaw_fuzzy_rules = {
            'kp': [
                self.get_parameter('yaw_fuzzy_kp_rules.error_small').value,
                self.get_parameter('yaw_fuzzy_kp_rules.error_medium').value,
                self.get_parameter('yaw_fuzzy_kp_rules.error_large').value
            ],
            'ki': [
                self.get_parameter('yaw_fuzzy_ki_rules.error_small').value,
                self.get_parameter('yaw_fuzzy_ki_rules.error_medium').value,
                self.get_parameter('yaw_fuzzy_ki_rules.error_large').value
            ],
            'kd': [
                self.get_parameter('yaw_fuzzy_kd_rules.error_small').value,
                self.get_parameter('yaw_fuzzy_kd_rules.error_medium').value,
                self.get_parameter('yaw_fuzzy_kd_rules.error_large').value
            ]
        }
        
        # 从YAML获取PITCH轴模糊规则矩阵
        pitch_fuzzy_rules = {
            'kp': [
                self.get_parameter('pitch_fuzzy_kp_rules.error_small').value,
                self.get_parameter('pitch_fuzzy_kp_rules.error_medium').value,
                self.get_parameter('pitch_fuzzy_kp_rules.error_large').value
            ],
            'ki': [
                self.get_parameter('pitch_fuzzy_ki_rules.error_small').value,
                self.get_parameter('pitch_fuzzy_ki_rules.error_medium').value,
                self.get_parameter('pitch_fuzzy_ki_rules.error_large').value
            ],
            'kd': [
                self.get_parameter('pitch_fuzzy_kd_rules.error_small').value,
                self.get_parameter('pitch_fuzzy_kd_rules.error_medium').value,
                self.get_parameter('pitch_fuzzy_kd_rules.error_large').value
            ]
        }
        
        # PID控制器初始化 - 使用分离的模糊PID控制器
        max_change = self.get_parameter('max_angle_change').value
        self.yaw_pid = FuzzyPIDController(
            base_kp=self.get_parameter('yaw_kp').value,
            base_ki=self.get_parameter('yaw_ki').value,
            base_kd=self.get_parameter('yaw_kd').value,
            output_limit=max_change,
            error_ranges=yaw_error_ranges,
            derror_ranges=yaw_derror_ranges,
            fuzzy_rules=yaw_fuzzy_rules
        )
        
        self.pitch_pid = FuzzyPIDController(
            base_kp=self.get_parameter('pitch_kp').value,
            base_ki=self.get_parameter('pitch_ki').value,
            base_kd=self.get_parameter('pitch_kd').value,
            output_limit=max_change,
            error_ranges=pitch_error_ranges,
            derror_ranges=pitch_derror_ranges,
            fuzzy_rules=pitch_fuzzy_rules
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
        
        # 误差历史，用于预测控制
        self.prev_pixel_error_x = 0.0
        self.prev_pixel_error_y = 0.0
        self.error_velocity_x = 0.0
        self.error_velocity_y = 0.0
        self.prev_time = None
        
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
        
        # 前馈控制初始化
        self.enable_feedforward = self.get_parameter('enable_feedforward').value
        self.velocity_feedforward_gain = self.get_parameter('velocity_feedforward_gain').value
        self.acceleration_feedforward_gain = self.get_parameter('acceleration_feedforward_gain').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.motion_detection_threshold = self.get_parameter('motion_detection_threshold').value
        self.motion_history_length = self.get_parameter('motion_history_length').value
        
        # 运动历史记录
        self.target_history = []  # 存储最近的目标位置和时间
        self.velocity_estimate_x = 0.0
        self.velocity_estimate_y = 0.0
        self.acceleration_estimate_x = 0.0
        self.acceleration_estimate_y = 0.0
        
        # QoS配置 - 实时性优化但保持兼容性
        qos = QoSProfile(
            depth=1,  # 保持最小队列深度，减少延迟
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 最佳努力，优先速度
            durability=QoSDurabilityPolicy.VOLATILE  # 易失性，减少内存占用
        )
        
        # 云台控制QoS配置 - 使用RELIABLE确保兼容性
        gimbal_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,  # 使用RELIABLE确保与其他节点兼容
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
            gimbal_qos  # 使用兼容的RELIABLE QoS配置
        )
        
        # 定时器：监控控制超时 - 提高检查频率
        self.timeout_timer = self.create_timer(0.05, self.check_timeout)  # 20Hz检查频率
        
        self.get_logger().info('Gimbal pixel controller started with Separated Fuzzy PID')
        self.get_logger().info(f'Target pixel: ({self.target_pixel_x}, {self.target_pixel_y})')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'YAW Fuzzy error ranges: Small={yaw_error_small}, Medium={yaw_error_medium}, Large={yaw_error_large}')
        self.get_logger().info(f'PITCH Fuzzy error ranges: Small={pitch_error_small}, Medium={pitch_error_medium}, Large={pitch_error_large}')
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
        
        # 前馈控制：运动检测和预测
        feedforward_x = 0.0
        feedforward_y = 0.0
        
        if self.enable_feedforward:
            # 更新目标历史记录
            self.target_history.append((msg.x, msg.y, current_time))
            if len(self.target_history) > self.motion_history_length:
                self.target_history.pop(0)
            
            # 计算运动速度和加速度
            if len(self.target_history) >= 3:
                # 使用最近3个点计算速度和加速度
                recent_points = self.target_history[-3:]
                dt1 = recent_points[1][2] - recent_points[0][2]
                dt2 = recent_points[2][2] - recent_points[1][2]
                
                if dt1 > 0 and dt2 > 0:
                    # 计算速度（像素/秒）
                    vel_x1 = (recent_points[1][0] - recent_points[0][0]) / dt1
                    vel_y1 = (recent_points[1][1] - recent_points[0][1]) / dt1
                    vel_x2 = (recent_points[2][0] - recent_points[1][0]) / dt2
                    vel_y2 = (recent_points[2][1] - recent_points[1][1]) / dt2
                    
                    # 平滑速度估计
                    self.velocity_estimate_x = 0.3 * vel_x2 + 0.7 * self.velocity_estimate_x
                    self.velocity_estimate_y = 0.3 * vel_y2 + 0.7 * self.velocity_estimate_y
                    
                    # 计算加速度（像素/秒²）
                    self.acceleration_estimate_x = 0.3 * (vel_x2 - vel_x1) / dt2 + 0.7 * self.acceleration_estimate_x
                    self.acceleration_estimate_y = 0.3 * (vel_y2 - vel_y1) / dt2 + 0.7 * self.acceleration_estimate_y
                    
                    # 检测是否在运动
                    motion_magnitude = math.sqrt(self.velocity_estimate_x**2 + self.velocity_estimate_y**2)
                    
                    if motion_magnitude > self.motion_detection_threshold:
                        # 计算前馈控制量：预测未来位置的误差
                        predicted_x = msg.x + self.velocity_estimate_x * self.prediction_time + 0.5 * self.acceleration_estimate_x * self.prediction_time**2
                        predicted_y = msg.y + self.velocity_estimate_y * self.prediction_time + 0.5 * self.acceleration_estimate_y * self.prediction_time**2
                        
                        # 前馈误差（预测位置与目标中心的偏差）
                        predicted_error_x = predicted_x - self.target_pixel_x
                        predicted_error_y = predicted_y - self.target_pixel_y
                        
                        # 前馈控制量
                        feedforward_x = self.velocity_feedforward_gain * self.velocity_estimate_x + self.acceleration_feedforward_gain * self.acceleration_estimate_x
                        feedforward_y = self.velocity_feedforward_gain * self.velocity_estimate_y + self.acceleration_feedforward_gain * self.acceleration_estimate_y
                        
                        self.get_logger().debug(f'Motion detected: vel=({self.velocity_estimate_x:.1f}, {self.velocity_estimate_y:.1f}), ff=({feedforward_x:.3f}, {feedforward_y:.3f})')
        
        # 死区检测 - 小误差时不响应，避免抖动
        if abs(pixel_error_x) < self.deadzone_pixel and abs(pixel_error_y) < self.deadzone_pixel and abs(feedforward_x) < 0.01 and abs(feedforward_y) < 0.01:
            self.get_logger().debug(f'Within deadzone: ({pixel_error_x:.1f}, {pixel_error_y:.1f}), skipping control')
            return
        
        # 自适应滤波：根据误差大小和变化率动态调整滤波强度
        error_magnitude = math.sqrt(pixel_error_x**2 + pixel_error_y**2)
        
        # 计算误差变化率
        if hasattr(self, 'prev_pixel_error_x') and hasattr(self, 'prev_pixel_error_y'):
            error_change_rate = math.sqrt(
                (pixel_error_x - self.prev_pixel_error_x)**2 + 
                (pixel_error_y - self.prev_pixel_error_y)**2
            )
        else:
            error_change_rate = 0.0
            
        # 保存当前误差用于下次计算变化率
        self.prev_pixel_error_x = pixel_error_x
        self.prev_pixel_error_y = pixel_error_y
        
        # 动态滤波系数：快速变化或大误差时减少滤波，慢速变化时增加滤波
        if error_magnitude > 20 or error_change_rate > 15:  # 大误差或快速变化
            dynamic_alpha = max(0.3, self.filter_alpha - 0.3)  # 减少滤波，提高响应
        elif error_magnitude > 10 or error_change_rate > 8:  # 中等误差或变化
            dynamic_alpha = max(0.4, self.filter_alpha - 0.2)  # 适度滤波
        else:  # 小误差且变化缓慢
            dynamic_alpha = self.filter_alpha  # 使用原始滤波系数
            
        # 自适应低通滤波器 - 平滑误差信号
        self.filtered_error_x = dynamic_alpha * self.filtered_error_x + (1 - dynamic_alpha) * pixel_error_x
        self.filtered_error_y = dynamic_alpha * self.filtered_error_y + (1 - dynamic_alpha) * pixel_error_y
        
        # 使用滤波后的误差
        pixel_error_x = self.filtered_error_x
        pixel_error_y = self.filtered_error_y
        
        # 预测控制：基于误差变化趋势预测未来位置
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                # 计算误差变化速度
                velocity_x = (pixel_error_x - self.prev_pixel_error_x) / dt
                velocity_y = (pixel_error_y - self.prev_pixel_error_y) / dt
                
                # 平滑速度估计（简单低通滤波）
                self.error_velocity_x = 0.7 * self.error_velocity_x + 0.3 * velocity_x
                self.error_velocity_y = 0.7 * self.error_velocity_y + 0.3 * velocity_y
                
                # 预测时间（根据系统延迟调整）
                prediction_time = 0.05  # 50ms 预测时间
                
                # 添加预测补偿
                predicted_error_x = pixel_error_x + self.error_velocity_x * prediction_time
                predicted_error_y = pixel_error_y + self.error_velocity_y * prediction_time
                
                # 仅在速度较大时使用预测，避免噪声放大
                if abs(self.error_velocity_x) > 10:  # 像素/秒
                    pixel_error_x = predicted_error_x
                if abs(self.error_velocity_y) > 10:  # 像素/秒
                    pixel_error_y = predicted_error_y
        
        # 更新历史信息
        self.prev_pixel_error_x = self.filtered_error_x
        self.prev_pixel_error_y = self.filtered_error_y
        self.prev_time = current_time
        
        # 像素误差归一化（转换为角度误差的近似）
        # 这个映射需要根据相机视场角调整
        fov_h = 1.0  # 水平视场角（弧度），需要根据实际相机调整
        fov_v = 0.75 # 垂直视场角（弧度），需要根据实际相机调整
        
        # 将像素误差转换为角度误差
        angle_error_yaw = -(pixel_error_x / self.image_width) * fov_h
        angle_error_pitch = -(pixel_error_y / self.image_height) * fov_v  # 负号：向上为正
        
        # 自适应增强：根据误差大小动态调整响应强度
        error_magnitude = math.sqrt(angle_error_yaw**2 + angle_error_pitch**2)
        
        # 动态增强系数：误差越大，响应越强
        if error_magnitude > 0.3:  # 大误差
            enhancement_factor = 1.5
        elif error_magnitude > 0.15:  # 中等误差
            enhancement_factor = 1.2
        else:  # 小误差
            enhancement_factor = 1.0
            
        # 应用增强系数
        angle_error_yaw *= enhancement_factor
        angle_error_pitch *= enhancement_factor
        
        # PID控制计算 - 模糊PID控制
        yaw_output, yaw_kp_adj, yaw_ki_adj, yaw_kd_adj = self.yaw_pid.update(angle_error_yaw, current_time)
        pitch_output, pitch_kp_adj, pitch_ki_adj, pitch_kd_adj = self.pitch_pid.update(angle_error_pitch, current_time)
        
        # 前馈控制计算
        feedforward_yaw = 0.0
        feedforward_pitch = 0.0
        
        if self.enable_feedforward and len(self.target_history) >= 2:
            # 检查运动速度是否足够大，只在明显运动时启用前馈控制
            speed_magnitude = math.sqrt(self.error_velocity_x**2 + self.error_velocity_y**2)
            
            # 只在速度超过阈值时启用前馈控制，避免在微小抖动时触发
            if speed_magnitude > self.motion_detection_threshold:
                # 基于速度的前馈控制
                velocity_feedforward_yaw = self.error_velocity_x * self.velocity_feedforward_gain * (fov_h / self.image_width)
                velocity_feedforward_pitch = self.error_velocity_y * self.velocity_feedforward_gain * (fov_v / self.image_height)
            
            # 基于加速度的前馈控制（使用已计算的速度变化）
            acceleration_feedforward_yaw = 0.0
            acceleration_feedforward_pitch = 0.0
            
            if len(self.target_history) >= 3:
                # 计算加速度（基于速度变化）
                recent_points = self.target_history[-3:]
                dt1 = recent_points[1][2] - recent_points[0][2]
                dt2 = recent_points[2][2] - recent_points[1][2]
                
                if dt1 > 0 and dt2 > 0:
                    # 计算两个时间段的速度
                    vel_x1 = (recent_points[1][0] - recent_points[0][0]) / dt1
                    vel_y1 = (recent_points[1][1] - recent_points[0][1]) / dt1
                    vel_x2 = (recent_points[2][0] - recent_points[1][0]) / dt2
                    vel_y2 = (recent_points[2][1] - recent_points[1][1]) / dt2
                    
                    # 计算加速度（像素/秒²）
                    if dt2 > 0:
                        acc_x = (vel_x2 - vel_x1) / dt2
                        acc_y = (vel_y2 - vel_y1) / dt2
                        
                        acceleration_feedforward_yaw = acc_x * self.acceleration_feedforward_gain * (fov_h / self.image_width)
                        acceleration_feedforward_pitch = acc_y * self.acceleration_feedforward_gain * (fov_v / self.image_height)
            
            # 综合前馈控制输出
            feedforward_yaw = velocity_feedforward_yaw + acceleration_feedforward_yaw
            feedforward_pitch = velocity_feedforward_pitch + acceleration_feedforward_pitch
            
            # 前馈控制限制（避免过度预测）- 更保守的限制
            max_feedforward = 0.02  # 降低最大前馈控制量到约1.1度，更适合精细控制
            feedforward_yaw = max(-max_feedforward, min(max_feedforward, feedforward_yaw))
            feedforward_pitch = max(-max_feedforward, min(max_feedforward, feedforward_pitch))
        
        # 组合PID和前馈控制输出
        total_yaw_output = yaw_output + feedforward_yaw
        total_pitch_output = pitch_output + feedforward_pitch
        
        # 动态运动阈值：根据误差大小调整阈值
        dynamic_threshold = self.min_movement_threshold
        if error_magnitude > 0.2:
            dynamic_threshold *= 0.5  # 大误差时降低阈值，提高响应
        elif error_magnitude > 0.1:
            dynamic_threshold *= 0.7  # 中等误差时适度降低阈值
            
        # 最小运动阈值检测 - 避免微小的无效运动
        if abs(total_yaw_output) < dynamic_threshold and abs(total_pitch_output) < dynamic_threshold:
            self.get_logger().debug(f'Movement below threshold: yaw={total_yaw_output:.6f}, pitch={total_pitch_output:.6f}')
            return
        
        # 更新云台角度
        new_yaw = self.current_yaw + total_yaw_output
        new_pitch = self.current_pitch + total_pitch_output
        
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
        
        # 日志输出 - 包含模糊PID调整信息和前馈控制
        self.get_logger().info(
            f'Target: ({msg.x:.1f}, {msg.y:.1f}) '
            f'Error: ({pixel_error_x:.1f}, {pixel_error_y:.1f}) '
            f'Gimbal: yaw={math.degrees(new_yaw):.1f}°, pitch={math.degrees(new_pitch):.1f}° '
            f'PID: Y={yaw_output:.4f}, P={pitch_output:.4f} '
            f'FF: Y={feedforward_yaw:.4f}, P={feedforward_pitch:.4f} '
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
