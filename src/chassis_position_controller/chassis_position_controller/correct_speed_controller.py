#!/usr/bin/env python3

"""
Correct Speed-Controlled Position Controller

This implementation properly handles speed control by:
1. Using position error to determine direction
2. Applying target speed as velocity magnitude
3. Proper proportional control for smooth movement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
import math
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CorrectSpeedController(Node):
    def __init__(self):
        super().__init__('correct_speed_controller')
        
        # 加载配置参数
        self.load_parameters()
        
        # 模糊PID控制参数
        self.kp_linear = 1.2
        self.kp_angular = 1.8
        
        # 模糊控制历史误差（用于模糊推理）
        self.prev_distance_error = 0.0
        self.prev_heading_error = 0.0
        
        # State variables
        self.current_pose = None
        self.target_pose = None
        self.last_target_pose = None  # 添加：用于检测新目标
        self.target_linear_speed = None
        self.target_angular_speed = None
        self.target_reached = False
        self.last_cmd_time = time.time()
        self.is_holding_position = False  # 添加：跟踪是否在保持位置状态
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/position_controller/target_reached', 10)
        self.current_speed_pub = self.create_publisher(Float32, '/position_controller/current_speed', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/chassis/odom', self.odom_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.target_with_speed_sub = self.create_subscription(
            TwistStamped, '/target_pose_with_speed', self.target_with_speed_callback, 10)
        self.speed_override_sub = self.create_subscription(
            Float32, '/target_speed_override', self.speed_override_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_callback)
        
        self.get_logger().info('Correct Speed Controller initialized')
        self.get_logger().info(f'Max speeds: linear={self.max_linear_velocity}m/s, angular={self.max_angular_velocity}rad/s')
        self.get_logger().info(f'Tolerances: position={self.position_tolerance}m, angle={math.degrees(self.angle_tolerance):.1f}°')
        
    def load_parameters(self):
        """从YAML配置文件加载参数"""
        try:
            # 获取包的配置文件路径
            package_share_directory = get_package_share_directory('chassis_position_controller')
            config_file_path = os.path.join(package_share_directory, 'config', 'position_controller.yaml')
            
            # 读取YAML配置文件
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                
            # 提取position_controller配置
            controller_config = config.get('position_controller', {})
            control_config = controller_config.get('control', {})
            pid_config = controller_config.get('pid', {})
            
            # 加载控制限制和容差
            self.max_linear_velocity = control_config.get('max_linear_velocity', 1.0)
            self.max_angular_velocity = control_config.get('max_angular_velocity', 1.5)
            self.position_tolerance = control_config.get('position_tolerance', 0.15)
            self.angle_tolerance = control_config.get('angle_tolerance', 0.15)
            self.control_frequency = control_config.get('control_frequency', 50.0)
            
            # 加载PID参数
            linear_pid = pid_config.get('linear', {})
            angular_pid = pid_config.get('angular', {})
            
            self.kp_linear = linear_pid.get('kp', 1.2)
            self.kp_angular = angular_pid.get('kp', 1.8)
            
            self.get_logger().info(f'✅ Config loaded: pos_tol={self.position_tolerance}m, ang_tol={math.degrees(self.angle_tolerance):.1f}°')
            
        except Exception as e:
            # 如果加载失败，使用默认值
            self.get_logger().warn(f'⚠️ Failed to load config: {e}, using defaults')
            self.max_linear_velocity = 1.0
            self.max_angular_velocity = 1.5
            self.position_tolerance = 0.15
            self.angle_tolerance = 0.15
            self.control_frequency = 50.0
            self.kp_linear = 1.2
            self.kp_angular = 1.8
        
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_pose = msg.pose.pose
        
    def target_pose_callback(self, msg):
        """Handle standard target pose"""
        self.target_pose = msg.pose
        self.target_linear_speed = None  # Use default
        self.target_angular_speed = None
        self.target_reached = False
        self.is_holding_position = False  # 重置保持位置状态
        self.last_cmd_time = time.time()
        
        self.get_logger().info(f'New target: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
    def target_with_speed_callback(self, msg):
        """Handle target with speed information"""
        from geometry_msgs.msg import Point, Quaternion
        
        # 检查是否是停止控制信号
        if (msg.header.frame_id == 'stop_control' or 
            (msg.twist.linear.x == -999.0 and msg.twist.linear.y == -999.0)):
            self.get_logger().info('Received stop control signal - disabling position controller')
            self.target_pose = None  # 清除目标，停止所有位置控制
            self.stop_robot()
            return
        
        # 检查是否是停止命令（所有速度为0）
        is_stop_command = (msg.twist.linear.x == 0.0 and msg.twist.linear.y == 0.0 and 
                          msg.twist.linear.z == 0.0 and msg.twist.angular.z == 0.0)
        
        if is_stop_command:
            # 收到停止命令，停止机器人但保持当前目标
            if not self.is_holding_position:  # 只在第一次收到停止命令时打印
                self.get_logger().info('Received stop command - holding current position')
                self.is_holding_position = True
            self.stop_robot()
            return
        
        # Create pose from twist message
        pose = type('Pose', (), {})()
        pose.position = Point()
        pose.position.x = msg.twist.linear.x
        pose.position.y = msg.twist.linear.y
        pose.position.z = 0.0
        
        yaw = msg.twist.angular.z
        pose.orientation = Quaternion()
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        
        # 检查是否是新目标（位置有变化）
        new_target = True
        if hasattr(self, 'last_target_pose') and self.last_target_pose is not None:
            dx = abs(pose.position.x - self.last_target_pose.position.x)
            dy = abs(pose.position.y - self.last_target_pose.position.y)
            if dx < 0.01 and dy < 0.01:  # 如果位置变化小于1cm，认为是同一目标
                new_target = False
        
        self.target_pose = pose
        self.target_linear_speed = abs(msg.twist.linear.z) if msg.twist.linear.z > 0 else None
        self.target_angular_speed = abs(msg.twist.angular.x) if msg.twist.angular.x > 0 else None
        self.target_reached = False
        self.is_holding_position = False  # 重置保持位置状态
        self.last_cmd_time = time.time()
        
        # 只有新目标时才打印日志
        if new_target:
            speed_info = f" @ {self.target_linear_speed:.2f}m/s" if self.target_linear_speed else ""
            self.get_logger().info(f'Speed target: ({pose.position.x:.2f}, {pose.position.y:.2f}, {math.degrees(yaw):.0f}°){speed_info}')
            self.last_target_pose = pose
        
    def speed_override_callback(self, msg):
        """Handle speed override"""
        if msg.data > 0:
            self.target_linear_speed = min(msg.data, self.max_linear_velocity)
            self.get_logger().info(f'Speed override: {self.target_linear_speed:.2f} m/s')
        else:
            self.target_linear_speed = None
            self.get_logger().info('Speed override cleared')
            
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def fuzzy_membership(self, value, center, width):
        """三角形模糊隶属度函数"""
        return max(0.0, 1.0 - abs(value - center) / width)
    
    def fuzzy_linear_control(self, distance_error, distance_change):
        """模糊线速度控制 - 激进版本"""
        # 距离误差模糊化: 更激进的区间划分
        near = self.fuzzy_membership(distance_error, 0.0, 0.25)    # 缩小"近"区间，更早加速
        medium = self.fuzzy_membership(distance_error, 0.4, 0.25)  # 缩小中等区间
        far = self.fuzzy_membership(distance_error, 1.0, 0.6)      # 缩小"远"区间，更早高速
        
        # 距离变化模糊化: 更敏感的变化检测
        approaching = self.fuzzy_membership(distance_change, -0.08, 0.08)  # 更敏感
        stable = self.fuzzy_membership(distance_change, 0.0, 0.08)
        departing = self.fuzzy_membership(distance_change, 0.08, 0.08)
        
        # 更激进的模糊规则推理
        slow = near * approaching  # 只有接近时才慢速 (修复：去掉不必要的max)
        medium_speed = max(near * stable, medium * approaching)  # 减少中速条件
        fast = max(far * stable, far * departing, far * approaching, 
                  medium * stable, medium * departing, near * departing)  # 增加快速条件
        
        # 更激进的解模糊化 (更倾向于高速)
        total_weight = slow + medium_speed + fast
        if total_weight > 0:
            speed_factor = (slow * 0.4 + medium_speed * 0.7 + fast * 1.0) / total_weight  # 提高各档速度
        else:
            speed_factor = 0.7  # 默认较高速度
            
        return speed_factor
    
    def fuzzy_angular_control(self, heading_error, angle_change):
        """模糊角速度控制 - 激进版本"""
        abs_heading = abs(heading_error)
        
        # 更激进的角度误差模糊化: 更早进入高速转向
        small_angle = self.fuzzy_membership(abs_heading, 0.0, math.radians(10))     # 缩小小角度范围
        medium_angle = self.fuzzy_membership(abs_heading, math.radians(20), math.radians(15))  # 提前中等角度
        large_angle = self.fuzzy_membership(abs_heading, math.radians(40), math.radians(25))   # 提前大角度
        
        # 更敏感的角度变化模糊化
        decreasing = self.fuzzy_membership(angle_change, -0.08, 0.08)
        stable = self.fuzzy_membership(angle_change, 0.0, 0.08)
        increasing = self.fuzzy_membership(angle_change, 0.08, 0.08)
        
        # 更激进的模糊规则推理
        slow_turn = small_angle * decreasing  # 只有角度减小时才慢转 (修复：去掉不必要的max)
        medium_turn = max(small_angle * stable, medium_angle * decreasing)  # 减少中等转速条件
        fast_turn = max(large_angle * stable, large_angle * increasing, large_angle * decreasing,
                       medium_angle * stable, medium_angle * increasing,
                       small_angle * increasing)  # 大幅增加快速转向条件
        
        # 更激进的解模糊化
        total_weight = slow_turn + medium_turn + fast_turn
        if total_weight > 0:
            angular_factor = (slow_turn * 0.4 + medium_turn * 0.7 + fast_turn * 1.0) / total_weight  # 提高各档角速度
        else:
            angular_factor = 0.7  # 默认较高角速度
            
        return angular_factor
        
    def control_callback(self):
        """Main control loop with correct speed control"""
        if not self.current_pose or not self.target_pose:
            return
            
        # Calculate position errors
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Calculate yaw angles
        target_q = self.target_pose.orientation
        target_yaw = math.atan2(
            2 * (target_q.w * target_q.z + target_q.x * target_q.y),
            1 - 2 * (target_q.y**2 + target_q.z**2)
        )
        
        current_q = self.current_pose.orientation
        current_yaw = math.atan2(
            2 * (current_q.w * current_q.z + current_q.x * current_q.y),
            1 - 2 * (current_q.y**2 + current_q.z**2)
        )
        
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # Check if target reached
        position_reached = distance_error < self.position_tolerance
        angle_reached = abs(yaw_error) < self.angle_tolerance
        
        if position_reached and angle_reached:
            if not self.target_reached:
                self.target_reached = True
                self.get_logger().info('🎯 Target reached!')
            self.stop_robot()
            self.publish_status()
            return
            
        self.target_reached = False
        
        # === 模糊PID控制实现 ===
        
        linear_vel = 0.0
        angular_vel = 0.0
        
        # 计算误差变化率
        distance_change = distance_error - self.prev_distance_error
        self.prev_distance_error = distance_error
        
        # Position control with fuzzy logic
        if distance_error > self.position_tolerance:
            # Calculate direction to target
            angle_to_target = math.atan2(dy, dx)
            heading_error = self.normalize_angle(angle_to_target - current_yaw)
            
            # 计算角度误差变化率
            angle_change = abs(heading_error) - abs(self.prev_heading_error)
            self.prev_heading_error = heading_error
            
            # 使用模糊控制确定速度因子
            linear_factor = self.fuzzy_linear_control(distance_error, distance_change)
            angular_factor = self.fuzzy_angular_control(heading_error, angle_change)
            
            # 确定基础期望速度
            if self.target_linear_speed is not None:
                base_speed = self.target_linear_speed
            else:
                base_speed = self.max_linear_velocity
            
            # 应用模糊控制的速度调节
            desired_speed = base_speed * linear_factor
            
            # 基于航向误差的运动策略 - 激进版本
            abs_heading_error = abs(heading_error)
            
            if abs_heading_error < math.radians(50):  # 扩大前进角度范围到50度
                # 更激进的距离调节 - 缩小减速区间
                if distance_error < 0.25:  # 缩小减速区间从0.4到0.25
                    desired_speed *= max(0.6, distance_error / 0.25)  # 提高最小速度从0.4到0.6
                
                linear_vel = desired_speed
                
                # 更敏感的航向修正
                if abs_heading_error > math.radians(3):  # 降低修正阈值从5度到3度
                    max_angular = self.max_angular_velocity * angular_factor
                    angular_vel = self.kp_angular * heading_error * angular_factor
                    angular_vel = max(-max_angular, min(max_angular, angular_vel))
                    
            elif abs_heading_error < math.radians(100):  # 扩大转向+前进范围到100度
                # 主要进行转向
                max_angular = self.max_angular_velocity * angular_factor
                angular_vel = self.kp_angular * heading_error * angular_factor
                angular_vel = max(-max_angular, min(max_angular, angular_vel))
                
                # 提高前进速度避免过慢
                linear_vel = desired_speed * 0.6  # 提高从0.4到0.6
                
            else:  # 100度以上 - 纯转向（缩小纯转向范围）
                max_angular = self.max_angular_velocity * angular_factor
                angular_vel = self.kp_angular * heading_error * angular_factor
                angular_vel = max(-max_angular, min(max_angular, angular_vel))
                linear_vel = 0.0
        
        # 最终朝向控制（到达位置后的精确朝向）- 激进版本
        if distance_error <= self.position_tolerance and abs(yaw_error) > self.angle_tolerance:
            # 使用模糊控制进行精确朝向调节
            angle_change = abs(yaw_error) - abs(self.prev_heading_error)
            angular_factor = self.fuzzy_angular_control(yaw_error, angle_change)
            
            max_angular = self.max_angular_velocity * angular_factor * 0.8  # 提高精确朝向速度从0.6到0.8
            yaw_control = self.kp_angular * yaw_error * angular_factor
            
            # 缩小减速区间 - 更晚才减速
            if abs(yaw_error) < math.radians(15):  # 缩小从20度到15度
                max_angular *= (abs(yaw_error) / math.radians(15))
            
            angular_vel = max(-max_angular, min(max_angular, yaw_control))
        
        # Apply absolute limits as safety
        linear_vel = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_vel))
        angular_vel = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_vel))
        
        # Create and publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(cmd)
        
        # Publish current speed
        current_speed = Float32()
        current_speed.data = abs(linear_vel)
        self.current_speed_pub.publish(current_speed)
        
        # Debug output (every few seconds) - 增加模糊控制信息
        if int(time.time() * 5) % 25 == 0:  # Every 5 seconds
            if distance_error > self.position_tolerance:
                heading_error = self.normalize_angle(math.atan2(dy, dx) - current_yaw)
                linear_factor = self.fuzzy_linear_control(distance_error, distance_change)
                angular_factor = self.fuzzy_angular_control(heading_error, 0)
                self.get_logger().info(f'Fuzzy Control: dist={distance_error:.3f}m, head_err={math.degrees(heading_error):.1f}°, ' +
                                     f'L_factor={linear_factor:.2f}, A_factor={angular_factor:.2f}, cmd=({linear_vel:.3f}, {angular_vel:.3f})')
            else:
                self.get_logger().info(f'Final Orientation: yaw_err={math.degrees(yaw_error):.1f}°, cmd=({linear_vel:.3f}, {angular_vel:.3f})')
        
        self.publish_status()
        
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def publish_status(self):
        """Publish status"""
        status = Bool()
        status.data = self.target_reached
        self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = CorrectSpeedController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
