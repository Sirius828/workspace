#!/usr/bin/env python3
"""
简化版云台轨迹规划器（支持轨迹插值）
功能：
1. 接收目标位置坐标 (geometry_msgs/Point)
2. 接收中线点坐标并生成平滑轨迹
3. 通过FOV或相机内参将像素坐标转换为云台角度
4. 发布关节状态到云台控制器

作者: Your Name
日期: 2025-07-13
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import JointState
import math
import time
import numpy as np
from collections import deque

class SimpleGimbalNode(Node):
    def __init__(self):
        super().__init__('simple_gimbal_node')

        # 声明参数
        self.declare_parameter('input_topic', '/target_position')
        self.declare_parameter('corner_points_topic', '/corner_points')  # 支持角点模式
        self.declare_parameter('midline_points_topic', '/midline_points')  # 支持中线点轨迹
        self.declare_parameter('joint_states_topic', '/gimbal_joint_states')
        self.declare_parameter('use_camera_info', False)
        self.declare_parameter('fov_horizontal', 90.0)  # deg
        self.declare_parameter('fov_vertical', 65.0)    # deg
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('camera_fx', 554.254)     # 如果 use_camera_info=true
        self.declare_parameter('camera_fy', 554.254)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)
        self.declare_parameter('yaw_min', -180.0)
        self.declare_parameter('yaw_max',  180.0)
        self.declare_parameter('pitch_min', -45.0)
        self.declare_parameter('pitch_max',  45.0)
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('log_angles', True)
        self.declare_parameter('corner_mode_enabled', False)  # 是否启用角点模式
        
        # 轨迹插值参数
        self.declare_parameter('trajectory_enabled', True)      # 是否启用轨迹插值
        self.declare_parameter('trajectory_frequency', 50.0)    # 轨迹更新频率 (Hz)
        self.declare_parameter('trajectory_buffer_size', 10)    # 轨迹点缓存大小
        self.declare_parameter('smoothing_factor', 0.3)         # 平滑因子 (0-1)
        self.declare_parameter('interpolation_steps', 5)        # 插值步数
        self.declare_parameter('trajectory_rate', 20.0)  # Hz for interpolation output

        # 读取参数
        p = self.get_parameters([
            'input_topic', 'corner_points_topic', 'midline_points_topic', 'joint_states_topic',
            'use_camera_info',
            'fov_horizontal','fov_vertical',
            'image_width','image_height',
            'camera_fx','camera_fy','camera_cx','camera_cy',
            'yaw_min','yaw_max','pitch_min','pitch_max',
            'debug_mode', 'log_angles', 'corner_mode_enabled',
            'trajectory_enabled', 'trajectory_frequency', 'trajectory_buffer_size', 
            'smoothing_factor', 'interpolation_steps', 'trajectory_rate'
        ])
        self.input_topic = p[0].value
        self.corner_points_topic = p[1].value
        self.midline_points_topic = p[2].value
        self.joint_states_topic = p[3].value
        self.use_cam_info = p[4].value
        self.fov_h = math.radians(p[5].value)
        self.fov_v = math.radians(p[6].value)
        self.img_w = p[7].value
        self.img_h = p[8].value
        self.fx = p[9].value
        self.fy = p[10].value
        self.cx = p[11].value
        self.cy = p[12].value
        self.yaw_min = p[13].value
        self.yaw_max = p[14].value
        self.pitch_min = p[15].value
        self.pitch_max = p[16].value
        self.debug_mode = p[17].value
        self.log_angles = p[18].value
        self.corner_mode_enabled = p[19].value
        self.trajectory_enabled = p[20].value
        self.trajectory_frequency = p[21].value
        self.trajectory_buffer_size = p[22].value
        self.smoothing_factor = p[23].value
        self.interpolation_steps = p[24].value
        self.trajectory_rate = p[25].value

        # 发布者与订阅者
        self.joint_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        
        # 单点模式订阅者
        self.point_sub = self.create_subscription(Point, self.input_topic, self.point_callback, 10)
        
        # Subscribe midline for all trajectories
        self.midline_sub = self.create_subscription(Point32, self.midline_points_topic, self.midline_callback, 10)
        # Trajectory timer at configured rate
        self.trajectory_timer = self.create_timer(1.0/self.trajectory_rate, self.update_trajectory)
        self.trajectory_buffer = deque(maxlen=self.trajectory_buffer_size)
        self.current_target = None
        self.last_target = None
        
        # 角点模式订阅者 (可选)
        if self.corner_mode_enabled:
            self.corner_sub = self.create_subscription(Point32, self.corner_points_topic, self.corner_callback, 10)
            self.corner_points = []
            self.corner_index = 0
            self.corner_timer = None

        # 状态变量
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.message_count = 0

        # 状态报告定时器
        if self.debug_mode:
            self.status_timer = self.create_timer(5.0, self.report_status)

        self.get_logger().info(f'简化版云台节点启动完成')
        self.get_logger().info(f'单点模式: 监听 {self.input_topic}')
        if self.trajectory_enabled:
            self.get_logger().info(f'轨迹模式: 监听 {self.midline_points_topic}')
        if self.corner_mode_enabled:
            self.get_logger().info(f'角点模式: 监听 {self.corner_points_topic}')
        self.get_logger().info(f'发布到: {self.joint_states_topic}')
        self.get_logger().info(f'转换方法: {"相机内参" if self.use_cam_info else "FOV近似"}')
        if self.trajectory_enabled:
            self.get_logger().info(f'轨迹插值: 启用 (频率: {self.trajectory_frequency}Hz, 平滑: {self.smoothing_factor})')

    def pixel_to_angle(self, x, y):
        """将像素坐标转换为云台角度"""
        if self.use_cam_info:
            # 针孔相机模型
            x_norm = (x - self.cx) / self.fx
            y_norm = (y - self.cy) / self.fy
            yaw = math.degrees(math.atan(x_norm))
            pitch = math.degrees(math.atan(-y_norm))  # Y轴向下为正，云台向上为正
        else:
            # FOV 近似方法
            x_ratio = (x - self.img_w/2) / (self.img_w/2)
            y_ratio = (y - self.img_h/2) / (self.img_h/2)
            yaw = math.degrees(x_ratio * (self.fov_h/2))
            pitch = math.degrees(-y_ratio * (self.fov_v/2))

        # 限制角度范围
        yaw = max(self.yaw_min, min(self.yaw_max, yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, pitch))

        return yaw, pitch

    def publish_joint_state(self, yaw_deg, pitch_deg):
        """发布关节状态"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'gimbal_base_link'
        js.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        js.position = [math.radians(yaw_deg), math.radians(pitch_deg)]
        js.velocity = [0.0, 0.0]
        js.effort = [0.0, 0.0]
        
        self.joint_pub.publish(js)
        
        # 更新状态
        self.last_yaw = yaw_deg
        self.last_pitch = pitch_deg
        self.message_count += 1

        if self.log_angles:
            self.get_logger().info(f'云台角度: Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°')

    def point_callback(self, msg: Point):
        """处理单点目标位置"""
        # Enqueue single-point target for interpolation
        yaw, pitch = self.pixel_to_angle(msg.x, msg.y)
        self.trajectory_buffer.append((yaw, pitch))
        if self.debug_mode:
            self.get_logger().info(f'Enqueued single point target: ({yaw:.2f}°, {pitch:.2f}°)')

    def midline_callback(self, msg: Point32):
        """处理中线点坐标并生成轨迹"""
        if not self.trajectory_enabled:
            return
            
        # 转换为云台角度
        yaw, pitch = self.pixel_to_angle(msg.x, msg.y)
        
        # 更新目标位置
        new_target = (yaw, pitch)
        
        if self.current_target is None:
            # 第一个目标点，直接设置
            self.current_target = new_target
            self.last_target = new_target
            if self.debug_mode:
                self.get_logger().info(f'接收首个中线点: ({msg.x:.1f}, {msg.y:.1f}) → 角度: ({yaw:.2f}°, {pitch:.2f}°)')
        else:
            # 将新目标添加到缓存
            self.trajectory_buffer.append(new_target)
            if self.debug_mode:
                self.get_logger().debug(f'缓存中线点: ({msg.x:.1f}, {msg.y:.1f}) → 角度: ({yaw:.2f}°, {pitch:.2f}°)')

    def update_trajectory(self):
        """更新轨迹并发布平滑插值的关节状态"""
        if not self.trajectory_enabled or self.current_target is None:
            return
            
        # 检查是否有新的目标点
        if len(self.trajectory_buffer) > 0:
            # 获取下一个目标点
            next_target = self.trajectory_buffer.popleft()
            
            # 计算当前位置到目标位置的插值
            current_yaw, current_pitch = self.last_target
            target_yaw, target_pitch = next_target
            
            # 使用平滑因子进行插值
            smooth_yaw = current_yaw + self.smoothing_factor * (target_yaw - current_yaw)
            smooth_pitch = current_pitch + self.smoothing_factor * (target_pitch - current_pitch)
            
            # 更新当前位置
            self.last_target = (smooth_yaw, smooth_pitch)
            self.current_target = next_target
            
            # 发布插值后的关节状态
            self.publish_joint_state(smooth_yaw, smooth_pitch)
            
            if self.debug_mode and self.message_count % int(self.trajectory_rate/2) == 0:  # 每0.5秒记录一次
                self.get_logger().debug(f'轨迹插值: 当前({smooth_yaw:.2f}°, {smooth_pitch:.2f}°) → 目标({target_yaw:.2f}°, {target_pitch:.2f}°)')
        else:
            # 没有新目标，继续向当前目标移动
            if self.last_target != self.current_target:
                current_yaw, current_pitch = self.last_target
                target_yaw, target_pitch = self.current_target
                
                # 计算距离
                yaw_diff = abs(target_yaw - current_yaw)
                pitch_diff = abs(target_pitch - current_pitch)
                
                # 如果距离很小，认为已到达
                if yaw_diff < 0.1 and pitch_diff < 0.1:
                    self.last_target = self.current_target
                else:
                    # 继续向目标移动
                    smooth_yaw = current_yaw + self.smoothing_factor * (target_yaw - current_yaw)
                    smooth_pitch = current_pitch + self.smoothing_factor * (target_pitch - current_pitch)
                    
                    self.last_target = (smooth_yaw, smooth_pitch)
                    self.publish_joint_state(smooth_yaw, smooth_pitch)

    def corner_callback(self, msg: Point32):
        """处理角点坐标 (用于矩形轨迹)"""
        if not self.corner_mode_enabled:
            return
            
        # 收集角点
        self.corner_points.append((msg.x, msg.y))
        self.get_logger().info(f'接收角点 #{len(self.corner_points)}: ({msg.x:.1f}, {msg.y:.1f})')
        
        # 如果收集到5个角点，开始循环轨迹
        if len(self.corner_points) == 5:
            # Enqueue all corner targets for one loop of rectangle
            for x,y in self.corner_points:
                self.trajectory_buffer.append(self.pixel_to_angle(x, y))
            # Close the loop by re-adding the first
            first = self.corner_points[0]
            self.trajectory_buffer.append(self.pixel_to_angle(first[0], first[1]))
            if self.debug_mode:
                self.get_logger().info(f'Enqueued corner loop of {len(self.corner_points)+1} points for interpolation')

    def start_corner_trajectory(self):
        """开始角点循环轨迹"""
        if self.corner_timer is not None:
            self.corner_timer.cancel()
        
        self.corner_index = 0
        # 每2秒移动到下一个角点
        self.corner_timer = self.create_timer(2.0, self.move_to_next_corner)
        self.move_to_next_corner()  # 立即移动到第一个点

    def move_to_next_corner(self):
        """移动到下一个角点"""
        if not self.corner_points:
            return
            
        # 获取当前角点
        x, y = self.corner_points[self.corner_index]
        yaw, pitch = self.pixel_to_angle(x, y)
        
        self.get_logger().info(f'移动到角点 {self.corner_index + 1}/4: ({x:.1f}, {y:.1f})')
        self.publish_joint_state(yaw, pitch)
        
        # 移动到下一个角点
        self.corner_index = (self.corner_index + 1) % len(self.corner_points)

    def report_status(self):
        """报告节点状态"""
        self.get_logger().info(f'状态报告: 已处理 {self.message_count} 条消息, '
                              f'当前角度: Yaw={self.last_yaw:.2f}°, Pitch={self.last_pitch:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGimbalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()