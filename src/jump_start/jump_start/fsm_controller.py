#!/usr/bin/env python3
"""
简单的FSM（有限状态机）机器人控制节点
功能：
1. 导航到指定位置
2. 视觉目标跟踪
3. 异常状态处理
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from enum import Enum
import math
import time
import os
import cv2
import numpy as np
from collections import deque
from typing import Optional, Tuple

from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, TwistStamped, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class FSMState(Enum):
    """FSM状态定义"""
    IDLE = "IDLE"                   # 空闲状态
    NAVIGATING = "NAVIGATING"       # 导航中
    TARGET_TRACKING = "TARGET_TRACKING"  # 目标跟踪
    SEARCHING = "SEARCHING"         # 搜索目标状态（旋转搜索）
    VICTORY = "VICTORY"             # 成功状态
    ERROR = "ERROR"                 # 错误状态
    EMERGENCY_STOP = "EMERGENCY_STOP"   # 紧急停止


class SimpleRobotFSM(Node):
    """简单机器人FSM控制器"""
    
    def __init__(self):
        super().__init__('simple_robot_fsm')
        
        # 声明参数
        self.declare_parameter('position_tolerance', 0.3)      # 位置容差 (m)
        self.declare_parameter('pixel_tolerance', 50)          # 像素容差
        self.declare_parameter('victory_time_threshold', 2.0)  # 胜利时间阈值 (s)
        self.declare_parameter('image_width', 640)             # 图像宽度
        self.declare_parameter('image_height', 480)            # 图像高度
        self.declare_parameter('navigation_timeout', 30.0)     # 导航超时 (s)
        self.declare_parameter('valid_tracking_x', 2.0)        # 有效跟踪的x坐标阈值 (m)
        self.declare_parameter('map_size', 4.0)                # 地图尺寸 (4x4米)
        
        # 目标位置参数
        self.declare_parameter('target_position.x', 2.0)       # 目标位置x坐标
        self.declare_parameter('target_position.y', 0.0)       # 目标位置y坐标  
        self.declare_parameter('target_position.z', 0.5)       # 目标位置z坐标
        
        # 视频录制参数
        self.declare_parameter('enable_video_recording', True)  # 是否启用视频录制
        self.declare_parameter('video_buffer_duration', 10.0)  # 视频缓存时长 (s)
        self.declare_parameter('video_fps', 30.0)              # 视频帧率
        self.declare_parameter('camera_topic', '/camera/image_raw')  # 相机话题
        self.declare_parameter('yolo_topic', '/yolo_detection_result')  # YOLO话题
        
        # 搜索参数
        self.declare_parameter('enable_search_rotation', True)  # 是否启用搜索旋转
        self.declare_parameter('target_pixel_timeout', 1.5)    # 目标像素更新超时 (s)
        self.declare_parameter('search_angular_velocity', 1.0) # 搜索角速度 (rad/s)
        self.declare_parameter('max_search_duration', 10.0)    # 最大搜索持续时间 (s)
        
        # 获取参数
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.pixel_tolerance = self.get_parameter('pixel_tolerance').value
        self.victory_time_threshold = self.get_parameter('victory_time_threshold').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.valid_tracking_x = self.get_parameter('valid_tracking_x').value
        self.map_size = self.get_parameter('map_size').value
        
        # 目标位置参数
        self.target_x = self.get_parameter('target_position.x').value
        self.target_y = self.get_parameter('target_position.y').value
        self.target_z = self.get_parameter('target_position.z').value
        
        # 视频录制参数
        self.enable_video_recording = self.get_parameter('enable_video_recording').value
        self.video_buffer_duration = self.get_parameter('video_buffer_duration').value
        self.video_fps = self.get_parameter('video_fps').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.yolo_topic = self.get_parameter('yolo_topic').value
        
        # 搜索参数
        self.enable_search_rotation = self.get_parameter('enable_search_rotation').value
        self.target_pixel_timeout = self.get_parameter('target_pixel_timeout').value
        self.search_angular_velocity = self.get_parameter('search_angular_velocity').value
        self.max_search_duration = self.get_parameter('max_search_duration').value
        
        # 计算图像中心
        self.image_center_x = self.image_width / 2.0
        self.image_center_y = self.image_height / 2.0
        
        # FSM状态
        self.current_state = FSMState.IDLE
        self.previous_state = FSMState.IDLE
        
        # 机器人状态
        self.current_pose: Optional[Pose] = None
        self.target_pose: Optional[Pose] = None
        self.target_pixel: Optional[Point] = None
        
        # 时间管理
        self.state_entry_time = time.time()
        self.target_in_center_start_time: Optional[float] = None
        self.victory_accumulated_time = 0.0  # 累计有效打击时间（仅统计，不用于胜利判定）
        self.current_hit_session_start: Optional[float] = None  # 当前连续打击开始时间
        self.last_target_pixel_time: Optional[float] = None  # 最后收到目标像素的时间
        
        # 标志位
        self.navigation_active = False
        self.target_detected = False
        self.emergency_stop_requested = False
        self.in_valid_tracking_zone = False  # 是否在有效跟踪区域（x>2）
        self.is_currently_hitting = False    # 当前是否在有效打击中
        self.is_searching = False            # 当前是否在搜索状态
        self.last_command_type = None        # 添加：跟踪最后发送的命令类型
        
        # 视频录制相关
        self.cv_bridge = CvBridge() if self.enable_video_recording else None
        self.video_buffer = deque(maxlen=int(self.video_buffer_duration * self.video_fps)) if self.enable_video_recording else None
        
        if self.enable_video_recording:
            self.video_save_path = os.path.join(get_package_share_directory('jump_start'), 'video')
            os.makedirs(self.video_save_path, exist_ok=True)
        else:
            self.video_save_path = None
        
        # QoS配置
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        qos_best_effort = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            '/chassis/odom',
            self.odom_callback,
            qos_best_effort
        )
        
        self.target_pixel_sub = self.create_subscription(
            Point,
            '/target_position_pixel',
            self.target_pixel_callback,
            qos_best_effort
        )
        
        self.mission_command_sub = self.create_subscription(
            String,
            '/mission_command',
            self.mission_command_callback,
            qos_reliable
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            qos_reliable
        )
        
        # YOLO检测结果订阅（用于视频录制触发）
        if self.enable_video_recording:
            self.yolo_detection_sub = self.create_subscription(
                String,  # 假设YOLO结果是String类型，根据实际情况调整
                self.yolo_topic,
                self.yolo_detection_callback,
                qos_best_effort
            )
            
            # 图像订阅（用于视频录制）
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                qos_best_effort
            )
        else:
            self.yolo_detection_sub = None
            self.image_sub = None
        
        # 发布者
        self.target_pose_pub = self.create_publisher(
            TwistStamped,
            '/target_pose_with_speed',
            qos_reliable
        )
        
        # 直接速度控制发布者（用于搜索旋转）
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )
        
        self.victory_pub = self.create_publisher(
            Bool,
            '/victory',
            qos_reliable
        )
        
        self.state_pub = self.create_publisher(
            String,
            '/fsm_state',
            qos_reliable
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/fsm_status',
            qos_reliable
        )
        
        # 服务
        self.start_service = self.create_service(
            Empty,
            '/start',
            self.start_service_callback
        )
        
        # 定时器 - FSM主循环
        self.fsm_timer = self.create_timer(0.1, self.fsm_update)  # 10Hz
        
        # 定时器 - 状态发布
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1Hz
        
        # 定时器 - 话题监控（检查订阅状态）
        self.topic_monitor_timer = self.create_timer(10.0, self.monitor_topics)  # 每10秒检查一次
        
        self.get_logger().info('Simple Robot FSM Controller started')
        self.get_logger().info(f'Position tolerance: {self.position_tolerance}m')
        self.get_logger().info(f'Pixel tolerance: {self.pixel_tolerance}px')
        self.get_logger().info(f'Victory time threshold: {self.victory_time_threshold}s')
        self.get_logger().info(f'Target position: ({self.target_x}, {self.target_y}, {self.target_z})')
        self.get_logger().info(f'Valid tracking zone: x >= {self.valid_tracking_x}m')
        self.get_logger().info(f'Video recording: {"enabled" if self.enable_video_recording else "disabled"}')
        if self.enable_video_recording:
            self.get_logger().info(f'Video config: {self.video_buffer_duration}s @ {self.video_fps}fps')
            self.get_logger().info(f'Camera topic: {self.camera_topic}')
            self.get_logger().info(f'YOLO topic: {self.yolo_topic}')
        self.get_logger().info(f'Search rotation: {"enabled" if self.enable_search_rotation else "disabled"}')
        if self.enable_search_rotation:
            self.get_logger().info(f'Search config: timeout={self.target_pixel_timeout}s, ω={self.search_angular_velocity}rad/s, max_duration={self.max_search_duration}s')
        self.get_logger().info('🚀 /start service ready - compatible with start_engine package')
    
    def start_service_callback(self, request, response):
        """处理 /start 服务调用（与 start_engine 包适配）"""
        self.get_logger().info('🚀 Received /start service call from start_engine GUI')
        
        try:
            # 执行默认的搜索和跟踪任务
            self.start_default_mission()
            
            self.get_logger().info('✅ Default mission started successfully')
            return response
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to start mission: {e}')
            return response
        self.get_logger().info(f'Valid tracking zone: x > {self.valid_tracking_x}m')
        self.get_logger().info(f'Map size: {self.map_size}x{self.map_size}m')
    
    def odom_callback(self, msg: Odometry):
        """里程计回调函数"""
        self.current_pose = msg.pose.pose
        
        # 检查是否在有效跟踪区域（通过半场）
        if self.current_pose is not None:
            current_x = self.current_pose.position.x
            was_in_zone = self.in_valid_tracking_zone
            self.in_valid_tracking_zone = current_x >= self.valid_tracking_x  # 修复：改为 >= 包含边界情况
            
            # 如果刚进入或离开有效区域，记录日志
            if was_in_zone != self.in_valid_tracking_zone:
                if self.in_valid_tracking_zone:
                    self.get_logger().info(f'🎯 Entered valid tracking zone (x={current_x:.2f} >= {self.valid_tracking_x})')
                else:
                    self.get_logger().info(f'🚫 Left valid tracking zone (x={current_x:.2f} < {self.valid_tracking_x})')
                    # 如果离开有效区域，停止当前打击会话，但不清零累计时间
                    if self.current_state == FSMState.TARGET_TRACKING:
                        self.is_currently_hitting = False
                        self.current_hit_session_start = None
    
    def target_pixel_callback(self, msg: Point):
        """目标像素位置回调函数"""
        self.target_pixel = msg
        self.target_detected = True
        self.last_target_pixel_time = time.time()  # 记录接收时间
        
        # 添加目标检测调试信息
        current_x = self.current_pose.position.x if self.current_pose else 0.0
        self.get_logger().info(f'🎯 Target detected at pixel ({msg.x:.0f}, {msg.y:.0f}), robot at x={current_x:.2f}m', throttle_duration_sec=3.0)
        
        # 如果正在搜索状态，收到目标后退出搜索
        if self.current_state == FSMState.SEARCHING:
            self.get_logger().info('🎯 Target found during search, returning to tracking')
            self.transition_to_state(FSMState.TARGET_TRACKING)
            return
        
        # 计算目标是否在图像中心
        distance_to_center = math.sqrt(
            (msg.x - self.image_center_x) ** 2 + 
            (msg.y - self.image_center_y) ** 2
        )
        
        # 只有在有效跟踪区域内且目标在中心时才累计打击时间
        target_in_center = distance_to_center <= self.pixel_tolerance
        
        # 添加目标位置调试信息
        center_status = "IN CENTER" if target_in_center else "OFF CENTER"
        zone_status = "VALID ZONE" if self.in_valid_tracking_zone else "INVALID ZONE"
        self.get_logger().info(f'Target status: {center_status} (dist={distance_to_center:.1f}px), {zone_status}', throttle_duration_sec=3.0)
        
        if target_in_center and self.in_valid_tracking_zone:
            if not self.is_currently_hitting:
                # 开始新的打击会话
                self.is_currently_hitting = True
                self.current_hit_session_start = time.time()
                current_x = self.current_pose.position.x if self.current_pose else 0.0
                self.get_logger().info(f'🎯 Started hitting session (distance: {distance_to_center:.1f}px, x={current_x:.2f}m)')
        else:
            if self.is_currently_hitting:
                # 结束当前打击会话，累加到总时间
                if self.current_hit_session_start is not None:
                    session_time = time.time() - self.current_hit_session_start
                    self.victory_accumulated_time += session_time
                    reason = "target left center" if not target_in_center else "left valid zone"
                    self.get_logger().info(f'⏹️ Hit session ended: {reason}, session: {session_time:.1f}s, total accumulated: {self.victory_accumulated_time:.1f}s (victory requires {self.victory_time_threshold}s consecutive)')
                
                self.is_currently_hitting = False
                self.current_hit_session_start = None
    
    def yolo_detection_callback(self, msg: String):
        """YOLO检测结果回调函数"""
        if not self.enable_video_recording:
            return
            
        # 当检测到目标时，触发视频保存
        if "target" in msg.data.lower() or "detected" in msg.data.lower():
            self.save_video_buffer()
    
    def image_callback(self, msg: Image):
        """图像回调函数，用于视频录制"""
        if not self.enable_video_recording or self.cv_bridge is None or self.video_buffer is None:
            return
            
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            # 添加时间戳
            timestamp = time.time()
            self.video_buffer.append((cv_image.copy(), timestamp))
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def save_video_buffer(self):
        """保存视频缓存的最后10秒"""
        if not self.enable_video_recording or self.video_buffer is None or len(self.video_buffer) == 0:
            return
            
        try:
            current_time = time.time()
            # 过滤指定时长的帧
            recent_frames = [(frame, ts) for frame, ts in self.video_buffer 
                           if current_time - ts <= self.video_buffer_duration]
            
            if len(recent_frames) == 0:
                return
                
            # 创建视频文件
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            video_filename = f"detection_{timestamp_str}.mp4"
            video_path = os.path.join(self.video_save_path, video_filename)
            
            # 获取帧尺寸
            height, width = recent_frames[0][0].shape[:2]
            
            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = min(self.video_fps, len(recent_frames) / self.video_buffer_duration)
            out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            
            # 写入帧
            for frame, _ in recent_frames:
                out.write(frame)
            
            out.release()
            self.get_logger().info(f'📹 Saved video: {video_filename} ({len(recent_frames)} frames)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save video: {e}')
    
    def mission_command_callback(self, msg: String):
        """任务命令回调函数"""
        command = msg.data.lower()
        
        if command.startswith('navigate'):
            # 解析导航命令: "navigate x y z"
            parts = command.split()
            if len(parts) >= 4:
                try:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    self.start_navigation(x, y, z)
                except ValueError:
                    self.get_logger().error(f'Invalid navigation command: {command}')
            else:
                self.get_logger().error(f'Invalid navigation command format: {command}')
        
        elif command == 'track':
            self.start_target_tracking()
        
        elif command == 'stop':
            self.stop_mission()
        
        elif command == 'reset':
            self.reset_fsm()
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def emergency_stop_callback(self, msg: Bool):
        """紧急停止回调函数"""
        if msg.data:
            self.emergency_stop_requested = True
            self.get_logger().warn('Emergency stop requested!')
    
    def start_navigation(self, x: float, y: float, z: float):
        """开始导航到指定位置"""
        if self.current_state in [FSMState.IDLE, FSMState.VICTORY, FSMState.ERROR]:
            # 创建目标姿态
            self.target_pose = Pose()
            self.target_pose.position.x = x
            self.target_pose.position.y = y
            self.target_pose.position.z = z
            
            self.transition_to_state(FSMState.NAVIGATING)
            self.get_logger().info(f'Starting navigation to ({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            self.get_logger().warn(f'Cannot start navigation in state: {self.current_state.value}')
    
    def start_target_tracking(self):
        """开始目标跟踪"""
        if self.current_state in [FSMState.IDLE, FSMState.VICTORY, FSMState.ERROR]:
            self.transition_to_state(FSMState.TARGET_TRACKING)
            self.get_logger().info('Starting target tracking')
        else:
            self.get_logger().warn(f'Cannot start tracking in state: {self.current_state.value}')
    
    def stop_mission(self):
        """停止当前任务"""
        if self.current_state != FSMState.IDLE:
            self.transition_to_state(FSMState.IDLE)
            self.get_logger().info('Mission stopped')
    
    def reset_fsm(self):
        """重置FSM"""
        self.transition_to_state(FSMState.IDLE)
        self.target_pose = None
        self.target_pixel = None
        self.navigation_active = False
        self.target_detected = False
        self.emergency_stop_requested = False
        self.in_valid_tracking_zone = False
        self.is_currently_hitting = False
        self.current_hit_session_start = None
        self.victory_accumulated_time = 0.0  # 重置时清零累计时间
        self.last_target_pixel_time = None
        self.is_searching = False
        if self.enable_video_recording and self.video_buffer is not None:
            self.video_buffer.clear()
        self.get_logger().info('FSM reset')
    
    def start_default_mission(self):
        """启动默认任务（导航到指定位置）"""
        if self.current_state not in [FSMState.IDLE, FSMState.VICTORY, FSMState.ERROR]:
            self.get_logger().warn('Cannot start mission - system is busy')
            return False
        
        # 导航到单一目标位置
        self.get_logger().info('🎯 Starting navigation to target position')
        
        # 从配置文件读取目标位置
        self.start_navigation(self.target_x, self.target_y, self.target_z)
        
        return True
    
    def transition_to_state(self, new_state: FSMState):
        """状态转换"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_entry_time = time.time()
            self.last_command_type = None  # 重置命令类型，允许新状态发送命令
            
            # 添加详细的状态转换调试信息
            transition_info = f'State transition: {self.previous_state.value} -> {new_state.value}'
            if new_state == FSMState.TARGET_TRACKING:
                transition_info += f' (Ready to track targets)'
            elif new_state == FSMState.SEARCHING:
                transition_info += f' (Starting rotation search)'
            elif new_state == FSMState.VICTORY:
                transition_info += f' (Mission accomplished!)'
            elif new_state == FSMState.ERROR:
                transition_info += f' (Error detected, stopping)'
            
            self.get_logger().info(transition_info)
            
            # 发布状态变化
            state_msg = String()
            state_msg.data = new_state.value
            self.state_pub.publish(state_msg)
    
    def fsm_update(self):
        """FSM主更新循环"""
        # 检查紧急停止
        if self.emergency_stop_requested:
            if self.current_state != FSMState.EMERGENCY_STOP:
                self.transition_to_state(FSMState.EMERGENCY_STOP)
            return
        
        # 根据当前状态执行相应逻辑
        if self.current_state == FSMState.IDLE:
            self.handle_idle_state()
        
        elif self.current_state == FSMState.NAVIGATING:
            self.handle_navigating_state()
        
        elif self.current_state == FSMState.TARGET_TRACKING:
            self.handle_target_tracking_state()
        
        elif self.current_state == FSMState.SEARCHING:
            self.handle_searching_state()
        
        elif self.current_state == FSMState.VICTORY:
            self.handle_victory_state()
        
        elif self.current_state == FSMState.ERROR:
            self.handle_error_state()
        
        elif self.current_state == FSMState.EMERGENCY_STOP:
            self.handle_emergency_stop_state()
    
    def handle_idle_state(self):
        """处理空闲状态"""
        # 在空闲状态下不执行任何动作
        pass
    
    def handle_navigating_state(self):
        """处理导航状态"""
        if self.current_pose is None or self.target_pose is None:
            return
        
        # 检查导航超时
        elapsed_time = time.time() - self.state_entry_time
        if elapsed_time > self.navigation_timeout:
            self.get_logger().error('Navigation timeout')
            self.transition_to_state(FSMState.ERROR)
            return
        
        # 计算到目标的距离
        distance = math.sqrt(
            (self.current_pose.position.x - self.target_pose.position.x) ** 2 +
            (self.current_pose.position.y - self.target_pose.position.y) ** 2
        )
        
        # 减少导航过程中的调试信息频率
        self.get_logger().info(f'Navigating to target: distance={distance:.2f}m, elapsed={elapsed_time:.1f}s', throttle_duration_sec=5.0)
        
        # 检查是否到达目标
        if distance <= self.position_tolerance:
            self.get_logger().info(f'🎯 Reached target position! (distance: {distance:.3f}m, time: {elapsed_time:.1f}s)')
            # 到达目标后自动切换到目标跟踪模式
            self.transition_to_state(FSMState.TARGET_TRACKING)
            return
        
        # 发布导航命令
        self.publish_navigation_command()
    
    def handle_target_tracking_state(self):
        """处理目标跟踪状态"""
        current_time = time.time()
        
        # 检查目标像素更新超时 - 修复：即使never received pixel data也要启动搜索
        if self.enable_search_rotation:
            if self.last_target_pixel_time is not None:
                time_since_last = current_time - self.last_target_pixel_time
                if time_since_last > self.target_pixel_timeout:
                    self.get_logger().warn(f'🔍 Target lost! No pixel data for {time_since_last:.1f}s (timeout: {self.target_pixel_timeout}s), starting search rotation')
                    # 停止位置控制器，让搜索命令生效
                    self.stop_position_controller()
                    self.transition_to_state(FSMState.SEARCHING)
                    return
            else:
                # 如果从未收到过目标像素数据，检查是否在跟踪状态停留太久
                time_in_tracking = current_time - self.state_entry_time
                if time_in_tracking > self.target_pixel_timeout:
                    self.get_logger().warn(f'🔍 No pixel data received for {time_in_tracking:.1f}s since entering tracking, starting search rotation')
                    # 停止位置控制器，让搜索命令生效
                    self.stop_position_controller()
                    self.transition_to_state(FSMState.SEARCHING)
                    return
        
        # 添加目标检测状态调试信息
        if self.last_target_pixel_time is not None:
            time_since_last = current_time - self.last_target_pixel_time
            self.get_logger().info(f'👁️ Target tracking: last pixel update {time_since_last:.1f}s ago (timeout: {self.target_pixel_timeout}s)', throttle_duration_sec=5.0)
        else:
            self.get_logger().warn('👁️ Target tracking: no pixel data received yet', throttle_duration_sec=5.0)
        
        if not self.target_detected:
            # 如果长时间没有检测到目标，但搜索功能启用时不要进入错误状态
            elapsed_time = time.time() - self.state_entry_time
            self.get_logger().info(f'⚠️ No target detected in tracking mode for {elapsed_time:.1f}s', throttle_duration_sec=5.0)
            if elapsed_time > 30.0 and not self.enable_search_rotation:  # 只有搜索功能关闭时才超时进入错误
                self.get_logger().warn('Target tracking timeout - no target detected and search disabled')
                self.transition_to_state(FSMState.ERROR)
                return
            # 如果启用了搜索功能，让上面的搜索逻辑处理，不要在这里return
            if not self.enable_search_rotation:
                return
        
        # 检查是否在有效跟踪区域 - 修复：不在有效区域也要继续搜索目标
        if not self.in_valid_tracking_zone:
            current_x = self.current_pose.position.x if self.current_pose else 0.0
            self.get_logger().info(f'🚫 Not in valid tracking zone (x={current_x:.2f} < {self.valid_tracking_x}), but continuing to track target', throttle_duration_sec=5.0)
            # 不在有效区域时停止累计时间，但继续跟踪目标
            self.is_currently_hitting = False
            self.current_hit_session_start = None
            # 不要return，继续执行目标跟踪逻辑
        
        # 计算当前连续击中时间
        current_session_time = 0.0
        if self.is_currently_hitting and self.current_hit_session_start is not None:
            current_session_time = time.time() - self.current_hit_session_start
        
        # 显示进度 - 只显示当前连续击中时间
        if self.is_currently_hitting:
            current_x = self.current_pose.position.x if self.current_pose else 0.0
            remaining_time = self.victory_time_threshold - current_session_time
            self.get_logger().info(f'🎯 Hitting target: {current_session_time:.1f}s/{self.victory_time_threshold}s, remaining={remaining_time:.1f}s (x={current_x:.2f}m)', throttle_duration_sec=1.0)
        else:
            # 当前没有在打击时显示状态
            current_x = self.current_pose.position.x if self.current_pose else 0.0
            self.get_logger().info(f'🎯 Ready to hit: waiting for target in center (x={current_x:.2f}m)', throttle_duration_sec=3.0)
        
        # 检查胜利条件 - 修改：只要当前连续击中时间达到阈值就胜利
        if current_session_time >= self.victory_time_threshold:
            self.get_logger().info(f'🏆 Continuously hit target for {current_session_time:.1f}s in valid zone - Victory!')
            
            # 发布胜利消息
            victory_msg = Bool()
            victory_msg.data = True
            self.victory_pub.publish(victory_msg)
            
            # 保存胜利时刻的视频
            self.save_video_buffer()
            
            self.transition_to_state(FSMState.VICTORY)
            return
        
        # 在跟踪状态下保持当前位置，不再发布导航命令
        # 这样可以防止机器人回到(0,0)
        self.publish_hold_position_command()
        
        # 重置目标检测标志
        self.target_detected = False
    
    def handle_searching_state(self):
        """处理搜索状态 - 旋转搜索目标"""
        current_time = time.time()
        elapsed_time = current_time - self.state_entry_time
        
        # 检查搜索超时
        if elapsed_time > self.max_search_duration:
            self.get_logger().warn(f'Search timeout after {elapsed_time:.1f}s, returning to error state')
            self.transition_to_state(FSMState.ERROR)
            return
        
        # 检查是否收到新的目标像素数据
        if (self.last_target_pixel_time is not None and 
            current_time - self.last_target_pixel_time <= self.target_pixel_timeout):
            self.get_logger().info('🎯 Target found during search, returning to tracking')
            self.transition_to_state(FSMState.TARGET_TRACKING)
            return
        
        # 发布旋转命令 - 直接到cmd_vel，绕过位置控制器
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.search_angular_velocity
        self.cmd_vel_pub.publish(cmd)
        
        # 显示搜索进度
        self.get_logger().info(f'🔍 Searching for target: {elapsed_time:.1f}s/{self.max_search_duration}s (ω={self.search_angular_velocity:.1f}rad/s)', throttle_duration_sec=2.0)
    
    def handle_victory_state(self):
        """处理胜利状态"""
        # 在胜利状态下保持静止
        pass
    
    def handle_error_state(self):
        """处理错误状态"""
        # 错误状态下停止所有运动
        self.publish_stop_command()
    
    def handle_emergency_stop_state(self):
        """处理紧急停止状态"""
        # 发布停止命令
        self.publish_stop_command()
        
        # 检查是否可以解除紧急停止
        if not self.emergency_stop_requested:
            self.transition_to_state(FSMState.IDLE)
            self.get_logger().info('Emergency stop cleared')
    
    def publish_navigation_command(self):
        """发布导航命令"""
        if self.target_pose is None:
            return
        
        # 只在命令类型变化或目标位置变化时发送
        current_target = (self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z)
        if self.last_command_type != f'navigate_{current_target}':
            # 创建TwistStamped消息
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'odom'
            
            # 设置目标位置和速度
            cmd.twist.linear.x = self.target_pose.position.x
            cmd.twist.linear.y = self.target_pose.position.y
            cmd.twist.linear.z = self.target_pose.position.z
            cmd.twist.angular.z = 0.0  # 默认角度
            
            self.target_pose_pub.publish(cmd)
            self.last_command_type = f'navigate_{current_target}'
    
    def publish_search_rotation_command(self):
        """发布搜索旋转命令"""
        # 只在进入搜索状态时发送一次，然后持续发送（因为搜索需要持续旋转）
        cmd = Twist()  # 修复：使用 Twist 而不是 TwistStamped
        
        # 设置角速度进行旋转搜索
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.search_angular_velocity
        
        self.cmd_vel_pub.publish(cmd)
        self.last_command_type = 'search'
    
    def publish_hold_position_command(self):
        """发布保持当前位置的命令（停止运动）"""
        # 只在命令类型变化时发送，避免重复发送
        if self.last_command_type != 'hold':
            # 修复：发送零速度命令而不是坐标，让机器人停在原地
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'odom'
            
            # 设置所有速度为0，停在当前位置
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = 0.0
            cmd.twist.angular.x = 0.0
            cmd.twist.angular.y = 0.0
            cmd.twist.angular.z = 0.0
            
            self.target_pose_pub.publish(cmd)
            self.last_command_type = 'hold'
    
    def stop_position_controller(self):
        """停止位置控制器，清除其目标"""
        # 发送一个特殊的停止信号到位置控制器
        # 使用一个特殊的坐标组合来表示"停止所有控制"
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'stop_control'  # 使用特殊frame_id标识停止命令
        
        # 设置特殊值表示停止控制
        cmd.twist.linear.x = -999.0  # 使用特殊值表示停止位置控制
        cmd.twist.linear.y = -999.0
        cmd.twist.linear.z = -999.0
        cmd.twist.angular.x = -999.0
        cmd.twist.angular.y = -999.0
        cmd.twist.angular.z = -999.0
        
        self.target_pose_pub.publish(cmd)
        self.get_logger().info('Sent stop signal to position controller')
    
    def publish_stop_command(self):
        """发布停止命令"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'odom'
        
        # 所有速度设为0
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        
        self.target_pose_pub.publish(cmd)
    
    def monitor_topics(self):
        """监控关键话题的订阅状态"""
        current_time = time.time()
        
        # 检查里程计数据
        if self.current_pose is None:
            self.get_logger().warn('⚠️ No odometry data received from /chassis/odom')
        else:
            self.get_logger().info(f'📍 Robot position: x={self.current_pose.position.x:.2f}m, y={self.current_pose.position.y:.2f}m', throttle_duration_sec=10.0)
        
        # 检查目标像素数据
        if self.last_target_pixel_time is None:
            self.get_logger().warn('⚠️ No target pixel data received from /target_position_pixel')
        else:
            time_since_last = current_time - self.last_target_pixel_time
            if time_since_last > 5.0:  # 5秒没有更新
                self.get_logger().warn(f'⚠️ Target pixel data is stale: {time_since_last:.1f}s old')
        
        # 检查当前状态信息
        elapsed_in_state = current_time - self.state_entry_time
        self.get_logger().info(f'🔄 Current state: {self.current_state.value} (for {elapsed_in_state:.1f}s)', throttle_duration_sec=10.0)
        
        # 检查有效跟踪区域状态
        if self.current_pose:
            zone_status = "IN VALID ZONE" if self.in_valid_tracking_zone else "NOT IN VALID ZONE"
            self.get_logger().info(f'🎯 Tracking zone: {zone_status} (x={self.current_pose.position.x:.2f} vs threshold>={self.valid_tracking_x})', throttle_duration_sec=10.0)
    
    def publish_status(self):
        """发布状态信息"""
        status_info = {
            'state': self.current_state.value,
            'time_in_state': time.time() - self.state_entry_time,
            'target_detected': self.target_detected,
            'navigation_active': self.navigation_active,
            'victory_accumulated_time': self.victory_accumulated_time,
            'is_currently_hitting': self.is_currently_hitting,
            'current_session_time': time.time() - self.current_hit_session_start if self.current_hit_session_start else 0.0,
            'in_valid_tracking_zone': self.in_valid_tracking_zone,
            'current_x': self.current_pose.position.x if self.current_pose else 0.0,
            'valid_tracking_threshold': self.valid_tracking_x,
            'video_buffer_size': len(self.video_buffer) if self.enable_video_recording and self.video_buffer else 0,
            'video_recording_enabled': self.enable_video_recording,
            'is_searching': self.current_state == FSMState.SEARCHING,
            'time_since_last_target': time.time() - self.last_target_pixel_time if self.last_target_pixel_time else float('inf'),
            'search_enabled': self.enable_search_rotation,
            'target_pixel_timeout': self.target_pixel_timeout,
            'search_angular_velocity': self.search_angular_velocity
        }
        
        if self.current_pose and self.target_pose:
            distance = math.sqrt(
                (self.current_pose.position.x - self.target_pose.position.x) ** 2 +
                (self.current_pose.position.y - self.target_pose.position.y) ** 2
            )
            status_info['distance_to_target'] = distance
        
        if self.target_pixel:
            distance_to_center = math.sqrt(
                (self.target_pixel.x - self.image_center_x) ** 2 + 
                (self.target_pixel.y - self.image_center_y) ** 2
            )
            status_info['pixel_distance_to_center'] = distance_to_center
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotFSM()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
