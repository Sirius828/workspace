#!/usr/bin/env python3
"""
Pixel‑to‑Gimbal Mapper  ‑ ROS 2 (Python)
=======================================

**功能**
--------
* 通过 4 个已标定的角点，将任意图像像素坐标 → 云台 (yaw,pitch) 角度组合
* 支持两种映射模型（参数 `model`）：
  1. **homography** ‑ 单应变换（默认，精度更高）
  2. **bilinear**   ‑ 先归一化到单位方形再双线性插值（依赖更少，速度极快）
* 运行期订阅：
  * **sensor_msgs/msg/CameraInfo** (仅记录相机参数，可选)
  * **geometry_msgs/msg/Point**    (像素坐标，单位: 像素)
* 实时发布：
  * **std_msgs/msg/Float64MultiArray** → [yaw, pitch]，单位: **弧度**

*作者*    : ChatGPT demo  
*日期*    : 2025‑07‑13
"""

from __future__ import annotations

import math
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import JointState

try:
    import cv2  # 用于 homography 计算
    _HAS_CV2 = True
except ImportError:  # bilinear 模型不需要
    _HAS_CV2 = False


class Pixel2Gimbal(Node):
    """ROS2 节点：给定像素坐标，输出云台角度。"""

    def __init__(self) -> None:
        super().__init__('pixel2gimbal')

        # -------- 参数声明 --------
        self.declare_parameter(
            'src_pixels',
            [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0],
            descriptor=ParameterDescriptor(
                description='像素 4 角点 [[u,v], ...]，顺时针或逆时针'))
        self.declare_parameter(
            'dst_angles',
            [0.0, 0.0, 0.1, 0.0, 0.1, 0.1, 0.0, 0.1],
            descriptor=ParameterDescriptor(
                description='对应云台角度 [[yaw,pitch], ...] (弧度)'))
        self.declare_parameter('pixel_topic', '/roi_center')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('gimbal_cmd_topic', '/gimbal_cmd')
        self.declare_parameter('model', 'homography',
                               descriptor=ParameterDescriptor(
                                   description='homography | bilinear'))
        self.declare_parameter('lowpass_alpha', 0.2,
                               descriptor=ParameterDescriptor(
                                   description='一阶低通滤波系数 0‑1'))

        # 读参数
        src_pixels: List[List[float]] = self.get_parameter('src_pixels').get_parameter_value().double_array_value  # type: ignore
        dst_angles: List[List[float]] = self.get_parameter('dst_angles').get_parameter_value().double_array_value  # type: ignore
        # rclpy 把 double_array 展平成 1‑D；转成 4×2
        src_pixels = np.array(src_pixels).reshape((-1, 2))
        dst_angles = np.array(dst_angles).reshape((-1, 2))

        if src_pixels.shape != (4, 2) or dst_angles.shape != (4, 2):
            raise ValueError('src_pixels 和 dst_angles 必须是 4×2 数组')

        self.model = self.get_parameter('model').get_parameter_value().string_value
        if self.model == 'homography':
            if not _HAS_CV2:
                self.get_logger().warn('cv2 未安装，自动切换到 bilinear')
                self.model = 'bilinear'
        if self.model not in ('homography', 'bilinear'):
            self.get_logger().error(f'不支持的 model: {self.model}')
            raise RuntimeError('invalid model')

        # -------- 预计算映射 --------
        if self.model == 'homography':
            self._build_homography(src_pixels, dst_angles)
        else:
            self._build_bilinear(src_pixels, dst_angles)

        # -------- 订阅 / 发布 --------
        pixel_topic = self.get_parameter('pixel_topic').get_parameter_value().string_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        gimbal_topic = self.get_parameter('gimbal_cmd_topic').get_parameter_value().string_value

        self.cam_info_sub = self.create_subscription(CameraInfo, cam_info_topic,
                                                     self._on_cam_info, 10)
        self.pixel_sub = self.create_subscription(Point, pixel_topic, self._on_pixel, 10)
        self.pub = self.create_publisher(Float64MultiArray, gimbal_topic, 10)
        # JointState publisher for simulation
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        # Publisher for raw (pre-processed) joint data
        self.raw_joint_pub = self.create_publisher(
            JointState, '/gimbal_joint_states', 10)
        # Interpolation state
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.interp_steps = 100  # 100 Hz over 10 seconds
        self.interp_count = self.interp_steps
        self.yaw_step = 0.0
        self.pitch_step = 0.0
        # Timer for interpolation publishing at 2000 Hz
        self.create_timer(1.0 / 150.0, self._on_timer)

        # Trajectory from vision services
        self.traj_type = None
        self.current_traj = []       # list of (u,v) tuples
        self.traj_angles = []        # list of (yaw_rad, pitch_rad)
        self.traj_idx = 0
        self._traj_buffer = []       # temp buffer for multi-point sequences

        # 低通滤波缓存
        self.alpha = self.get_parameter('lowpass_alpha').get_parameter_value().double_value
        self.yaw_lp = None  # type: float | None
        self.pitch_lp = None  # type: float | None

        # 打印参数
        self.get_logger().info(f"src_pixels: {src_pixels.tolist()}")
        self.get_logger().info(f"dst_angles: {dst_angles.tolist()}")
        self.get_logger().info(f"model: {self.model}")
        self.get_logger().info(f"pixel_topic: {pixel_topic}, camera_info_topic: {cam_info_topic}, gimbal_cmd_topic: {gimbal_topic}")
        self.get_logger().info(f"lowpass_alpha: {self.alpha}")

        self.get_logger().info(f'Pixel2Gimbal 节点已启动，映射模型: {self.model}')

        # Subscribe to vision service outputs
        self.create_subscription(Point,   '/target_position',  self._on_center,  10)
        self.create_subscription(Point32, '/corner_points',    self._on_square,  10)
        self.create_subscription(Point32, '/midline_points',   self._on_midline, 10)

    # ==================== 构造映射 ====================
    def _build_homography(self, src: np.ndarray, dst: np.ndarray):
        # 把弧度缩放到可辨识范围，避免 float 精度丢失
        scale = 1.0
        if np.max(np.abs(dst)) < 0.02:  # 约 1.1°
            scale = 100.0
            dst *= scale
        self.H, _ = cv2.findHomography(src, dst)  # 3×3
        self.scale = scale
        self.get_logger().debug(f'Homography matrix:\n{self.H}')

    def _build_bilinear(self, src: np.ndarray, dst: np.ndarray):
        # _T_ : 像素 → 单位方形 (s,t)
        if _HAS_CV2:
            self.T = cv2.getPerspectiveTransform(src.astype(np.float32),
                                                 np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]], dtype=np.float32))
        else:
            # minimal 4‑point projective transform (numpy)
            A = []
            for (u, v), (s, t) in zip(src, [[0, 0], [1, 0], [1, 1], [0, 1]]):
                A.extend([
                    [u, v, 1, 0, 0, 0, -s * u, -s * v],
                    [0, 0, 0, u, v, 1, -t * u, -t * v]
                ])
            A = np.array(A)
            b = np.array([0, 0, 1, 0, 0, 1, 1, 0])
            h = np.linalg.solve(A, b)
            self.T = np.append(h, 1).reshape(3, 3)
        self.dst_angles = dst  # 4×2

    # ==================== 回调 ====================
    def _on_cam_info(self, msg: CameraInfo):
        # 目前不必使用，只打印一次以便调试
        if not hasattr(self, '_cam_logged'):
            self.get_logger().info(f'接收到 CameraInfo: k_fx={msg.k[0]:.1f}, k_fy={msg.k[4]:.1f}')
            setattr(self, '_cam_logged', True)

    def _pix2angle(self, u: float, v: float) -> tuple[float, float]:
        if self.model == 'homography':
            vec = self.H @ np.array([u, v, 1.0])
            yaw, pitch = vec[0] / vec[2], vec[1] / vec[2]
            return yaw / self.scale, pitch / self.scale
        else:  # bilinear
            s, t, w = self.T @ np.array([u, v, 1.0])
            s, t = s / w, t / w
            # clamp s,t to [0,1]
            s = float(np.clip(s, 0.0, 1.0))
            t = float(np.clip(t, 0.0, 1.0))
            yaw = (1 - s) * (1 - t) * self.dst_angles[0, 0] + s * (1 - t) * self.dst_angles[1, 0] + \
                  s * t * self.dst_angles[2, 0] + (1 - s) * t * self.dst_angles[3, 0]
            pitch = (1 - s) * (1 - t) * self.dst_angles[0, 1] + s * (1 - t) * self.dst_angles[1, 1] + \
                    s * t * self.dst_angles[2, 1] + (1 - s) * t * self.dst_angles[3, 1]
            return yaw, pitch

    def _on_pixel(self, msg: Point):
        u, v = msg.x, msg.y
        yaw, pitch = self._pix2angle(u, v)

        # Publish raw joint angles (radians) before any processing
        js_raw = JointState()
        js_raw.header.stamp = self.get_clock().now().to_msg()
        js_raw.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        js_raw.position = [yaw, pitch]
        self.raw_joint_pub.publish(js_raw)

        # 一阶低通
        if self.yaw_lp is None:
            self.yaw_lp = yaw
            self.pitch_lp = pitch
        else:
            self.yaw_lp = self.alpha * yaw + (1 - self.alpha) * self.yaw_lp
            self.pitch_lp = self.alpha * pitch + (1 - self.alpha) * self.pitch_lp

        # Raw angles in radians (no inversion here)
        yaw_rad   = self.yaw_lp
        pitch_rad = self.pitch_lp
        
        # Setup interpolation from current to target in radians
        self.target_yaw = yaw_rad
        self.target_pitch = pitch_rad
        self.interp_count = 0
        self.yaw_step = (self.target_yaw - self.current_yaw) / self.interp_steps
        self.pitch_step = (self.target_pitch - self.current_pitch) / self.interp_steps

        # Publish raw command in degrees if desired
        yaw_deg   = yaw_rad   * 180.0 / math.pi
        pitch_deg = pitch_rad * 180.0 / math.pi
        out = Float64MultiArray()
        out.data = [yaw_deg, pitch_deg]
        self.pub.publish(out)


    def _on_timer(self):
        # Trajectory handling
        if self.traj_angles and self.traj_type in ('center', 'square', 'midline'):
            # Total segments equals number of trajectory points minus 1
            total_segs = len(self.traj_angles) - 1
            if self.interp_count < self.interp_steps:
                # Continue interpolation of current segment
                self.current_yaw   += self.yaw_step
                self.current_pitch += self.pitch_step
                self.interp_count  += 1
            else:
                # Finished current segment
                if self.traj_idx < total_segs:
                    # Advance to next segment
                    prev_idx = self.traj_idx
                    self.traj_idx += 1
                    yaw0, pitch0 = self.traj_angles[prev_idx]
                    yaw1, pitch1 = self.traj_angles[self.traj_idx]
                    self.current_yaw = yaw0
                    self.current_pitch = pitch0
                    self.interp_count = 0
                    self.yaw_step   = (yaw1 - yaw0) / self.interp_steps
                    self.pitch_step = (pitch1 - pitch0) / self.interp_steps
                else:
                    # Completed full loop: clear trajectory
                    self.traj_angles = []
                    self.traj_type = None
                    return
            # Publish joint angles with inverted sign
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
            # Invert once here for simulation
            js.position = [-self.current_yaw, -self.current_pitch]
            self.joint_pub.publish(js)
            # Publish synchronous raw (non‑inverted) joint data
            js_raw = JointState()
            js_raw.header = js.header          # same timestamp
            js_raw.name = js.name
            js_raw.position = [self.current_yaw, self.current_pitch]
            self.raw_joint_pub.publish(js_raw)
            return
        # Fallback: no trajectory, do nothing
        # (Legacy single-point interpolation block removed)
        # if self.interp_count < self.interp_steps:
        #     self.current_yaw += self.yaw_step
        #     self.current_pitch += self.pitch_step
        #     self.interp_count += 1
        # # Publish JointState
        # js = JointState()
        # js.header.stamp = self.get_clock().now().to_msg()
        # js.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        # js.position = [self.current_yaw, self.current_pitch]
        # self.joint_pub.publish(js)


    # ==================== Trajectory callbacks ====================
    def _on_center(self, msg: Point):
        # Single-point trajectory with interpolation
        self.get_logger().info('Received center point trajectory')
        # Compute target angle
        yaw_tgt, pitch_tgt = self._pix2angle(msg.x, msg.y)
        # Prepare a two-point trajectory: current → target
        self.traj_type = 'center'
        # Use current_yaw/current_pitch as start
        start = (self.current_yaw, self.current_pitch)
        end = (yaw_tgt, pitch_tgt)
        self.traj_angles = [start, end]
        self.get_logger().info(f"Center trajectory angles (rad): start={start}, end={end}")
        # Reset interpolation index
        self.traj_idx = 1
        self.interp_count = 0
        # Compute steps
        self.yaw_step = (end[0] - start[0]) / self.interp_steps
        self.pitch_step = (end[1] - start[1]) / self.interp_steps

    def _on_square(self, msg: Point32):
        # Collect exactly 5 points (TL,TR,BR,BL,TL)
        if self.traj_type != 'square_init':
            self.get_logger().info('Initializing square trajectory')
            self.traj_type = 'square_init'
            self._traj_buffer = []
        self._traj_buffer.append((msg.x, msg.y))

        if len(self._traj_buffer) == 5:
            self.get_logger().info('Square trajectory ready')
            self.traj_type = 'square'
            self.current_traj = list(self._traj_buffer)      # keep 5 points to close loop
            # Convert to yaw/pitch and invert sign to compensate final publish inversion
            self.traj_angles = [self._pix2angle(u, v) for u, v in self.current_traj]            
            self.get_logger().info(f"Square_loop pixel points: {self.current_traj}")
            self.get_logger().info(f"Square_loop adjusted angles (rad): {self.traj_angles}")
            self.traj_idx = 0
            self._setup_interpolation()

    def _on_midline(self, msg: Point32):
        # Midline loop: collect points until duplicate of first
        if self.traj_type != 'midline_init':
            self.get_logger().info('Initializing midline trajectory')
            self.traj_type = 'midline_init'
            self._traj_buffer = []
        self._traj_buffer.append((msg.x, msg.y))
        # Check duplicate of first to finish
        if len(self._traj_buffer) > 1 and (msg.x, msg.y) == self._traj_buffer[0]:
            self.get_logger().info('Midline trajectory ready')
            self.traj_type = 'midline'
            self.current_traj = list(self._traj_buffer)
            self.traj_angles = [self._pix2angle(u, v) for u, v in self.current_traj]            
            self.traj_idx = 0
            self._setup_interpolation()

    def _setup_interpolation(self):
        # Prepare step values for the first segment
        if len(self.traj_angles) >= 2:
            yaw0, pitch0 = self.traj_angles[0]
            yaw1, pitch1 = self.traj_angles[1]
            self.current_yaw = yaw0
            self.current_pitch = pitch0
            self.traj_idx = 1
            self.interp_count = 0
            self.yaw_step = (yaw1 - yaw0) / self.interp_steps
            self.pitch_step = (pitch1 - pitch0) / self.interp_steps


# ==================== main ====================

def main(args: List[str] | None = None):
    rclpy.init(args=args)
    try:
        node = Pixel2Gimbal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
