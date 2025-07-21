#!/usr/bin/env python3
"""
Vision Services Node for ROS2
提供四种视觉服务：RESET, SQUARE_LOOP, A4_LOOP, A4_LOOP_ROT
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Point32
import cv2
from cv_bridge import CvBridge
import yaml
import os
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory

# 导入服务接口
from vision_services_msgs.srv import Reset, SquareLoop, A4Loop, A4LoopRot
from vision_services.image_processor import ImageProcessor


class VisionServicesNode(Node):
    """视觉服务节点"""
    
    def __init__(self):
        super().__init__('vision_services_node')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = False
        
        # 图像稳定检测
        self.image_history = []
        self.stability_threshold = 1.0  # 1秒稳定时间
        self.stability_check_interval = 0.1  # 每100ms检查一次
        self.frame_diff_threshold = 5000  # 帧差阈值
        
        # 加载配置参数
        self._load_config()
        # 打印加载后的配置
        self.get_logger().info(f"Configuration loaded: {self.config}")
        
        # 初始化图像处理器
        self.image_processor = ImageProcessor(self.config)
        
        # 检查是否启用预览
        preview_enabled = self.get_parameter('preview_enabled').value
        if preview_enabled:
            self.get_logger().info('Preview interface enabled')
        else:
            self.get_logger().info('Preview interface disabled')
        
        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # 创建发布者
        self.target_pub = self.create_publisher(
            Point, 
            self.config['publishing']['topics']['target_position'], 
            10
        )
        
        self.corner_pub = self.create_publisher(
            Point32, 
            self.config['publishing']['topics']['corner_points'], 
            10
        )
        
        self.midline_pub = self.create_publisher(
            Point32, 
            self.config['publishing']['topics']['midline_points'], 
            10
        )
        
        # 创建服务
        self.reset_srv = self.create_service(
            Reset, 
            'vision_services/reset', 
            self.handle_reset
        )
        
        self.square_loop_srv = self.create_service(
            SquareLoop, 
            'vision_services/square_loop', 
            self.handle_square_loop
        )
        
        self.a4_loop_srv = self.create_service(
            A4Loop, 
            'vision_services/a4_loop', 
            self.handle_a4_loop
        )
        
        self.a4_loop_rot_srv = self.create_service(
            A4LoopRot, 
            'vision_services/a4_loop_rot', 
            self.handle_a4_loop_rot
        )
        
        self.get_logger().info('Vision Services Node initialized')
        self.get_logger().info(f'Subscribed to: /camera/color/image_raw')
        self.get_logger().info(f'Services available:')
        self.get_logger().info('  - vision_services/reset')
        self.get_logger().info('  - vision_services/square_loop')
        self.get_logger().info('  - vision_services/a4_loop')
        self.get_logger().info('  - vision_services/a4_loop_rot')
    
    def _load_config(self):
        """从ROS2参数加载配置"""
        # 声明所有参数
        self.declare_parameter('gaussian_blur_kernel_size_x', 5)
        self.declare_parameter('gaussian_blur_kernel_size_y', 5)
        self.declare_parameter('gaussian_blur_sigma_x', 0)
        self.declare_parameter('gaussian_blur_sigma_y', 0)
        self.declare_parameter('morphology_kernel_size_x', 3)
        self.declare_parameter('morphology_kernel_size_y', 3)
        self.declare_parameter('morphology_iterations', 2)
        self.declare_parameter('canny_low_threshold', 50)
        self.declare_parameter('canny_high_threshold', 150)
        self.declare_parameter('canny_aperture_size', 3)
        
        # 中心裁切参数
        self.declare_parameter('center_crop_enabled', False)
        self.declare_parameter('center_crop_width', 800)
        self.declare_parameter('center_crop_height', 600)
        self.declare_parameter('center_crop_zoom_factor', 1.5)
        
        # 正方形检测参数
        self.declare_parameter('square_contour_min_area', 1000)
        self.declare_parameter('square_contour_max_area', 50000)
        self.declare_parameter('square_contour_approx_epsilon_factor', 0.02)
        self.declare_parameter('square_validation_min_side_ratio', 0.8)
        self.declare_parameter('square_validation_max_side_ratio', 1.2)
        self.declare_parameter('square_validation_min_angle', 80)
        self.declare_parameter('square_validation_max_angle', 100)
        
        # A4检测参数
        self.declare_parameter('a4_contour_min_area', 5000)
        self.declare_parameter('a4_contour_max_area', 100000)
        self.declare_parameter('a4_contour_approx_epsilon_factor', 0.01)
        self.declare_parameter('tape_hsv_lower_h', 0)
        self.declare_parameter('tape_hsv_lower_s', 0)
        self.declare_parameter('tape_hsv_lower_v', 0)
        self.declare_parameter('tape_hsv_upper_h', 180)
        self.declare_parameter('tape_hsv_upper_s', 255)
        self.declare_parameter('tape_hsv_upper_v', 50)
        self.declare_parameter('tape_morphology_kernel_size_x', 5)
        self.declare_parameter('tape_morphology_kernel_size_y', 5)
        self.declare_parameter('tape_morphology_iterations', 3)
        self.declare_parameter('rectangle_validation_min_aspect_ratio', 1.3)
        self.declare_parameter('rectangle_validation_max_aspect_ratio', 1.5)
        
        # 发布参数
        self.declare_parameter('target_position_topic', '/target_position')
        self.declare_parameter('corner_points_topic', '/corner_points')
        self.declare_parameter('midline_points_topic', '/midline_points')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        
        # 调试和预览参数
        self.declare_parameter('debug_show_images', False)
        self.declare_parameter('preview_enabled', True)
        # Predefined plate corners in pixel coords: flat [x1,y1,x2,y2,x3,y3,x4,y4]
        self.declare_parameter('predef_corners', [80.0, 120.0, 560.0, 120.0, 560.0, 420.0, 80.0, 420.0])
        self.declare_parameter('image_preprocessing.roi_mask.enabled', True)
        self.declare_parameter('image_preprocessing.roi_mask.polygon', [80.0, 120.0, 560.0, 120.0, 560.0, 420.0, 80.0, 420.0])
        
        # 构建配置字典
        self.config = {
            'image_preprocessing': {
                'gaussian_blur': {
                    'kernel_size_x': self.get_parameter('gaussian_blur_kernel_size_x').value,
                    'kernel_size_y': self.get_parameter('gaussian_blur_kernel_size_y').value,
                    'sigma_x': self.get_parameter('gaussian_blur_sigma_x').value,
                    'sigma_y': self.get_parameter('gaussian_blur_sigma_y').value
                },
                'morphology': {
                    'kernel_size_x': self.get_parameter('morphology_kernel_size_x').value,
                    'kernel_size_y': self.get_parameter('morphology_kernel_size_y').value,
                    'iterations': self.get_parameter('morphology_iterations').value
                },
                'canny': {
                    'low_threshold': self.get_parameter('canny_low_threshold').value,
                    'high_threshold': self.get_parameter('canny_high_threshold').value,
                    'aperture_size': self.get_parameter('canny_aperture_size').value
                },
                'center_crop': {
                    'enabled': self.get_parameter('center_crop_enabled').value,
                    'width': self.get_parameter('center_crop_width').value,
                    'height': self.get_parameter('center_crop_height').value,
                    'zoom_factor': self.get_parameter('center_crop_zoom_factor').value
                },
                'roi_mask': {
                    'enabled': self.get_parameter('image_preprocessing.roi_mask.enabled').value,
                    'polygon': self.get_parameter('image_preprocessing.roi_mask.polygon').value,
                },
            },
            'square_detection': {
                'contour': {
                    'min_area': self.get_parameter('square_contour_min_area').value,
                    'max_area': self.get_parameter('square_contour_max_area').value,
                    'approx_epsilon_factor': self.get_parameter('square_contour_approx_epsilon_factor').value
                },
                'square_validation': {
                    'min_side_ratio': self.get_parameter('square_validation_min_side_ratio').value,
                    'max_side_ratio': self.get_parameter('square_validation_max_side_ratio').value,
                    'min_angle': self.get_parameter('square_validation_min_angle').value,
                    'max_angle': self.get_parameter('square_validation_max_angle').value
                }
            },
            'a4_detection': {
                'contour': {
                    'min_area': self.get_parameter('a4_contour_min_area').value,
                    'max_area': self.get_parameter('a4_contour_max_area').value,
                    'approx_epsilon_factor': self.get_parameter('a4_contour_approx_epsilon_factor').value
                },
                'tape_detection': {
                    'hsv_lower_h': self.get_parameter('tape_hsv_lower_h').value,
                    'hsv_lower_s': self.get_parameter('tape_hsv_lower_s').value,
                    'hsv_lower_v': self.get_parameter('tape_hsv_lower_v').value,
                    'hsv_upper_h': self.get_parameter('tape_hsv_upper_h').value,
                    'hsv_upper_s': self.get_parameter('tape_hsv_upper_s').value,
                    'hsv_upper_v': self.get_parameter('tape_hsv_upper_v').value,
                    'morphology_kernel_size_x': self.get_parameter('tape_morphology_kernel_size_x').value,
                    'morphology_kernel_size_y': self.get_parameter('tape_morphology_kernel_size_y').value,
                    'morphology_iterations': self.get_parameter('tape_morphology_iterations').value
                },
                'rectangle_validation': {
                    'min_aspect_ratio': self.get_parameter('rectangle_validation_min_aspect_ratio').value,
                    'max_aspect_ratio': self.get_parameter('rectangle_validation_max_aspect_ratio').value
                }
            },
            'publishing': {
                'topics': {
                    'target_position': self.get_parameter('target_position_topic').value,
                    'corner_points': self.get_parameter('corner_points_topic').value,
                    'midline_points': self.get_parameter('midline_points_topic').value
                },
                'frame_id': self.get_parameter('frame_id').value
            },
            'debug': {
                'show_images': self.get_parameter('debug_show_images').value
            },
            'preview': {
                'enabled': self.get_parameter('preview_enabled').value,
                'window_size': {
                    'width': 800,
                    'height': 600
                },
                'window_position': {
                    'x': 100,
                    'y': 100
                },
                'display_options': {
                    'show_original': True,
                    'show_preprocessed': True,
                    'show_contours': True,
                    'show_results': True,
                    'show_info_panel': True
                },
                'colors': {
                    'square_contour': [0, 255, 0],
                    'rectangle_contour': [255, 0, 0],
                    'center_point': [0, 0, 255],
                    'corner_points': [255, 255, 0],
                    'midline': [128, 0, 128],
                    'text': [255, 255, 255]
                },
                'drawing': {
                    'contour_thickness': 2,
                    'point_radius': 5,
                    'text_font_scale': 0.6,
                    'text_thickness': 2
                },
                'update_rate': 30
        
            },
            'predef_corners': self.get_parameter('predef_corners').value
        }
        
        self.get_logger().info('Loaded configuration from ROS2 parameters')
    
    def _load_default_config(self):
        """加载默认配置"""
        self.config = {
            'image_preprocessing': {
                'gaussian_blur': {'kernel_size': [5, 5], 'sigma_x': 0, 'sigma_y': 0},
                'morphology': {'kernel_size': [3, 3], 'iterations': 2},
                'canny': {'low_threshold': 50, 'high_threshold': 150, 'aperture_size': 3}
            },
            'square_detection': {
                'contour': {'min_area': 1000, 'max_area': 50000, 'approx_epsilon_factor': 0.02},
                'square_validation': {'min_side_ratio': 0.8, 'max_side_ratio': 1.2, 'min_angle': 80, 'max_angle': 100}
            },
            'a4_detection': {
                'contour': {'min_area': 5000, 'max_area': 100000, 'approx_epsilon_factor': 0.01},
                'tape_detection': {'hsv_lower': [0, 0, 0], 'hsv_upper': [180, 255, 50], 'morphology_kernel_size': [5, 5], 'morphology_iterations': 3},
                'rectangle_validation': {'min_aspect_ratio': 1.3, 'max_aspect_ratio': 1.5, 'min_angle': 85, 'max_angle': 95}
            },
            'publishing': {
                'topics': {
                    'target_position': '/target_position',
                    'corner_points': '/corner_points',
                    'midline_points': '/midline_points'
                },
                'frame_id': 'camera_color_optical_frame'
            },
            'debug': {'show_images': False, 'log_level': 'INFO'}
        }
        self.get_logger().warn('Using default configuration')
    
    def image_callback(self, msg):
        """图像回调函数"""
        if not self.image_lock:
            try:
                current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # 添加时间戳
                current_time = time.time()
                
                # 更新图像历史（用于稳定性检测）
                self._update_image_history(current_image, current_time)
                
                # 更新最新图像
                self.latest_image = current_image
                
                # 如果启用了预览，实时更新预览窗口
                if self.config['preview']['enabled']:
                    self.image_processor.update_preview(current_image)
                    
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')
    
    def handle_reset(self, request, response):
        """
        处理RESET服务（使用预定义角点）
        """
        self.get_logger().info('Received RESET service request (using predefined corners)')
        # Compute center from predefined corners
        flat = self.config['predef_corners']
        corners = []
        for i in range(0, len(flat), 2):
            p = Point()
            p.x, p.y, p.z = flat[i], flat[i+1], 0.0
            corners.append(p)
        center = Point()
        center.x = sum(p.x for p in corners) / 4.0
        center.y = sum(p.y for p in corners) / 4.0
        center.z = 0.0
        self.target_pub.publish(center)
        response.success = True
        response.message = f"Predefined center published: ({center.x:.1f}, {center.y:.1f})"
        response.center_point = center
        return response
    
    def handle_square_loop(self, request, response):
        """
        处理SQUARE_LOOP服务（使用预定义角点）
        """
        self.get_logger().info('Received SQUARE_LOOP service request (using predefined corners)')
        flat = self.config['predef_corners']
        corners = []
        for i in range(0, len(flat), 2):
            p = Point32()
            p.x, p.y, p.z = flat[i], flat[i+1], 0.0
            corners.append(p)
        # Publish loop: 4 corners + first again
        for p in corners:
            self.corner_pub.publish(p)
        self.corner_pub.publish(corners[0])
        response.success = True
        response.message = "Predefined corners published (loop)"
        response.corner_points = corners + [corners[0]]
        return response
    
    def handle_a4_loop(self, request, response):
        """
        处理A4_LOOP服务
        识别图像中用黑色胶带拼接的矩形框，得到胶带矩形框内外边界的矩形框中线
        """
        self.get_logger().info('Received A4_LOOP service request')
        
        if self.latest_image is None:
            response.success = False
            response.message = "No image available"
            response.midline_points = []
            return response
        
        # 等待图像稳定
        self.get_logger().info('Waiting for image to stabilize...')
        if not self._wait_for_stable_image():
            response.success = False
            response.message = "Image did not stabilize within timeout"
            response.midline_points = []
            return response
        
        try:
            self.image_lock = True
            self.get_logger().info('Image stable, starting A4 rectangle detection...')
            # ROI mask application block
            img = self.latest_image.copy()
            roi_cfg = self.config['image_preprocessing'].get('roi_mask', {})
            if roi_cfg.get('enabled', False) and roi_cfg.get('polygon'):
                # build mask
                mask = np.zeros(img.shape[:2], dtype=np.uint8)
                arr = np.array(roi_cfg['polygon'], dtype=np.int32)
                pts = arr.reshape(-1, 2) if arr.ndim == 1 else arr
                cv2.fillPoly(mask, [pts], 255)
                # Fill area outside ROI with white to preserve only ROI for detection
                mask_inv = cv2.bitwise_not(mask)
                img[mask_inv == 255] = (255, 255, 255)
            midline_points, _ = self.image_processor.detect_a4_rectangle(img, rotated=False)
            self.image_lock = False
            
            if midline_points is not None:
                # 发布中线角点
                for i, point in enumerate(midline_points):
                    self.midline_pub.publish(point)
                    self.get_logger().info(f'Midline point {i+1}: ({point.x:.1f}, {point.y:.1f})')
                
                # 发布第一个点形成闭环
                self.midline_pub.publish(midline_points[0])
                self.get_logger().info(f'Midline point 1 (loop): ({midline_points[0].x:.1f}, {midline_points[0].y:.1f})')
                
                response.success = True
                response.message = f"A4 rectangle midline detected and published"
                response.midline_points = midline_points + [midline_points[0]]
                
            else:
                response.success = False
                response.message = "No A4 rectangle detected in image"
                response.midline_points = []
                
                self.get_logger().warn('No A4 rectangle detected in image')
                
        except Exception as e:
            self.image_lock = False
            response.success = False
            response.message = f"Error processing image: {str(e)}"
            response.midline_points = []
            
            self.get_logger().error(f'Error in A4_LOOP service: {e}')
        
        return response
    
    def handle_a4_loop_rot(self, request, response):
        """
        处理A4_LOOP_ROT服务
        识别图像中用黑色胶带拼接的旋转矩形框，得到胶带矩形框内外边界的矩形框中线
        """
        self.get_logger().info('Received A4_LOOP_ROT service request')
        
        if self.latest_image is None:
            response.success = False
            response.message = "No image available"
            response.midline_points = []
            response.rotation_angle = 0.0
            return response
        
        # 等待图像稳定
        self.get_logger().info('Waiting for image to stabilize...')
        if not self._wait_for_stable_image():
            response.success = False
            response.message = "Image did not stabilize within timeout"
            response.midline_points = []
            response.rotation_angle = 0.0
            return response
        
        try:
            self.image_lock = True
            self.get_logger().info('Image stable, starting rotated A4 rectangle detection...')
            img = self.latest_image.copy()
            roi_cfg = self.config['image_preprocessing'].get('roi_mask', {})
            if roi_cfg.get('enabled', False) and roi_cfg.get('polygon'):
                mask = np.zeros(img.shape[:2], dtype=np.uint8)
                arr = np.array(roi_cfg['polygon'], dtype=np.int32)
                pts = arr.reshape(-1, 2) if arr.ndim == 1 else arr
                cv2.fillPoly(mask, [pts], 255)
                mask_inv = cv2.bitwise_not(mask)
                img[mask_inv == 255] = (255, 255, 255)
            midline_points, angle = self.image_processor.detect_a4_rectangle(img, rotated=True)
            self.image_lock = False
            
            if midline_points is not None:
                # 发布中线角点
                for i, point in enumerate(midline_points):
                    self.midline_pub.publish(point)
                    self.get_logger().info(f'Rotated midline point {i+1}: ({point.x:.1f}, {point.y:.1f})')
                
                # 发布第一个点形成闭环
                self.midline_pub.publish(midline_points[0])
                self.get_logger().info(f'Rotated midline point 1 (loop): ({midline_points[0].x:.1f}, {midline_points[0].y:.1f})')
                
                response.success = True
                response.message = f"Rotated A4 rectangle midline detected and published (angle: {angle:.1f}°)"
                response.midline_points = midline_points + [midline_points[0]]
                response.rotation_angle = float(angle) if angle is not None else 0.0
                
                self.get_logger().info(f'Detected rotation angle: {angle:.1f}°')
                
            else:
                response.success = False
                response.message = "No rotated A4 rectangle detected in image"
                response.midline_points = []
                response.rotation_angle = 0.0
                
                self.get_logger().warn('No rotated A4 rectangle detected in image')
                
        except Exception as e:
            self.image_lock = False
            response.success = False
            response.message = f"Error processing image: {str(e)}"
            response.midline_points = []
            response.rotation_angle = 0.0
            
            self.get_logger().error(f'Error in A4_LOOP_ROT service: {e}')
        
        return response
    
    def _order_corners(self, corners):
        """
        对角点进行排序，确保按顺时针或逆时针顺序排列
        """
        # 计算中心点
        center_x = sum(corner.x for corner in corners) / len(corners)
        center_y = sum(corner.y for corner in corners) / len(corners)
        
        # 按角度排序
        import math
        def angle_from_center(corner):
            return math.atan2(corner.y - center_y, corner.x - center_x)
        
        sorted_corners = sorted(corners, key=angle_from_center)
        return sorted_corners
    
    def _update_image_history(self, image: np.ndarray, timestamp: float):
        """更新图像历史记录"""
        # 将图像转换为灰度以便比较
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 添加到历史记录
        self.image_history.append({
            'image': gray,
            'timestamp': timestamp
        })
        
        # 保持历史记录在合理范围内（最多保存2秒的图像）
        cutoff_time = timestamp - 2.0
        self.image_history = [item for item in self.image_history 
                             if item['timestamp'] > cutoff_time]
    
    def _is_image_stable(self) -> bool:
        """检查图像是否稳定"""
        if len(self.image_history) < 2:
            return False
        
        current_time = time.time()
        stable_duration = 0
        
        # 从最新的图像开始向前检查
        for i in range(len(self.image_history) - 1, 0, -1):
            current_frame = self.image_history[i]['image']
            previous_frame = self.image_history[i-1]['image']
            time_diff = self.image_history[i]['timestamp'] - self.image_history[i-1]['timestamp']
            
            # 计算帧差
            try:
                # 确保两帧大小相同
                if current_frame.shape != previous_frame.shape:
                    previous_frame = cv2.resize(previous_frame, 
                                              (current_frame.shape[1], current_frame.shape[0]))
                
                # 计算绝对差值
                diff = cv2.absdiff(current_frame, previous_frame)
                mean_diff = np.mean(diff)
                
                # 如果帧差小于阈值，认为是稳定的
                if mean_diff < self.frame_diff_threshold:
                    stable_duration += time_diff
                    
                    # 如果稳定时间超过阈值，返回True
                    if stable_duration >= self.stability_threshold:
                        return True
                else:
                    # 如果发现不稳定，停止检查
                    break
                    
            except Exception as e:
                # 如果比较失败，认为不稳定
                break
        
        return False
    
    def _wait_for_stable_image(self, timeout: float = 5.0) -> bool:
        """等待图像稳定"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self._is_image_stable():
                self.get_logger().info(f'Image stabilized after {time.time() - start_time:.2f}s')
                return True
            
            # 短暂休眠
            time.sleep(self.stability_check_interval)
        
        self.get_logger().warn(f'Image did not stabilize within {timeout}s timeout')
        return False

    def destroy_node(self):
        """清理节点资源"""
        if hasattr(self, 'image_processor'):
            self.image_processor.cleanup()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = VisionServicesNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
